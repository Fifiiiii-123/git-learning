/*
 * ============================================================================
 *  文件：2-苏霏霏-3226006369-物理与光电工程学院-串口通信代码.ino
 *  工程：串口通信代码（第二块 UNO R3 · 按键板）
 * ----------------------------------------------------------------------------
 *  整合自：
 *    KeypadBoard.ino        —— 任务三按键板（4 键单击，发 '1'~'4'）
 *    KeypadBoard_Task5.ino  —— 任务五按键板（4 键单击/双击/三击，发 '1'~'8'）
 *
 *  功能：同一块按键板支持两种任务模式，串口控制冲突时自动/手动切换，
 *        与主控板（meArm 控制代码）串口互相对应。
 *
 *  模式映射：
 *    MODE3（任务三 · 单击，释放发送）：
 *      键1 → '1' 循环夹取        键2 → '2' 录制开始/结束
 *      键3 → '3' 播放             键4 → '4' 回中
 *    MODE5（任务五 · 单击/双击/三击）：
 *      键1 → '1' 示教/记录
 *      键2 → '2' 暂停 / 双击 '3' 继续
 *      键3 → '4' 取消 / 双击 '5' 折线 / 三击 '6' 曲线
 *      键4 → '7' 直线 / 双击 '8' 基础图形
 *
 *  模式同步（“自己切换”，两板对应）：
 *    ① 本板是模式主控：上电默认 DEFAULT_MODE；长按 键1（≥1s）切换模式；
 *       切换时与每 3s 心跳向主控板回发 "M:T3"/"M:T5"，主控板收到后自动跟随。
 *    ② 若接回主控板 TX→本板 RX，本板还会监听主控板启动握手
 *       "MODE:T3"/"MODE:T5" 自动对齐。
 *
 *  接线（与两份原程序一致，波特率 9600）：
 *    本板 TX(D1) → 主控板 RX(D0)；本板 GND → 主控板 GND
 *    （可选）主控板 TX(D1) → 本板 RX(D0)，用于启动握手自动对齐
 *  按键：D8/D9/D10/D11，另一端接 GND，按下=低电平（内部上拉 INPUT_PULLUP）。
 * ============================================================================
 */

#define K1 8                // 键1
#define K2 9                // 键2
#define K3 10               // 键3
#define K4 11               // 键4

#define DEBOUNCE_MS  30     // 按键消抖时间（ms）
#define CLICK_WINDOW 350    // 连击判定窗口：释放后 350ms 内再按计为下一击（MODE5）
#define LONG_PRESS_MS 1000  // 长按 键1 切换模式所需时长（ms）
#define MODE_BEAT_MS  3000  // 模式心跳间隔：每 3s 向主控板重申当前模式

/* ---------------- 任务模式 ----------------
 * MODE3：任务三（单击 1~4）   MODE5：任务五（连击 1~8）
 * DEFAULT_MODE：上电默认模式；想默认任务五，把这里改成 MODE5 再烧录 */
enum KeyMode { MODE3, MODE5 };
#define DEFAULT_MODE MODE3
KeyMode mode = DEFAULT_MODE;

/* ---------------- 按键状态机（两种模式共用） ---------------- */
struct Key {
  uint8_t  pin;
  uint8_t  clicks;       // MODE5：本次连续按下的次数
  bool     stable;       // 防抖确认电平（true=松开，false=按下）
  bool     prev;         // 上一次原始电平（用于检测变化）
  uint32_t debounceAt;   // 电平变化起点
  uint32_t pressAt;      // 确认按下时刻（长按检测用）
  uint32_t releaseAt;    // 确认释放时刻（MODE5 结算窗口起点）
  bool     settle;       // MODE5：有击数、等待超窗结算
  bool     pending;      // MODE3：已确认按下、等待释放发送
  bool     longHandled;  // 本次按压已做过长按切换（避免重复触发/误发）
};

Key keys[4] = {
  { K1, 0, true, true, 0, 0, 0, false, false, false },
  { K2, 0, true, true, 0, 0, 0, false, false, false },
  { K3, 0, true, true, 0, 0, 0, false, false, false },
  { K4, 0, true, true, 0, 0, 0, false, false, false },
};

/* ==================== 模式切换 ====================
 * 切换后立即向主控板回发 "M:T3"/"M:T5"，主控板自动跟随切换 */
void toggleMode(){
  mode = (mode == MODE3) ? MODE5 : MODE3;
  Serial.print(F("M:T"));
  Serial.println(mode == MODE3 ? 3 : 5);
  Serial.print(F(">>> 已切换："));
  Serial.println(mode == MODE3
                 ? F("任务三模式（单击：1循环夹取 2录制 3播放 4回中）")
                 : F("任务五模式（连击：1示教 2暂停/3继续 4取消 5折线 6曲线 7直线 8图形）"));
}

/* ==================== 按键映射 ====================
 * 按引脚与击数查表发送指令字符；无对应映射则忽略 */
void sendKeyCmd(uint8_t pin, uint8_t n){
  char cmd = 0;
  if (mode == MODE3){                                  // 任务三：单击
    if (pin == K1 && n == 1)      cmd = '1';           // 循环夹取
    else if (pin == K2 && n == 1) cmd = '2';           // 录制开始/结束
    else if (pin == K3 && n == 1) cmd = '3';           // 播放
    else if (pin == K4 && n == 1) cmd = '4';           // 回中
  } else {                                             // 任务五：连击
    if (pin == K1 && n == 1)      cmd = '1';           // 示教/记录
    else if (pin == K2 && n == 1) cmd = '2';           // 暂停
    else if (pin == K2 && n == 2) cmd = '3';           // 继续
    else if (pin == K3 && n == 1) cmd = '4';           // 取消
    else if (pin == K3 && n == 2) cmd = '5';           // 折线绘制
    else if (pin == K3 && n == 3) cmd = '6';           // 曲线绘制
    else if (pin == K4 && n == 1) cmd = '7';           // 直线绘制
    else if (pin == K4 && n == 2) cmd = '8';           // 基础图形
  }
  if (cmd) Serial.println(cmd);                        // 发送（换行结尾，主控按行解析）
}

/* ==================== MODE3 扫描（任务三：单击，释放发送） ====================
 * 按下仅标记 pending，释放时发送指令；
 * 长按 键1 ≥1s 切换模式（取消本次发送） */
void scanMode3(Key &k, uint32_t now){
  bool raw = digitalRead(k.pin);
  /* 电平变化：记录防抖起点 */
  if (raw != k.prev){
    k.prev = raw;
    k.debounceAt = now;
    return;
  }
  /* 稳定超过 DEBOUNCE_MS 才确认电平变化 */
  if (raw != k.stable && now - k.debounceAt >= DEBOUNCE_MS){
    k.stable = raw;
    if (raw == LOW){                                   // 确认按下
      k.pending = true;
      k.pressAt = now;
      k.longHandled = false;
    } else {                                           // 确认释放 → 发送指令
      if (k.pending && !k.longHandled) sendKeyCmd(k.pin, 1);
      k.pending = false;
    }
  }
  /* 长按 键1 ≥1s：切换模式 */
  if (k.pin == K1 && k.stable == LOW && !k.longHandled &&
      now - k.pressAt >= LONG_PRESS_MS){
    k.longHandled = true;
    k.pending = false;
    toggleMode();
  }
}

/* ==================== MODE5 扫描（任务五：单击/双击/三击） ====================
 * 消抖 → 计击数 → 释放后 350ms 无新按下按总击数结算；
 * 长按 键1 ≥1s 切换模式（取消本次连击结算） */
void scanMode5(Key &k, uint32_t now){
  bool raw = digitalRead(k.pin);
  /* 电平变化：记录防抖起点 */
  if (raw != k.prev){
    k.prev = raw;
    k.debounceAt = now;
    return;
  }
  /* 稳定超过 DEBOUNCE_MS 才确认电平变化 */
  if (raw != k.stable && now - k.debounceAt >= DEBOUNCE_MS){
    k.stable = raw;
    if (raw == LOW){                                   // 确认按下：击数 +1
      k.clicks++;
      k.settle = true;
      k.pressAt = now;
      k.longHandled = false;
    } else {                                           // 确认释放：结算窗口起算
      k.releaseAt = now;
    }
  }
  /* 长按 键1 ≥1s：切换模式 */
  if (k.pin == K1 && k.stable == LOW && !k.longHandled &&
      now - k.pressAt >= LONG_PRESS_MS){
    k.longHandled = true;
    k.clicks = 0;
    k.settle = false;
    toggleMode();
  }
  /* 结算：已释放且 CLICK_WINDOW 内无新按下 */
  if (k.settle && k.stable && now - k.releaseAt >= CLICK_WINDOW){
    sendKeyCmd(k.pin, k.clicks);
    k.clicks = 0;
    k.settle = false;
  }
}

/* ==================== 监听主控板握手（可选回接线） ====================
 * 主控板启动会打印含 "MODE:T3"/"MODE:T5" 的行，收到后自动对齐模式 */
char rxBuf[16];
int  rxLen = 0;
void listenMaster(){
  while (Serial.available()){
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r'){                     // 行结束
      if (rxLen > 0){
        rxBuf[rxLen] = '\0';
        /* 在行内查找 "MODE:T3"/"MODE:T5" 子串 */
        for (int i = 0; i + 6 < rxLen; i++){
          if (rxBuf[i] == 'M' && rxBuf[i+1] == 'O' && rxBuf[i+2] == 'D' &&
              rxBuf[i+3] == 'E' && rxBuf[i+4] == ':' && rxBuf[i+5] == 'T'){
            KeyMode m = (rxBuf[i+6] == '5') ? MODE5 : MODE3;
            if (m != mode){
              mode = m;
              Serial.print(F(">>> 收到主控板握手，已对齐："));
              Serial.println(mode == MODE3 ? F("任务三模式") : F("任务五模式"));
            }
          }
        }
      }
      rxLen = 0;
    } else if (rxLen < (int)sizeof(rxBuf) - 1){
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;                                       // 溢出丢弃
    }
  }
}

/* ==================== 初始化 ==================== */
void setup(){
  pinMode(K1, INPUT_PULLUP);                           // 按下=低电平
  pinMode(K2, INPUT_PULLUP);
  pinMode(K3, INPUT_PULLUP);
  pinMode(K4, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(F("=== 串口通信代码（按键板）就绪 D8~D11 ==="));
  /* 上电先向主控板声明当前模式（若主控板未就绪，3s 心跳会补发） */
  Serial.print(F("M:T"));
  Serial.println(mode == MODE3 ? 3 : 5);
  Serial.println(F("长按 键1(≥1s) 切换任务模式；默认任务三（单击 1~4）"));
}

/* ==================== 主循环 ==================== */
void loop(){
  uint32_t now = millis();
  for (int i = 0; i < 4; i++){                         // 按当前模式扫描 4 键
    if (mode == MODE3) scanMode3(keys[i], now);
    else               scanMode5(keys[i], now);
  }
  listenMaster();                                      // 可选回接线：监听主控板握手

  /* 模式心跳：每 3s 重申当前模式，保证主控板始终跟随 */
  static uint32_t lastBeat = 0;
  if (now - lastBeat >= MODE_BEAT_MS){
    lastBeat = now;
    Serial.print(F("M:T"));
    Serial.println(mode == MODE3 ? 3 : 5);
  }
}
