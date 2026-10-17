/*
 * ============================================================================
 *  文件：2-苏霏霏-3226006369-电子科学与技术-meArm代码-任务三任务五.cpp
 *  工程：meArm 控制代码（第一块 UNO R3 · 舵机主控，标准 C++ 源码格式）
 * ----------------------------------------------------------------------------
 *  说明：独立 .cpp 源码（非 .ino 草稿），已显式包含 <Arduino.h>，
 *        并在文件上部给出全部函数声明，可直接编译烧录到 Arduino UNO R3。
 *  整合自：
 *    Task3.ino  —— 任务三：按键控制（循环夹取/录制/播放/回中）
 *    2149.ino   —— 任务五：示教 + 运动学逆解轨迹绘制（直线/折线/曲线/图形）
 *
 *  工作模式（与按键板“串口通信代码”串口对应，收到 M:T3 / M:T5 自动切换）：
 *    MODE3（任务三）：'1'循环夹取  '2'录制开始/结束  '3'播放  '4'回中
 *    MODE5（任务五）：'1'示教/记录  '2'暂停  '3'继续  '4'取消
 *                     '5'折线  '6'曲线  '7'直线  '8'基础图形
 *
 *  模式同步：
 *    上电默认 MODE3（改 DEFAULT_MODE 可换默认）；启动打印 "MODE:T3/T5" 握手行；
 *    运行中监听按键板心跳 "M:T3"/"M:T5" 自动跟随切换（长按按键板键1即可切换任务）。
 *
 *  摇杆：任务三录制时的 4 轴实时操控通道（右摇杆上下/左右 → 后臂/底座，
 *        左摇杆上下/左右 → 前臂/夹爪）；任务五示教模式（S_TEACH）下切换为
 *        纸面坐标移动笔尖（joystickTeach）。
 *
 *  硬件接线（与原程序一致）：
 *    舵机 base=D9  rArm=D8  fArm=D7  claw=D6
 *    摇杆 A3/A2/A1/A0（右摇杆上下/左右 → 后臂/底座，左摇杆上下/左右 → 前臂/夹爪）
 *    本地摇杆按键 D2=播放(仅MODE3调试) D3=回中；板载 LED D13=录制指示
 * ============================================================================
 */
#include <Arduino.h>          // Arduino 核心库（.cpp 需显式包含）
#include <Servo.h>            // Arduino 舵机驱动库（标准库）
#include <math.h>             // 数学库：acos/atan2/sqrt/degrees 等

/* ---------------- 任务模式 ----------------
 * MODE3：任务三（按键控制/录制）   MODE5：任务五（轨迹绘制）
 * DEFAULT_MODE：上电默认模式；与按键板 DEFAULT_MODE 保持一致即可 */
enum TaskMode { MODE3, MODE5 };
#define DEFAULT_MODE MODE3
TaskMode mode = DEFAULT_MODE;

/* ---------------- 引脚定义 ---------------- */
#define PIN_BASE  9           // 底座舵机 → 数字引脚 9
#define PIN_RARM  8           // 后臂舵机 → 数字引脚 8
#define PIN_FARM  7           // 前臂舵机 → 数字引脚 7
#define PIN_CLAW  6           // 夹爪舵机 → 数字引脚 6
#define PIN_SW1   2           // 左摇杆按键（本地调试：播放，ArmJoystick V3.0 固定 D2）
#define PIN_SW2   3           // 右摇杆按键（本地调试：回中，ArmJoystick V3.0 固定 D3）
#define PIN_LED   13          // 板载 LED（录制状态指示）

/* ---------------- 摇杆模拟引脚（任务三录制操控） ---------------- */
#define JOY1_X    A3          // 右摇杆上下 → 后臂
#define JOY1_Y    A2          // 右摇杆左右 → 底座
#define JOY2_X    A1          // 左摇杆上下 → 前臂
#define JOY2_Y    A0          // 左摇杆左右 → 夹爪

/* ---------------- 舵机限位 ---------------- */
#define BASE_MIN  0           // 底座角度下限
#define BASE_MAX  180         // 底座角度上限
#define RARM_MIN  45          // 后臂角度下限（对齐 JoyFK_alpha2）
#define RARM_MAX  160         // 后臂角度上限（防堵转）
#define FARM_MIN  30          // 前臂角度下限
#define FARM_MAX  110         // 前臂角度上限（对齐 JoyFK_alpha2）
#define CLAW_MIN  5           // 夹爪张开极限
#define CLAW_MAX  130         // 夹爪闭合极限（对齐 JoyFK_alpha2）
#define CLAW_HOLD 120         // 夹笔保持角：任务五绘制/示教时夹爪闭合夹稳铅笔

/* ==================== 点位定义（任务三，按实物标定！） ====================
 * 按键1 循环夹取 A/B/C 时使用；目前为示例值，联调时须按实物标定后填入。
 * 注意：结构体须定义在所有函数之前（Arduino 会自动生成函数原型） */
struct Pose { int b, r, f, c; };
const Pose HOME    = { 85, 30, 45, 60 };   // 待机位
const Pose GRAB_A  = { 46, 133, 30, 45 };  // 抓取 A
const Pose GRAB_B  = { 130, 107, 58, 45 }; // 抓取 B
const Pose GRAB_C  = { 107, 124, 52, 45 }; // 抓取 C
const Pose PLACE_A = { 126, 144, 48, 45 }; // 放置 A
const Pose PLACE_B = { 73, 155, 130, 45 }; // 放置 B
const Pose PLACE_C = { 32, 128, 31, 45 };  // 放置 C
const int GRIP_CLOSE = CLAW_MAX;           // 闭合=夹住
const int GRIP_OPEN  = CLAW_MIN;           // 张开=放下

/* ---------------- 摇杆参数（速度控制） ---------------- */
#define JOY_CENTER   512      // 摇杆模拟值中点
#define JOY_DEADBAND 25       // 死区（中位 ±25 内不动）
#define JOY_MAX_STEP 1        // 单轮最大角度增量

/* ---------------- 摇杆方向配置（任务三，按 test1.1 实测） ----------------
 * 3 轴推杆方向与期望运动相反 → 系数 -1；夹爪 +1。
 * DIR_TEACH_*：任务五示教模式的纸面坐标方向（实测 y 反了 → -1） */
#define DIR_JOY1_X  -1         // 后臂方向系数
#define DIR_JOY1_Y  -1         // 底座方向系数
#define DIR_JOY2_X  -1         // 前臂方向系数
#define DIR_JOY2_Y  +1         // 夹爪方向系数
#define DIR_TEACH_X +1         // 任务五示教：纸面 x（左右；反了改 -1）
#define DIR_TEACH_Y -1         // 任务五示教：纸面 y（前后；实测反了改 +1）

/* ---------------- 速度参数 ---------------- */
int servoDelay = 15;          // 每转 1° 的延时毫秒数（越大越慢）

/* ---------------- 录制参数（任务三） ---------------- */
#define REC_SAMPLE_MS   50    // 采样间隔 50ms → 220帧≈11秒，满足 >10s 要求
#define REC_MAX_FRAMES  220   // 最多帧数（4字节/帧，共约 880B RAM）

/* ---------------- 机械臂几何参数（任务五，对齐 JoyFK_alpha2 实物标定） ----------------
 * L1=底盘高度、L2=后臂长、L3=前臂长（mm）；PEN_Z=笔尖触纸时腕点高度（mm）；
 * 抬笔用腕点抬升 PEN_LIFT mm 实现；END_Z/END_Z_N 为画完收尾抬笔高度 */
#define L1  38.0f
#define L2  80.0f
#define L3  80.0f
#define PEN_Z  20.0f
#define PEN_LIFT  8
#define END_Z   36.0f
#define END_Z_N 70.0f

/* ---------------- 示教起点（任务五，模型坐标 mm） ---------------- */
#define TEACH_START_X  0.0f     // 示教起点 x（左右）
#define TEACH_START_Y  50.0f    // 示教起点 y（前后）

/* ---------------- 舵机对象与当前角度 ---------------- */
Servo base, rArm, fArm, claw; // 4 个舵机对象
int curBase = 90;             // 底座当前角度
int curRArm = 90;             // 后臂当前角度
int curFArm = 90;             // 前臂当前角度
int curClaw = 90;             // 夹爪当前角度

/* ==================== 状态机（任务五） ====================
 * S_IDLE 待机 / S_TEACH 示教 / S_DRAWING 绘制中 / S_PAUSED 暂停 */
enum State { S_IDLE, S_TEACH, S_DRAWING, S_PAUSED };
State state = S_IDLE;

/* ---------------- 示教点数据（任务五） ---------------- */
struct Point3D { float x, y, z; };       // 三维纸面坐标（mm）
Point3D teachPts[5];                     // 示教点数组（最多 5 点）
int  teachCount = 0;                     // 已记录的示教点数
bool penDown = false;                    // 笔尖当前是否落笔

/* 从 Flash(PROGMEM) 读取一个三维点：绘制坐标表是只读常量，
 * 用 PROGMEM 存进 Flash 可省约 324B RAM（Uno 只有 2KB SRAM） */
Point3D flashPoint(const Point3D *p){
  Point3D r;
  r.x = pgm_read_float(&p->x);
  r.y = pgm_read_float(&p->y);
  r.z = pgm_read_float(&p->z);
  return r;
}

/* ---------------- 示教摇杆中心校准值（任务五） ---------------- */
int joyCX = JOY_CENTER;       // A2（纸面 x）中心校准值
int joyCY = JOY_CENTER;       // A3（纸面 y）中心校准值

/* ---------------- 录制帧缓冲（任务三） ---------------- */
struct Frame { uint8_t b, r, f, c; };    // 一帧 = 底座/后臂/前臂/夹爪角度
Frame frames[REC_MAX_FRAMES];           // 帧缓冲数组
int   recCount = 0;                     // 已录制的帧数
bool  recording = false;                // 是否正在录制
bool  hasRecord = false;                // 是否已有可播放的数据

/* ---------------- 串口行缓冲 ---------------- */
char lineBuf[32];
int  lineLen = 0;

/* ==================== 函数声明（.cpp 无 .ino 自动原型，显式声明保证任意调用顺序） ==================== */
Point3D flashPoint(const Point3D *p);
int clampVal(int v, int lo, int hi);
void syncMove(int tBase, int tRArm, int tFArm, int tClaw, int stepMs);
void moveTo(const Pose &p, int stepMs);
void centerAll();
int axisDelta(int pin);
void joystickControl();
void setMode(TaskMode m);
void pickAndPlace(const Pose &grab, const Pose &place, int lift = 15);
void button1LoopPick();
void button2Record();
void captureFrame();
void button3Play();
void checkLocalButtons();
void forwardKin(float t1Deg, float aDeg, float bDeg, float &mx, float &my, float &mz);
void servoToGeo(float &t1, float &alpha, float &beta);
void printPenPos();
bool solveIK(float x, float y, float z, int &bAng, int &rAng, int &fAng);
bool movePenTo(float x, float y, bool down);
bool moveTo3D(float x, float y, float z);
void drawLineTo(Point3D p0, Point3D p1);
void demoDrawN();
void drawLineDemo();
void drawPolyline();
void drawCurve();
int readJoyAvg(int pin);
void enterTeach();
void joystickTeach();
void teachOrRecord();
void handleCommand(const char *line);
void readSerialLine();
void setup();
void loop();

/* ==================== 基础工具函数 ==================== */

/* 角度限位钳制：把 v 限制在 [lo, hi] 内 */
int clampVal(int v, int lo, int hi){
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==================== 多舵机同步移动 ====================
 * 功能：4 个关节同步逐度运动到目标角度（带限位保护）
 * 说明：任务五绘制/暂停状态下，每度之间都处理一次串口——
 *       暂停('2')、继续('3')、取消('4')、模式切换即时生效 */
void syncMove(int tBase, int tRArm, int tFArm, int tClaw, int stepMs){
  bool outOfLimit = false;
  if (tBase < BASE_MIN || tBase > BASE_MAX){ tBase = clampVal(tBase, BASE_MIN, BASE_MAX); outOfLimit = true; }
  if (tRArm < RARM_MIN || tRArm > RARM_MAX){ tRArm = clampVal(tRArm, RARM_MIN, RARM_MAX); outOfLimit = true; }
  if (tFArm < FARM_MIN || tFArm > FARM_MAX){ tFArm = clampVal(tFArm, FARM_MIN, FARM_MAX); outOfLimit = true; }
  if (tClaw < CLAW_MIN || tClaw > CLAW_MAX){ tClaw = clampVal(tClaw, CLAW_MIN, CLAW_MAX); outOfLimit = true; }
  if (outOfLimit) Serial.println(F("! 部分目标角度超出限位，已自动限制"));

  bool done = false;
  while (!done){
    done = true;
    if (curBase != tBase){ curBase += (tBase > curBase) ? 1 : -1; base.write(curBase);  done = false; }
    if (curRArm != tRArm){ curRArm += (tRArm > curRArm) ? 1 : -1; rArm.write(curRArm); done = false; }
    if (curFArm != tFArm){ curFArm += (tFArm > curFArm) ? 1 : -1; fArm.write(curFArm); done = false; }
    if (curClaw != tClaw){ curClaw += (tClaw > curClaw) ? 1 : -1; claw.write(curClaw); done = false; }
    delay(stepMs);
    if (state == S_DRAWING || state == S_PAUSED){    // 任务五绘制中：按键即时生效
      readSerialLine();
      if (state == S_PAUSED){                        // 暂停：原地等待（期间仍处理串口）
        while (state == S_PAUSED) { readSerialLine(); delay(20); }
      }
      if (state != S_DRAWING) return;                // 取消：立即停止移动
    }
  }
}

/* 整体移动到某点位（任务三） */
void moveTo(const Pose &p, int stepMs){ syncMove(p.b, p.r, p.f, p.c, stepMs); }

/* ==================== 回中 ====================
 * 所有关节回到初始姿态；任务五夹爪保持闭合夹笔，任务三夹爪张开 */
void centerAll(){
  Serial.println(F(">>> 回中"));
  if (mode == MODE5) syncMove(90, 90, 90, CLAW_HOLD, servoDelay);
  else               syncMove(90, 90, 90, 30, servoDelay);
}

/* ==================== 摇杆轴增量 ====================
 * 死区内返回 0，越推越远返回越大（±1 ~ ±JOY_MAX_STEP） */
int axisDelta(int pin){
  int v = analogRead(pin);
  if (v > JOY_CENTER + JOY_DEADBAND){
    int span = 1023 - (JOY_CENTER + JOY_DEADBAND);
    if (span <= 0) return 1;
    return map(v - (JOY_CENTER + JOY_DEADBAND), 0, span, 1, JOY_MAX_STEP);
  }
  if (v < JOY_CENTER - JOY_DEADBAND){
    int span = JOY_CENTER - JOY_DEADBAND;
    if (span <= 0) return -1;
    return -map((JOY_CENTER - JOY_DEADBAND) - v, 0, span, 1, JOY_MAX_STEP);
  }
  return 0;
}

/* ==================== 摇杆实时操控（任务三录制操控通道） ====================
 * 4 轴速度控制：右摇杆上下→后臂、左右→底座；左摇杆上下→前臂、左右→夹爪。
 * 任务三录制期间用它操控机械臂并定时采样。 */
void joystickControl(){
  int dRArm = DIR_JOY1_X * axisDelta(JOY1_X);     // 后臂增量
  int dBase = DIR_JOY1_Y * axisDelta(JOY1_Y);     // 底座增量
  int dFArm = DIR_JOY2_X * axisDelta(JOY2_X);     // 前臂增量
  int dClaw = DIR_JOY2_Y * axisDelta(JOY2_Y);     // 夹爪增量
  if (dBase || dRArm || dFArm || dClaw){
    if (dBase){ curBase = clampVal(curBase + dBase, BASE_MIN, BASE_MAX); base.write(curBase); }
    if (dRArm){ curRArm = clampVal(curRArm + dRArm, RARM_MIN, RARM_MAX); rArm.write(curRArm); }
    if (dFArm){ curFArm = clampVal(curFArm + dFArm, FARM_MIN, FARM_MAX); fArm.write(curFArm); }
    if (dClaw){ curClaw = clampVal(curClaw + dClaw, CLAW_MIN, CLAW_MAX); claw.write(curClaw); }
    delay(servoDelay);                            // 每步延时 = 当前速度档
  }
}

/* ==================== 模式切换 ====================
 * 收到按键板心跳 M:T3/M:T5 时调用；切到任务五时终止可能存在的录制 */
void setMode(TaskMode m){
  if (m == mode) return;
  mode = m;
  if (mode == MODE5){                            // 切到任务五：终止录制
    recording = false;
    hasRecord = false;
    digitalWrite(PIN_LED, LOW);
  }
  Serial.print(F(">>> 主控切换至 "));
  Serial.println(mode == MODE5 ? F("任务五（轨迹绘制）") : F("任务三（按键控制/录制）"));
}

/* ==================== 任务三：夹取并放置 ====================
 * 完整动作序列：回待机→下降抓取→闭合→抬升→平移→下降→张开→回待机 */
void pickAndPlace(const Pose &grab, const Pose &place, int lift){
  moveTo(HOME, servoDelay);                      // 回待机
  int hiR = grab.r;                              // 高度位后臂角
  int hiF = grab.f - lift;                       // 高度位前臂角（上抬）
  syncMove(grab.b, hiR, hiF, GRIP_OPEN, servoDelay);   // 到抓取点上方
  syncMove(grab.b, grab.r, grab.f, GRIP_OPEN, servoDelay); // 下降
  delay(300);
  syncMove(grab.b, grab.r, grab.f, GRIP_CLOSE, servoDelay); // 闭合夹爪
  delay(300);
  syncMove(grab.b, hiR, hiF, GRIP_CLOSE, servoDelay);   // 抬升
  syncMove(place.b, hiR, hiF, GRIP_CLOSE, servoDelay);  // 平移
  syncMove(place.b, place.r, place.f, GRIP_CLOSE, servoDelay); // 下降
  delay(300);
  syncMove(place.b, place.r, place.f, GRIP_OPEN, servoDelay); // 张开放下
  delay(300);
  moveTo(HOME, servoDelay);                      // 回待机
}

/* ==================== 任务三：循环夹取（按键 '1'） ====================
 * 依次执行 夹取A→夹取B→夹取C→夹取A→… 循环 */
int pickCycle = 0;            // 0→A 1→B 2→C 循环计数
void button1LoopPick(){
  Serial.print(F(">>> 循环夹取 第")); Serial.print(pickCycle + 1); Serial.println(F(" 个物体"));
  switch (pickCycle){
    case 0: pickAndPlace(GRAB_A, PLACE_A); break;
    case 1: pickAndPlace(GRAB_B, PLACE_B); break;
    default: pickAndPlace(GRAB_C, PLACE_C); break;
  }
  pickCycle = (pickCycle + 1) % 3;               // 循环
}

/* ==================== 任务三：录制开始/结束（按键 '2'） ==================== */
void button2Record(){
  if (!recording){                              // 未在录制 → 开始
    recCount = 0;
    recording = true;
    hasRecord = false;
    digitalWrite(PIN_LED, HIGH);                // LED 常亮 = 录制中
    Serial.println(F(">>> 录制开始：请摇杆操控机械臂（>10秒明显位移），再按一次结束"));
  } else {                                      // 录制中 → 结束
    recording = false;
    hasRecord = (recCount > 0);
    digitalWrite(PIN_LED, LOW);
    Serial.print(F(">>> 录制结束，共 ")); Serial.print(recCount); Serial.println(F(" 帧"));
  }
}

/* ==================== 录制采样 ==================== */
void captureFrame(){
  if (recCount < REC_MAX_FRAMES){
    frames[recCount].b = curBase;
    frames[recCount].r = curRArm;
    frames[recCount].f = curFArm;
    frames[recCount].c = curClaw;
    recCount++;
  }
}

/* ==================== 任务三：播放（按键 '3'） ==================== */
void button3Play(){
  if (!hasRecord){
    Serial.println(F("! 没有可播放的录制数据"));
    return;
  }
  Serial.print(F(">>> 播放 ")); Serial.print(recCount); Serial.println(F(" 帧"));
  for (int i = 0; i < recCount; i++){
    syncMove(frames[i].b, frames[i].r, frames[i].f, frames[i].c, 5);
    delay(1);
  }
  Serial.println(F(">>> 播放完成"));
}

/* ==================== 本地按键检测（任务三调试备用，仅 MODE3 有效） ====================
 * D2=左摇杆按键=播放   D3=右摇杆按键=回中（防抖 300ms） */
void checkLocalButtons(){
  static unsigned long lastSw1 = 0, lastSw2 = 0;
  unsigned long now = millis();
  if (mode != MODE3) return;                     // 任务五模式下不启用，避免干扰绘制
  if (digitalRead(PIN_SW1) == LOW && now - lastSw1 > 300){ lastSw1 = now; button3Play(); }
  if (digitalRead(PIN_SW2) == LOW && now - lastSw2 > 300){ lastSw2 = now; centerAll(); }
}

/* ==================== 任务五：运动学正解（对齐 JoyFK_alpha2） ==================== */
void forwardKin(float t1Deg, float aDeg, float bDeg,
                float &mx, float &my, float &mz){
  float t1 = radians(t1Deg);
  float a  = radians(aDeg);
  float b  = radians(bDeg);
  float r  = L2 * cos(a) - L3 * cos(a + b);
  mx = -r * sin(t1);
  my = r * cos(t1);
  mz = L1 + L2 * sin(a) - L3 * sin(a + b);
}

/* 舵机角 → 几何角 */
void servoToGeo(float &t1, float &alpha, float &beta){
  t1    = (float)(curBase - 90);
  alpha = (float)(180 - curRArm);
  beta  = (float)curFArm;
}

/* 串口打印当前笔尖正解坐标（调试/示教验证用） */
void printPenPos(){
  float t1, a, b, mx, my, mz;
  servoToGeo(t1, a, b);
  forwardKin(t1, a, b, mx, my, mz);
  Serial.print(F("  笔尖(x,y,z): "));
  Serial.print(mx, 1); Serial.print(F(", "));
  Serial.print(my, 1); Serial.print(F(", "));
  Serial.println(mz, 1);
}

/* ==================== 任务五：运动学逆解（与正解互逆） ====================
 * 输入纸面坐标 (x,y) 与腕点高度 z，输出舵机角；不可达返回 false */
bool solveIK(float x, float y, float z, int &bAng, int &rAng, int &fAng){
  float R = sqrt(x * x + y * y);
  float Z = z - L1;
  float d = sqrt(R * R + Z * Z);
  if (d > L2 + L3 - 1.0f || d < fabs(L2 - L3) + 1.0f){
    return false;                                // 超出工作空间
  }
  float cosB = (L2 * L2 + L3 * L3 - d * d) / (2.0f * L2 * L3);
  if (cosB > 1.0f) cosB = 1.0f;
  if (cosB < -1.0f) cosB = -1.0f;
  float beta  = acos(cosB);
  float phi   = atan2(Z, R);
  float alpha = phi + atan2(L3 * sin(beta), L2 - L3 * cos(beta));
  float rChk = L2 * cos(alpha) - L3 * cos(alpha + beta);
  if (rChk < 5.0f) return false;                 // 肘朝后奇异姿态
  float t1 = -atan2(x, y);
  bAng = (int)round(degrees(t1) + 90.0f);
  rAng = (int)round(180.0f - degrees(alpha));
  fAng = (int)round(degrees(beta));
  bAng = clampVal(bAng, BASE_MIN, BASE_MAX);
  rAng = clampVal(rAng, RARM_MIN, RARM_MAX);
  fAng = clampVal(fAng, FARM_MIN, FARM_MAX);
  /* 回代验证：限位钳制后实际到不了目标则判不可达 */
  float vT1  = radians((float)bAng - 90.0f);
  float vAlp = radians(180.0f - (float)rAng);
  float vBet = radians((float)fAng);
  float vr   = L2 * cos(vAlp) - L3 * cos(vAlp + vBet);
  float vmx  = -vr * sin(vT1);
  float vmy  = vr * cos(vT1);
  if (sqrt((vmx - x) * (vmx - x) + (vmy - y) * (vmy - y)) > 5.0f){
    return false;
  }
  return true;
}

/* ==================== 笔尖移动到纸面坐标 ====================
 * down=true 落笔（腕点贴纸面），false 抬笔（腕点抬升 PEN_LIFT mm） */
bool movePenTo(float x, float y, bool down){
  int b, r, f;
  float zT = down ? PEN_Z : PEN_Z + PEN_LIFT;
  if (!solveIK(x, y, zT, b, r, f)){
    Serial.print(F("! 目标不可达 (")); Serial.print(x); Serial.print(F(","));
    Serial.print(y); Serial.println(F(")"));
    return false;
  }
  syncMove(b, r, f, CLAW_HOLD, servoDelay);      // 夹爪闭合夹稳铅笔
  penDown = down;
  return true;
}

/* ==================== 三维坐标移动（z 由调用方显式给定） ==================== */
bool moveTo3D(float x, float y, float z){
  int b, r, f;
  if (!solveIK(x, y, z, b, r, f)){
    Serial.print(F("! 目标不可达 (")); Serial.print(x); Serial.print(F(","));
    Serial.print(y); Serial.print(F(",")); Serial.println(z);
    return false;
  }
  syncMove(b, r, f, CLAW_HOLD, servoDelay);
  return true;
}

/* ---------------- 插补绘制参数 ---------------- */
#define DRAW_STEPS 40   // 线段插补步数

/* ==================== 两点直线插补绘制 ==================== */
void drawLineTo(Point3D p0, Point3D p1){
  movePenTo(p0.x, p0.y, false);                  // 抬笔移到起点
  movePenTo(p0.x, p0.y, true);                   // 落笔
  for (int i = 1; i <= DRAW_STEPS; i++){
    readSerialLine();                            // 每步处理按键：暂停/取消即时生效
    if (state == S_PAUSED){
      while (state == S_PAUSED) { readSerialLine(); delay(20); }
    }
    if (state != S_DRAWING) return;              // 取消：退出绘制
    float t = (float)i / DRAW_STEPS;
    float x = p0.x + (p1.x - p0.x) * t;
    float y = p0.y + (p1.y - p0.y) * t;
    if (!movePenTo(x, y, true)) break;
  }
  moveTo3D(p1.x, p1.y, END_Z);                   // 画完抬笔
}

/* ==================== 基础图形：字母 N（任务五子任务2） ====================
 * 坐标表存于 Flash（PROGMEM），读取时用 flashPoint 辅助函数 */
void demoDrawN(){
  static const PROGMEM Point3D pts[3][7] = {
    {  // 边1：左竖线
      { -15, 58, 20 }, { -15, 68, 17 }, { -15, 78, 15 }, { -15, 88, 10 },
      { -18, 98, 7 }, { -18, 104, 5 }, { -18, 110, 5 }
    },
    {  // 边2：斜线
      { -16, 105, 5 }, { -13, 100, 5 }, {  -11, 90, 7 }, {  -9, 80, 10 },
      {   -7, 70, 15 }, {  -5, 62, 17 }, {  -3, 58, 20 }
    },
    {  // 边3：右竖线
      {  -5, 62, 20 }, {  -5, 68, 17 }, {  -5, 78, 15 }, {  -7, 88, 10 },
      {  -7, 98, 7 }, {  -8, 104, 5 }, {  -9, 110, 5 }
    }
  };
  for (int e = 0; e < 3; e++){
    for (int i = 0; i < 7; i++){
      readSerialLine();
      if (state == S_PAUSED){
        while (state == S_PAUSED) { readSerialLine(); delay(20); }
      }
      if (state != S_DRAWING){
        Serial.print(F("! 绘制中途退出 state=")); Serial.println(state);
        return;
      }
      if (!moveTo3D(flashPoint(&pts[e][i]).x,
                    flashPoint(&pts[e][i]).y,
                    flashPoint(&pts[e][i]).z)) break;
    }
    Serial.print(F(">>> 边")); Serial.print(e + 1); Serial.println(F(" 完成"));
  }
  Point3D pN = flashPoint(&pts[2][6]);
  moveTo3D(pN.x, pN.y, END_Z_N);                 // N 专用：收笔抬得更高
  Serial.println(F(">>> N 绘制完成，笔已抬起"));
}

/* ==================== 任务五子任务一：6 坐标直线绘制 ====================
 * 坐标表存于 Flash（PROGMEM） */
void drawLineDemo(){
  static const PROGMEM Point3D pts[6] = {
    { 0, 40, 28 },   // ① 起点上方（抬笔移入）
    { 0, 50, 20 },   // ② 下笔到纸面
    { 0, 60, 15 },   // ③ 画线
    { 0, 70, 7 },    // ④ 画线
    { 0, 80, 0 },    // ⑤ 画线
    { 0, 90, 20 }    // ⑥ 终点上方（抬笔收笔）
  };
  for (int i = 0; i < 6; i++){
    readSerialLine();
    if (state == S_PAUSED){
      while (state == S_PAUSED) { readSerialLine(); delay(20); }
    }
    if (state != S_DRAWING) return;
    if (!moveTo3D(flashPoint(&pts[i]).x,
                  flashPoint(&pts[i]).y,
                  flashPoint(&pts[i]).z)) break;
  }
  moveTo3D(pts[5].x, pts[5].y, END_Z_N);
}

/* ==================== 任务五子任务4：五点折线绘制 ==================== */
void drawPolyline(){
  if (teachCount < 2){
    Serial.println(F("! 示教点不足，先进入示教模式记录点"));
    return;
  }
  Serial.println(F(">>> 折线绘制开始"));
  centerAll();
  for (int i = 0; i + 1 < teachCount; i++){
    if (state != S_DRAWING) break;
    drawLineTo(teachPts[i], teachPts[i + 1]);
  }
  moveTo3D(teachPts[teachCount - 1].x, teachPts[teachCount - 1].y, END_Z);
  Serial.println(F(">>> 折线绘制结束"));
}

/* ==================== 任务五子任务5：五点平滑曲线绘制（Catmull-Rom） ==================== */
void drawCurve(){
  if (teachCount < 3){
    Serial.println(F("! 平滑曲线至少需要 3 个点"));
    return;
  }
  Serial.println(F(">>> 曲线绘制开始"));
  centerAll();
  const int SEGS = 24;
  for (int i = 0; i + 3 < teachCount + 2; i++){
    if (state != S_DRAWING) break;
    Point3D p0 = (i - 1 >= 0) ? teachPts[i - 1] : teachPts[0];
    Point3D p1 = teachPts[i];
    Point3D p2 = (i + 1 < teachCount) ? teachPts[i + 1] : teachPts[teachCount - 1];
    Point3D p3 = (i + 2 < teachCount) ? teachPts[i + 2] : teachPts[teachCount - 1];
    float prevX = p1.x, prevY = p1.y;
    for (int s = 1; s <= SEGS; s++){
      readSerialLine();
      if (state == S_PAUSED){
        while (state == S_PAUSED) { readSerialLine(); delay(20); }
      }
      if (state != S_DRAWING) return;
      float t = (float)s / SEGS;
      float t2 = t * t, t3 = t2 * t;
      float x = 0.5f * ((2 * p1.x) + (-p0.x + p2.x) * t +
                        (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                        (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
      float y = 0.5f * ((2 * p1.y) + (-p0.y + p2.y) * t +
                        (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                        (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
      Point3D np = { x, y, PEN_Z };
      if (s == 1) movePenTo(prevX, prevY, false);
      movePenTo(np.x, np.y, true);
    }
  }
  moveTo3D(teachPts[teachCount - 1].x, teachPts[teachCount - 1].y, END_Z);
  Serial.println(F(">>> 曲线绘制结束"));
}

/* ==================== 摇杆中心采样（任务五示教校准） ==================== */
int readJoyAvg(int pin){
  long sum = 0;
  for (int i = 0; i < 16; i++){
    sum += analogRead(pin);
    delay(2);
  }
  return (int)(sum / 16);
}

/* ==================== 进入示教模式（任务五按键 '1' 首次） ==================== */
void enterTeach(){
  teachCount = 1;                                // 预置起点即第 1 个点
  teachPts[0] = { TEACH_START_X, TEACH_START_Y, PEN_Z };
  state = S_TEACH;
  Serial.println(F(">>> 正在校准摇杆中心，请松开摇杆不要碰..."));
  delay(500);
  joyCX = readJoyAvg(JOY1_Y);                    // A2 → 纸面 x 轴中心
  joyCY = readJoyAvg(JOY1_X);                    // A3 → 纸面 y 轴中心
  Serial.print(F("    校准完成 中心x=")); Serial.print(joyCX);
  Serial.print(F(" 中心y=")); Serial.println(joyCY);
  movePenTo(TEACH_START_X, TEACH_START_Y, false);
  Serial.println(F(">>> 示教模式：摇杆移动笔尖，按键1单击记录（最多5点）；按键3双击=折线 / 三击=曲线"));
  Serial.println(F("    注意：只能往机械臂前方/两侧推（y 保持正值）；方向反了就调 DIR_TEACH_X/Y"));
  Serial.print(F("    当前笔尖(起点)坐标：")); printPenPos();
}

/* ==================== 示教摇杆操控（任务五，纸面坐标移动笔尖） ====================
 * 右摇杆上下(A3)→纸面y前后(-1)、左右(A2)→纸面x左右(+1)，速度控制逐度逼近 */
void joystickTeach(){
  int vx = analogRead(JOY1_Y);                    // A2 原始值（纸面 x）
  int vy = analogRead(JOY1_X);                    // A3 原始值（纸面 y）
  int dx = 0, dy = 0;
  if (vx > joyCX + JOY_DEADBAND)      dx = +1;
  else if (vx < joyCX - JOY_DEADBAND) dx = -1;
  if (vy > joyCY + JOY_DEADBAND)      dy = +1;
  else if (vy < joyCY - JOY_DEADBAND) dy = -1;
  static unsigned long lastChgVx = 0, lastChgVy = 0;
  static int lvx = 0, lvy = 0;
  if (abs(vx - lvx) > 5) lastChgVx = millis();
  if (abs(vy - lvy) > 5) lastChgVy = millis();
  lvx = vx; lvy = vy;
  int dX = DIR_TEACH_X * dx;                      // 纸面 x 方向系数 +1
  int dY = DIR_TEACH_Y * dy;                      // 纸面 y 方向系数 -1
  if (dX || dY){
    float tx = teachPts[teachCount - 1].x + dX * 2.0f;
    float ty = teachPts[teachCount - 1].y + dY * 2.0f;
    int b, r, f;
    if (solveIK(tx, ty, PEN_Z + PEN_LIFT, b, r, f)){
      if (curBase != b){ curBase += (b > curBase) ? 1 : -1; base.write(curBase); }
      if (curRArm != r){ curRArm += (r > curRArm) ? 1 : -1; rArm.write(curRArm); }
      if (curFArm != f){ curFArm += (f > curFArm) ? 1 : -1; fArm.write(curFArm); }
      float t1, a, b2, mx, my, mz;
      servoToGeo(t1, a, b2);
      forwardKin(t1, a, b2, mx, my, mz);
      teachPts[teachCount - 1].x = mx;            // 记录真实到达位置
      teachPts[teachCount - 1].y = my;
    } else {
      Serial.println(F("! 该方向越界不可达"));
    }
    delay(servoDelay);
  }
}

/* ==================== 任务五：示教/记录（按键 '1' / 调试 't'） ==================== */
void teachOrRecord(){
  if (state != S_TEACH){
    enterTeach();                                  // 未在示教 → 进入
  } else {
    if (teachCount < 5){
      if (teachCount > 0){
        teachPts[teachCount] = teachPts[teachCount - 1];
      }
      teachCount++;
      Serial.print(F(">>> 已记录示教点 ")); Serial.println(teachCount);
    } else {
      Serial.println(F("! 已达 5 点上限"));
    }
  }
}

/* ==================== 串口指令解析（总入口） ====================
 * 分发顺序：
 *   ① 模式心跳 "M:T3"/"M:T5"（按键板 → 本板）
 *   ② 单字符指令（按当前模式分发 '1'~'8'）
 */
void handleCommand(const char *line){
  int len = strlen(line);
  if (len == 0) return;

  /* ① 模式切换指令（按键板心跳/手动切换） */
  if (len >= 4 && line[0] == 'M' && line[1] == ':' && line[2] == 'T' &&
      (line[3] == '3' || line[3] == '5')){
    setMode(line[3] == '5' ? MODE5 : MODE3);
    return;
  }

  char c = tolower(line[0]);

  /* ② 单字符指令（按键板按键 '1'~'8' 或调试字符） */
  if (len == 1){
    if (mode == MODE3){
      switch (c){                                   /* 任务三指令 */
        case '1': button1LoopPick(); break;         // 循环夹取
        case '2': button2Record();   break;         // 录制开始/结束
        case '3': button3Play();     break;         // 播放
        case '4': centerAll();       break;         // 回中
        default:  Serial.println(F("! 未知指令（MODE3：1循环夹取 2录制 3播放 4回中）"));
      }
    } else {                                        /* 任务五指令 */
      switch (c){
        case '1':
        case 't': teachOrRecord(); break;           // 示教/记录（t：串口调试别名）
        case '2': if (state == S_DRAWING){ state = S_PAUSED; Serial.println(F(">>> 已暂停")); } break;
        case '3': if (state == S_PAUSED){ state = S_DRAWING; Serial.println(F(">>> 已继续")); } break;
        case '4': state = S_IDLE; Serial.println(F(">>> 已取消，回待机")); centerAll(); break;
        case '5': if (state == S_TEACH){ state = S_DRAWING; drawPolyline(); state = S_IDLE; } break;
        case '6': if (state == S_TEACH){ state = S_DRAWING; drawCurve(); state = S_IDLE; } break;
        case '7': if (state != S_IDLE){ Serial.println(F("! 绘制/示教中，忽略")); break; }
                  state = S_DRAWING; drawLineDemo(); state = S_IDLE; break;
        case '8': if (state != S_IDLE){ Serial.println(F("! 绘制/示教中，忽略")); break; }
                  state = S_DRAWING; demoDrawN(); state = S_IDLE; break;
        case 'p': if (state == S_DRAWING){ state = S_PAUSED; Serial.println(F(">>> 已暂停")); }
                  else if (state == S_PAUSED){ state = S_DRAWING; Serial.println(F(">>> 已继续")); }
                  break;                            // p：串口调试 暂停/继续 切换
        default:  Serial.println(F("! 指令：1示教/记录 2暂停 3继续 4取消 5折线 6曲线 7直线 8图形"));
      }
    }
    return;
  }
  Serial.println(F("! 未知指令"));
}

/* ==================== 串口逐行读取 ====================
 * 单字符按键指令 '1'~'8' 收到即处理（不等换行，暂停/继续/取消零延迟）；
 * 其他指令（M:T3、M:T5）拼成完整行后处理 */
void readSerialLine(){
  while (Serial.available()){
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r'){
      if (lineLen > 0){
        lineBuf[lineLen] = '\0';
        handleCommand(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < (int)sizeof(lineBuf) - 1){
      lineBuf[lineLen++] = ch;
      if (lineLen == 1 && ch >= '1' && ch <= '8'){
        lineBuf[1] = '\0';
        handleCommand(lineBuf);
        lineLen = 0;
      }
    } else {
      lineLen = 0;                                  // 溢出丢弃
    }
  }
}

/* ==================== 初始化 ==================== */
void setup(){
  pinMode(PIN_SW1, INPUT_PULLUP);                   // D2 输入上拉
  pinMode(PIN_SW2, INPUT_PULLUP);                   // D3 输入上拉
  pinMode(PIN_LED, OUTPUT);                         // LED 输出
  digitalWrite(PIN_LED, LOW);

  base.attach(PIN_BASE);  delay(200);
  rArm.attach(PIN_RARM);  delay(200);
  fArm.attach(PIN_FARM);  delay(200);
  claw.attach(PIN_CLAW);  delay(200);

  base.write(90); rArm.write(90); fArm.write(90); claw.write(90);
  delay(300);

  Serial.begin(9600);
  Serial.println(F("=== meArm 控制代码（任务三/任务五整合版）==="));
  Serial.print(F("MODE:T"));                        // 启动握手：按键板接回 TX 可自动对齐
  Serial.println(mode == MODE5 ? 5 : 3);
  Serial.println(F("MODE3：1循环夹取 2录制 3播放 4回中 | MODE5：1示教 2暂停 3继续 4取消 5折线 6曲线 7直线 8图形"));
}

/* ==================== 主循环 ====================
 * 每轮：
 *   1. 处理串口指令（含按键板模式心跳）
 *   2. 本地按键检测（仅 MODE3）
 *   3. 任务五示教：摇杆移动笔尖；否则摇杆操控（任务三录制同通道）
 *   4. 任务三录制：定时采样记录 */
void loop(){
  readSerialLine();                                 // 串口指令 + 模式心跳
  checkLocalButtons();                              // 本地摇杆按键（仅 MODE3）

  if (state == S_TEACH){
    joystickTeach();                                // 任务五示教：纸面坐标移动笔尖
  } else if (state == S_IDLE){
    joystickControl();                              // 任务三录制操控通道
    if (recording){                                 // 任务三录制采样
      static unsigned long lastSample = 0;
      if (millis() - lastSample >= REC_SAMPLE_MS){
        lastSample = millis();
        captureFrame();
        if (recCount >= REC_MAX_FRAMES){            // 帧数已满自动结束
          recording = false;
          hasRecord = true;
          digitalWrite(PIN_LED, LOW);
          Serial.println(F("! 帧数已满，自动结束录制"));
        }
      }
    }
  }
}
