#include <POP32.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

// === GLOBAL VARIABLE ===

#define SERVO_OPEN 90
#define SERVO_CLOSE 0
#define MOTOR_SPEED 45

#define L_OFFSET 0
#define R_OFFSET 5
#define MOTOR_SPEED 55
const int REF[4] = { 770, 760, 860, 680 };

// === LOW-LEVEL FUNCTION ===

void setMotor(short L, short R) {
  motor(1, L - L_OFFSET);
  motor(3, L - L_OFFSET);
  motor(2, R - R_OFFSET);
  motor(4, R - R_OFFSET);
}

void dropCube(unsigned short pin, unsigned short cubes) {
  unsigned short angle = 20, dea = 180;
  if (pin == 2) {
    angle = 0;
    dea = 220;
  }
  for (int i = 0; i < cubes; i++) {
    servo(pin, 100);
    servo(pin, angle);
    delay(200);
    servo(pin, 100);
    delay(300);
  }
}


void setMotorMil(short L, short R, unsigned short time) {
  unsigned long start = millis();
  setMotor(L, R);
  while (millis() <= (start + time))
    ;
  ao();
}

// === IMU INNITS ===

BNO08x imu;

// ---------------- ค่าตั้งการต่อ IMU ----------------
const uint8_t IMU_ADDRS[] = { 0x4A, 0x4B };  // ลองทั้งสองแอดเดรสให้เอง
const int8_t IMU_INT_PIN = -1;               // -1 = ไม่ได้ต่อ
const int8_t IMU_RST_PIN = -1;               // -1 = ไม่ได้ต่อ
const uint32_t I2C_CLOCK = 100000;           // 100 kHz เสถียรสุด ถ้านิ่งแล้วค่อยลอง 400000

// ---------------- ค่าปรับจูน ----------------
const float KP = 2;  // ยิ่งมาก ยิ่งหมุนแรงตอนห่างเป้า
// ทิศของมุมจาก IMU: ต้องให้ "หมุนขวา = มุมเพิ่มขึ้น"
// ถ้าหุ่นวิ่งหนีเป้า (err โตขึ้นเรื่อยๆ จนไปติดแถว ±180) ให้สลับเป็น -1.0 / 1.0
const float YAW_SIGN = 1.0;

// วิ่งตรง
const float KP_STRAIGHT = 2.0;  // ยิ่งมาก ยิ่งแก้ทิศแรง (ถ้าส่ายซ้ายขวาให้ลด)
const int MAX_CORR = 25;        // จำกัดแรงแก้ทิศสูงสุด
const bool DEBUG_TURN = true;   // true = พิมพ์ err/speed ออก Serial ตอนเลี้ยว (ปิดเมื่อจูนเสร็จ)
const float KD = 0.04;          // ตัวหน่วง: ยิ่งมาก ยิ่งเบรกเร็วตอนใกล้เป้า (เริ่ม 0.03-0.1)
const int MAX_SPEED = 70;
const int MIN_SPEED = 37;
const float SLOW_ZONE_DEG = 15.0;  // เหลืออีกกี่องศาถึงเปลี่ยนจาก MAX_SPEED เป็น MIN_SPEED (ยิ่งน้อย ยิ่งใช้แรงเต็มนาน)
const float TURN_TOLERANCE = 1.5;
const unsigned long SETTLE_MS = 150;
const unsigned long TURN_TIMEOUT = 1700;

// ---------------- ตัวแปรสถานะ ----------------
float rawHeading = 0;
float headingOffset = 0;
float targetHeading = 0;

// ============================================================
//                    ส่วนจัดการ IMU
// ============================================================
void scanI2C() {
  Serial.println("Scanning I2C...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x");
      Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  no I2C device (เช็กสาย SDA/SCL/ไฟ)");
}

void setReports() {
  imu.enableGameRotationVector(10);  // อัปเดตทุก 10 ms
}

// พยายามเชื่อมต่อ IMU หลายรอบ + ลองทุกแอดเดรส
bool initIMU(int maxTries) {
  for (int t = 1; t <= maxTries; t++) {
    for (uint8_t i = 0; i < sizeof(IMU_ADDRS); i++) {
      Wire.begin();
      Wire.setClock(I2C_CLOCK);

      if (imu.begin(IMU_ADDRS[i], Wire, IMU_INT_PIN, IMU_RST_PIN)) {
        Serial.print("BNO086 ready at 0x");
        Serial.println(IMU_ADDRS[i], HEX);
        return true;
      }
      Serial.print("try ");
      Serial.print(t);
      Serial.print(" addr 0x");
      Serial.print(IMU_ADDRS[i], HEX);
      Serial.println(" : begin() failed");
      delay(300);
    }
    delay(700);
  }
  return false;
}

float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a <= -180) a += 360;
  return a;
}

bool updateIMU() {
  if (imu.wasReset()) setReports();
  if (imu.getSensorEvent() && imu.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    // getYaw() เป็นเรเดียน บวก = ทวนเข็ม จึงใส่ลบให้บวก = หมุนขวา
    rawHeading = YAW_SIGN * imu.getYaw() * 180.0 / PI;
    return true;
  }
  return false;
}

float getHeading() {
  updateIMU();
  return wrap180(rawHeading - headingOffset);
}

// ตั้งศูนย์ เหมือนรีเซ็ตเข็มทิศ
void zeroHeading() {
  unsigned long t0 = millis();
  while (millis() - t0 < 300) updateIMU();
  headingOffset = rawHeading;
  targetHeading = 0;
}

// ============================================================
//                    ส่วนควบคุมมอเตอร์
// ============================================================
// มอเตอร์: 1 = ซ้ายหน้า, 2 = ขวาหน้า, 3 = ซ้ายหลัง, 4 = ขวาหลัง
// ถ้าล้อหลังหมุนสวนทางกับล้อหน้า ให้สลับ SIGN ของล้อนั้นเป็น -1
const int M3_SIGN = 1;
const int M4_SIGN = 1;

// left/right: บวก = ล้อเดินหน้า
void setDrive(int left, int right) {
  motor(1, left);
  motor(3, M3_SIGN * left);
  motor(2, right);
  motor(4, M4_SIGN * right);
}

void spin(int speed) {  // บวก = หมุนขวา, ลบ = หมุนซ้าย
  setDrive(speed, -speed);
}

void turnToHeading(float target) {
  target = wrap180(target);
  targetHeading = target;

  unsigned long t0 = millis();
  unsigned long inTolSince = 0;

  float err = wrap180(target - getHeading());
  float prevErr = err;
  unsigned long prevT = micros();
  float dErr = 0;        // อัตราการเปลี่ยนของ error (องศา/วินาที) กรองแล้ว
  bool reached = false;  // เคยเข้าช่วง tolerance แล้วหรือยัง (ใช้ทำ hysteresis)

  while (millis() - t0 < TURN_TIMEOUT) {
    bool fresh = updateIMU();  // true เมื่อมีค่าใหม่จาก IMU
    err = wrap180(target - wrap180(rawHeading - headingOffset));

    // คำนวณ D เฉพาะตอนที่มีค่าใหม่ (กันค่าซ้ำทำให้ derivative เป็น 0 สลับสไปก์)
    if (fresh) {
      unsigned long now = micros();
      float dt = (now - prevT) / 1000000.0;
      if (dt > 0.002) {
        float raw = wrap180(err - prevErr) / dt;
        dErr = 0.7 * dErr + 0.3 * raw;  // low-pass filter ลด noise
        prevErr = err;
        prevT = now;
      }
    }

    // Hysteresis: ตอนแรกต้องเข้าช่วง TOLERANCE ก่อน แต่พอเข้าแล้ว
    // ถ้าเฉื่อยไหลออกมานิดหน่อย (ไม่เกิน 2 เท่า) ก็ไม่สั่งแก้ใหม่ ปล่อยให้นิ่ง
    float band = reached ? TURN_TOLERANCE * 2.0 : TURN_TOLERANCE;
    if (fabs(err) <= band) {
      reached = true;
      AO();
      if (inTolSince == 0) inTolSince = millis();
      else if (millis() - inTolSince >= SETTLE_MS) break;
      delay(2);
      continue;
    }
    inTolSince = 0;

    // ไกลเป้า (เกิน SLOW_ZONE_DEG) -> หมุนเต็มที่ MAX_SPEED
    // เข้าใกล้เป้า (ภายใน SLOW_ZONE_DEG) -> ลดเหลือ MIN_SPEED ทันที
    // ทิศมอเตอร์ตาม err เสมอ ไม่กลับทิศ (กันสบัด)
    float dir = (err > 0) ? 1.0 : -1.0;
    float mag = (fabs(err) > SLOW_ZONE_DEG) ? MAX_SPEED : MIN_SPEED;

    spin((int)(dir * mag));

    if (DEBUG_TURN) {  // พิมพ์ทุก ~20 ms: เวลา(ms), error(องศา), ความเร็วที่สั่ง
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 20) {
        lastPrint = millis();
        Serial.print(millis() - t0);
        Serial.print(',');
        Serial.print(err);
        Serial.print(',');
        Serial.println((int)(dir * mag));
      }
    }
    delay(2);
  }
  AO();
}

void turnLeft90() {
  delay(65);
  turnToHeading(targetHeading + 90);
}
void turnRight90() {
  delay(65);
  turnToHeading(targetHeading - 90);
}

void turnLeft180() {
  delay(65);
  turnToHeading(targetHeading + 180);
}
void turnRight180() {
  delay(65);
  turnToHeading(targetHeading - 180);
}
void turnLeft45() {
  turnToHeading(targetHeading + 45);
}
void turnRight45() {
  turnToHeading(targetHeading - 45);
}

// ============================================================
// วิ่งตรงตามทิศ (heading hold)
// ============================================================
// speed บวก = เดินหน้า, ลบ = ถอยหลัง, ms = ระยะเวลา
// ล็อกทิศที่ targetHeading ปัจจุบัน (ทิศที่เลี้ยวล่าสุด) แล้วปรับล้อซ้าย/ขวาให้ตรง
void driveStraight(int speed, unsigned long ms, bool apColor = false) {
  float target = targetHeading;
  unsigned long t0 = millis();
  float err;
  int corr, steer = 0, corrline = 0;
  int dir = (speed >= 0) ? 1 : -1;
  bool leftLine, rightLine;
  unsigned long starttime;
  int maxsp = 100;
  float rate = 0.2;
  while (millis() - t0 < ms) {
    err = wrap180(target - getHeading());
    corr = (int)(KP_STRAIGHT * err);
    starttime = millis();
    if (!apColor) {
      leftLine = analog(0) < REF[0];
      rightLine = analog(1) < REF[1];
      if (corrline >= 5) corrline -= corrline * 0.08;
      else corrline = 0;
      if (leftLine && rightLine) {
        AO();
        setDrive(-20, -20);
        delay(500);
        break;
      } else if (leftLine) corrline -= 35;
      else if (rightLine) corrline += 35;
    }

    corr = constrain(corr + corrline, -MAX_CORR, MAX_CORR);

    setDrive(constrain(speed + dir * corr, -100, 100),
             constrain(speed - dir * corr, -100, 100));
    delay(2);
  }
  AO();
}

// หันไปทิศที่สั่ง (เทียบจุดศูนย์) แล้ววิ่งตรงไปทางนั้น
void driveAtHeading(float heading, int speed, unsigned long ms) {
  turnToHeading(heading);
  delay(100);
  driveStraight(speed, ms);
}

void driveBlocks(short blocks, bool apColor = false) {
  short dir = 1;
  if (blocks < 0) {blocks = -blocks; dir = -dir;}
  driveStraight(55*dir, 420);
  if (blocks > 1) driveStraight(60*dir, 280 * (blocks - 1), apColor);
}

void deliver(short dir){
  driveStraight(30*dir, 500,true);
}

void load(){
  oled.textSize(2);
  oled.text(1, 2, "LOAD!");
  oled.show();
  beep();
  waitAnykey();
  beep();
  delay(200);
  oled.clear();
  oled.text(1, 2, "GO!");
  oled.show();
}

// === HIGH-LEVEL FUNCTION ===

void actRun() {
  driveBlocks(3);
  turnRight90();
  driveBlocks(2);
  turnRight90();
  driveBlocks(1);
  turnRight90();
  driveBlocks(1);
  turnLeft90();
  driveBlocks(1);
  deliver(1);
  dropCube(1, 1);
  turnLeft180();
  dropCube(2, 1);
  deliver(1);
  turnRight90();
  driveBlocks(4);
  turnRight90();
  driveBlocks(1);
  turnRight90();
  driveBlocks(2);
  deliver(1);
  dropCube(1, 1);
  turnLeft180();
  dropCube(2, 1);
  deliver(1);
  driveBlocks(3);
  turnLeft90();
  driveBlocks(3);
  turnRight90();
  driveBlocks(1);
  deliver(1);
  dropCube(1, 1);
  turnLeft180();
  dropCube(2, 1);
  deliver(1);
  driveBlocks(1);
  turnLeft90();
  driveBlocks(2);
  turnLeft90();
  driveBlocks(2);
  turnRight90();
  deliver(1);
  dropCube(1, 1);
  turnRight180();
  dropCube(2, 1);
  deliver(1);
}

void testIMU() {
}

void testIR() {
  oled.clear();
  oled.drawFastHLine(5, 5, 40, WHITE);
  oled.text(7, 4, "[A] AUTO CAL");
  while (1) {
    oled.text(0, 0, "%d   ", analog(0));
    oled.text(0, 16, "%d   ", analog(1));
    oled.text(2, 0, "%d   ", analog(2));
    oled.text(2, 16, "%d   ", analog(3));
    oled.show();
    delay(111);
  }
}

void testServo() {
  for (int cr = 1; cr <= 2; cr++) {
    servo(cr, 100);
    servo(cr, 20);
    delay(170);
    servo(cr, 100);
  }
}

void actCalib() {
  Serial.begin(115200);
  oled.text(0, 6, "Init IMU");
  oled.text(1, 1, "Waiting for IMU...");
  oled.show();

  delay(1000);  // รอให้ BNO086 บูตเสร็จหลังจ่ายไฟ
  oled.textColor(BLACK, WHITE);
  oled.text(1, 1, "Waiting for IMU...");
  oled.textColor(WHITE, BLACK);
  oled.text(2, 1, "Scanning address...");
  oled.show();
  scanI2C();  // ดูว่าเห็นอุปกรณ์ที่แอดเดรสอะไรบ้าง

  if (!initIMU(5)) {
    Serial.println("BNO086 init FAILED");
    Serial.println("- ถอดไฟแล้วเสียบใหม่ทั้งบอร์ดและ IMU");
    Serial.println("- ต่อสาย RST แล้วตั้ง IMU_RST_PIN");
    Serial.println("- ลองลด I2C_CLOCK / เช็กไฟ 3.3V");
    while (true) delay(1000);  // หยุดไว้ ไม่ให้หุ่นวิ่งมั่ว
  }
  oled.textColor(BLACK, WHITE);
  oled.text(2, 1, "Scanning address...");
  oled.textColor(WHITE, BLACK);
  oled.show();
  setReports();
  delay(500);
  zeroHeading();
}

void actAbout() {
  oled.text(1, 8, "BY");
  oled.text(4, 5, "Press any...");
  oled.textSize(2);
  oled.text(3, 8, "godPP");
  oled.show();
  waitAnykey();
}

// === SETUP/UTILS ===

typedef struct {
  const char *name;
  void (*fn)(void);
} MenuItem;

const MenuItem items[] = {
  { "RUN", actRun },
  { "TIMU", testIMU },
  { "TIR", testIR },
  { "TSV", testServo },
  { "SET0", actCalib },
};

void innit() {
  servo(1, 100);
  servo(2, 100);
  unsigned short k = knob(4), lastK = 99;
  while (1) {
    k = knob(4);
    if (SW_OK()) {
      beep();
      oled.clear();
      oled.textSize(1);
      oled.text(0, 6, "Running");
      oled.textSize(2);
      oled.text(1, 3, items[k].name);
      oled.textSize(1);
      oled.show();
      items[k].fn();
      oled.clear();
      lastK = 99;
    }
    if (k != lastK) {
      oled.clear();
      oled.textSize(1);
      if (k > 0) oled.text(7, 0, items[k - 1].name);
      if (k < 4) oled.text(7, 17, items[k + 1].name);
      oled.textSize(3);
      oled.text(1, 1, items[k].name);
      oled.show();
      lastK = k;
    }
  }
}

void setup() {
  actCalib();
  innit();
}

void loop() {
  waitAnykey_bmp();

}
