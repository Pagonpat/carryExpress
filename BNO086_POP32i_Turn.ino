/*
  BNO086 + POP-32i : หมุนหุ่นยนต์ด้วยมุม Heading (เวอร์ชันแก้ปัญหา "not found")
  ------------------------------------------------------------------------
  ไลบรารี (Library Manager):
    - "SparkFun BNO08x Cortex Based IMU"
    - POP32 (มากับบอร์ด POP-32i)

  การต่อสาย (I2C):
    VIN -> 3.3V | GND -> GND | SDA -> SDA | SCL -> SCL
    (ถ้ามี) RST -> ขา GPIO ที่ว่าง  แล้วใส่เลขขาที่ IMU_RST_PIN ด้านล่าง
    (ถ้ามี) INT -> ขา GPIO ที่ว่าง  แล้วใส่เลขขาที่ IMU_INT_PIN ด้านล่าง

  ทิศทาง: มุมบวก = หมุนขวา (ตามเข็มนาฬิกา)
  มอเตอร์: 1 = ซ้ายหน้า, 2 = ขวาหน้า, 3 = ซ้ายหลัง, 4 = ขวาหลัง
*/

#include <POP32.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

const int REF[4] = { 770, 760, 860, 680 };

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
const float KD = 0.04;           // ตัวหน่วง: ยิ่งมาก ยิ่งเบรกเร็วตอนใกล้เป้า (เริ่ม 0.03-0.1)
const int MAX_SPEED = 70;
const int MIN_SPEED = 35;
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

void stopMotor() {
  setDrive(0, 0);
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
      stopMotor();
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
  stopMotor();
}

void turnLeft90() {
  delay(65);
  turnToHeading(targetHeading + 90);
}
void turnRight90() {
  delay(65);
  turnToHeading(targetHeading - 90);
}

void turnLeft180(){
  delay(65);
  turnToHeading(targetHeading + 180);
}
void turnRight180(){
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
  int corr;
  int dir = (speed >= 0) ? 1 : -1;
  bool leftLine,rightLine;

  while (millis() - t0 < ms) {
    err = wrap180(target - getHeading());
    corr = (int)(KP_STRAIGHT * err);
    if (!apColor) {
    leftLine = analog(0) < REF[0];
    rightLine = analog(1) < REF[1];
    
      if (leftLine && rightLine) {
        stopMotor();
        setDrive(-20, -20);
        delay(500);
        break;
      } else if (leftLine) corr += 5;
      else if (rightLine) corr -= 5;
    }

    corr = constrain(corr, -MAX_CORR, MAX_CORR);

    setDrive(constrain(speed + dir * corr, -100, 100),
             constrain(speed - dir * corr, -100, 100));
    delay(2);
  }
  stopMotor();
}

// หันไปทิศที่สั่ง (เทียบจุดศูนย์) แล้ววิ่งตรงไปทางนั้น
void driveAtHeading(float heading, int speed, unsigned long ms) {
  turnToHeading(heading);
  delay(100);
  driveStraight(speed, ms);
}

void driveBlocks(unsigned short blocks, bool apColor = false) {
  driveStraight(55, 430);
  if (blocks > 1) driveStraight(50, 370 * (blocks - 1), apColor);
}


// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);  // รอให้ BNO086 บูตเสร็จหลังจ่ายไฟ

  scanI2C();  // ดูว่าเห็นอุปกรณ์ที่แอดเดรสอะไรบ้าง

  if (!initIMU(5)) {
    Serial.println("BNO086 init FAILED");
    Serial.println("- ถอดไฟแล้วเสียบใหม่ทั้งบอร์ดและ IMU");
    Serial.println("- ต่อสาย RST แล้วตั้ง IMU_RST_PIN");
    Serial.println("- ลองลด I2C_CLOCK / เช็กไฟ 3.3V");
    while (true) delay(1000);  // หยุดไว้ ไม่ให้หุ่นวิ่งมั่ว
  }

  setReports();
  delay(500);
  zeroHeading();

  Serial.println("Press OK to start");
  waitAnykey_bmp();
}

void loop() {
  driveBlocks(1,true);
  waitSW_OK();  // กด OK เพื่อทำรอบใหม่
}