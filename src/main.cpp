#include <Arduino.h>

/*=============INCLUDE LIBRARIES==============*/
#include <MPU6050_tockn.h>
#include <Wire.h>

/*=============DEFINE PIN====================*/
#define BUZZER_PIN 18

/*=============INITIALIZE OBJECT=============*/
MPU6050 mpu(Wire);

/*==CONVERSION CONSTANT FROM m/s² TO G (GRAVITY)==*/
const float G_TO_MS2 = 9.80665;

/*=================SETUP====================*/
void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Wait for Serial to be ready
  while (!Serial);

  // Initialize I2C
  Wire.begin();

  // Initialize Sensor
  mpu.begin();
  
  // Cấu hình lại dải đo và bộ lọc
  // Thanh ghi ACCEL_CONFIG (0x1C): set AFS_SEL=2 (0x10) cho ±8G
  mpu.writeMPU6050(0x1C, 0x10);
  // Thanh ghi CONFIG (0x1A): set DLPF_CFG=4 (0x04) cho 21Hz
  mpu.writeMPU6050(0x1A, 0x04);
  // Thanh ghi GYRO_CONFIG đã được set ±500°/s (0x08) trong begin(), giữ nguyên
  // Thanh ghi SMPLRT_DIV (0x19) mặc định 0x00 => sample rate 1kHz
  Serial.println("MPU6050 initialized with ±8G and 21Hz filter");

  // cablirate sensor
  mpu.calcGyroOffsets(true);

  // Initialize Buzzer Pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

/*=================LOOP====================*/
void loop() {
  // update new data from sensor
  mpu.update();
  Serial.println();

  // read accel value (unit : G)
  float accX_g = mpu.getAccX();
  float accY_g = mpu.getAccY();
  float accZ_g = mpu.getAccZ();

  Serial.print("Acceleration X: ");
  Serial.print(accX_g);
  Serial.print(" G, Y: ");
  Serial.print(accY_g);
  Serial.print(" G, Z: ");
  Serial.print(accZ_g);
  Serial.println(" G");

  float accTotal = sqrt(accX_g * accX_g + accY_g * accY_g + accZ_g * accZ_g);
  Serial.print("Acceleration Total: ");
  Serial.println(accTotal);

  delay(100);
}