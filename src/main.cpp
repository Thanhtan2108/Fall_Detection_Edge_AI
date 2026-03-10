#include <Arduino.h>

/*=============INCLUDE LIBRARIES==============*/
#include <MPU6050_tockn.h>
#include <Wire.h>

/*=============INITIALIZE OBJECT=============*/
MPU6050 mpu(Wire);

/*==CONVERSION CONSTANT FROM m/s² TO G (GRAVITY)==*/
const float G_TO_MS2 = 9.80665;
