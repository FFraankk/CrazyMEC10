#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

MPU6050 mpu6050(Wire);

long timer = 0;

float acc[3]={0,0,0};
float angle[3]={0,0,0};
float acc_estimated[3]={0,0,0};
float angle_estimated[3]={0,0,0};
SimpleKalmanFilter KalmanFilter_acc(1.9, 1.5, 6);
SimpleKalmanFilter KalmanFilter_angle(1, 1.5, 1);
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop(){
    mpu6050.update();
 
acc[0]=mpu6050.getAccX();
acc_estimated[0]=KalmanFilter_acc.updateEstimate(mpu6050.getAccX());
angle[0]=mpu6050.getAngleX();
angle_estimated[0]=KalmanFilter_angle.updateEstimate(mpu6050.getAngleX());
//Serial.print(acc[0]);
//Serial.print(",");
//Serial.println(acc_estimated[0]);
Serial.print(angle[0]);
Serial.print(",");
Serial.println(angle_estimated[0]);
  }
