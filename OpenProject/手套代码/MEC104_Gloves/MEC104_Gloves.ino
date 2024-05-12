//库文件导入
#include <SoftwareSerial.h>

#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

//定义
#define DWQ_NUM     5               //宏定义电位器数量
#define ADC_MIN_ADDR   0     //握紧时采集电位器值起始地址
#define ADC_MAX_ADDR   30    //张手时采集电位器值起始地址
#define JXB_PWM_RANGE  1000.0 //量程

#define PIN_beep    5               //宏定义蜂鸣器引脚
#define PIN_nled    13              //宏定义工作指示灯引脚

#define beep_on() digitalWrite(PIN_beep, HIGH);
#define beep_off() digitalWrite(PIN_beep, LOW);

#define nled_on() digitalWrite(PIN_nled, LOW);
#define nled_off() digitalWrite(PIN_nled, HIGH);

byte dwq_pin[DWQ_NUM] = {A6, A7, A0, A2, A3};               //定义电位器引脚数组
int  ADC_MAX[DWQ_NUM] = {0, 0, 0, 0, 0};  //默认握紧时采集电位器值
int  ADC_MIN[DWQ_NUM] = {0, 0, 0, 0, 0};  //默认张手时采集电位器值
int  ADC_MID[DWQ_NUM] = {0, 0, 0, 0, 0};            //默认电位器中值
int  pos[DWQ_NUM] = {0}, pos_x=0, pos_y=0;          //变量pos用来存储转化后的电位器数据
int dwq_output[DWQ_NUM] = {0,0,0,0,0};              //电位器输出数据
float pos_p[DWQ_NUM] = {1,1,1,1,1};                 //放大倍数
float acc[3]={0,0,0};
float angle[3]={0,0,0};
float acc_estimated[3]={0,0,0};
float angle_estimated[3]={0,0,0};
static float data_to_be_sent[7]={0,0,0,0,0,0,0};
static float state=0.0;

MPU6050 mpu6050(Wire);
SimpleKalmanFilter KalmanFilter_acc(1.9, 1.5, 6);
SimpleKalmanFilter KalmanFilter_angle(1, 1.5, 1);


void setup() {
  Serial.begin(9600);  
//  Serial.println("bluetooth is ready!");
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("循环开始");
   nled_on();beep_on();delay(100);nled_off();beep_off();delay(100);
   nled_on();beep_on();delay(100);nled_off();beep_off();delay(100);
   nled_on();beep_on();delay(100);nled_off();beep_off();delay(100);

}
//主循环
void loop() {
  read_DWQ();
  mpu6050_read(); 
  judgestate();
  send();
}
