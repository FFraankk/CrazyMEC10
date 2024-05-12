/*
    硬件IO分布：
    1、x轴 y轴 I2C芯片
    2、电位器 A6 A7 A0 A2 A3
    3、舵机引脚 11 10 9 8 7 6
    4、led引脚13 蜂鸣器引脚 5 校准按键 3 模式按键 4

    闭合手 控制小车
    张开手 控制云台
    其他控制手指
*/

#include <Servo.h>            // 调用Servo.h库
#include <Wire.h>
#include <I2Cdev.h>
#include <ADXL345.h>
#include <EEPROM.h>           //调用<EEPROM.h库


#define SERVO_NUM   6               //宏定义电位器数量
#define DWQ_NUM     5               //宏定义电位器数量
#define PIN_beep    5               //宏定义蜂鸣器引脚
#define PIN_nled    13              //宏定义工作指示灯引脚
#define PIN_KEY_J   3               //宏定义校准引脚
#define PIN_KEY_M   4               //宏定义模式引脚

#define ADC_MIN_ADDR   0     //握紧时采集电位器值起始地址
#define ADC_MAX_ADDR   30    //张手时采集电位器值起始地址
#define INT_DATA_TYPE  2     //数据类型,数据类型=几个字节就是几，相当于数据拆分成字节
#define JXB_PWM_RANGE  1000.0 //量程
#define JXB_PWM_STRAT  (500+(2000-JXB_PWM_RANGE)/2)     //机械臂PWM起始值
#define X_MIDDLE 0
#define Y_MIDDLE 0

#define beep_on() digitalWrite(PIN_beep, HIGH);
#define beep_off() digitalWrite(PIN_beep, LOW);

#define nled_on() digitalWrite(PIN_nled, LOW);
#define nled_off() digitalWrite(PIN_nled, HIGH);

char cmd_return[100], cmd_return_bak[100];
Servo myservo[SERVO_NUM];                          //创建舵机类数组
byte servo_pin[SERVO_NUM] = {11, 10, 9, 8, 7, 6};       //定义舵机控制引脚数组
byte dwq_pin[DWQ_NUM] = {A6, A7, A0, A2, A3};               //定义电位器引脚数组
int  ADC_MAX[DWQ_NUM] = {0, 0, 0, 0, 0};  //默认握紧时采集电位器值
int  ADC_MIN[DWQ_NUM] = {0, 0, 0, 0, 0};  //默认张手时采集电位器值
int  ADC_MID[DWQ_NUM] = {0, 0, 0, 0, 0};            //默认张手时采集电位器值
int  pos[DWQ_NUM] = {0}, pos_x=0, pos_y=0;          //变量pos用来存储转化后的电位器数据
int  pos_bak[5] = {0}, pos_x_bak=0, pos_y_bak=0;             //变量pos备份值
int  pwm_value[DWQ_NUM] = {0};                      //变量pwm值
float pos_p[DWQ_NUM] = {1,1,1,1,1};                         //放大倍数
u8 robot_mode = 0;

ADXL345 myAdxl345;


void setup(){
    Serial.begin(115200);                           //初始化波特率为115200
    pinMode(PIN_nled, OUTPUT);                      //设置LED引脚为输出模式
    pinMode(PIN_beep, OUTPUT);                      //设置蜂鸣器引脚为输出模式
    pinMode(PIN_KEY_M, INPUT_PULLUP);               //将模式按键对应引脚设置为内部上拉输入模式，防止误判
    pinMode(PIN_KEY_J, INPUT_PULLUP);               //将校准按键对应引脚设置为内部上拉输入模式，防止误判


    Wire.begin();
    myAdxl345.initialize();
    myAdxl345.testConnection();

    eepromRead(ADC_MIN_ADDR, sizeof(ADC_MIN), (u8 *)ADC_MIN);//读取存储电位器最小值
    eepromRead(ADC_MAX_ADDR, sizeof(ADC_MAX), (u8 *)ADC_MAX);//读取存储电位器最大值
    for(int i=0;i<DWQ_NUM;i++) {
        ADC_MID[i] = (ADC_MIN[i] + ADC_MAX[i])/2;
        pos_p[i] = JXB_PWM_RANGE/abs(ADC_MIN[i] - ADC_MAX[i]);//放大倍数
    }

    for(byte i = 0; i < SERVO_NUM; i++){
         myservo[i].attach(servo_pin[i]);   // 将5引脚与声明的舵机对象连接起来
         myservo[i].writeMicroseconds(1500);//初始化舵机为1500状态       
    } 

    //启动示意
    nled_on();beep_on();delay(100);nled_off();beep_off();delay(100);
    nled_on();beep_on();delay(100);nled_off();beep_off();delay(100);
    nled_on();beep_on();delay(100);nled_off();beep_off();delay(100);

#if 0
      sprintf(cmd_return,"ADC_MIN:%04d %04d %04d %04d %04d",
                          ADC_MIN[0], ADC_MIN[1], ADC_MIN[2], ADC_MIN[3], ADC_MIN[4]);
      Serial.println(cmd_return);  

      sprintf(cmd_return,"ADC_MAX:%04d %04d %04d %04d %04d",
                          ADC_MAX[0], ADC_MAX[1], ADC_MAX[2], ADC_MAX[3], ADC_MAX[4]);
      Serial.println(cmd_return);   
#endif
}

void loop(){ 
    loop_nled();        //led灯闪烁函数
    loop_key_j();       //校验函数
//    loop_Yhand_R();        //旋钮控制
}


int Filter(int FILTER_N, int pin) {
  int filter_buf[FILTER_N];
  int i, j;
  int filter_temp;
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = analogRead(pin);
    delayMicroseconds(300);
  }
  // 采样值从小到大排列（冒泡法）
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf[i] > filter_buf[i + 1]) {
        filter_temp = filter_buf[i];
        filter_buf[i] = filter_buf[i + 1];
        filter_buf[i + 1] = filter_temp;
      }
    }
  }
  return filter_buf[(FILTER_N - 1) / 2];
}


void loop_nled() {
    static u32 systick_ms_bak = 0;
    static u8 flag=0;
    if(millis() - systick_ms_bak<500)return;
    systick_ms_bak = millis();
    flag = !flag;
    if(flag) {
        nled_on();
    } else {
        nled_off();
    }
}
