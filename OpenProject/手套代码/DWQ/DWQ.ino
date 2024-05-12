//函数调用区
#include <EEPROM.h>           //调用<EEPROM.h>库
//#include <SoftwareSerial.h>

//定义变量区
//SoftwareSerial bluetoothSerial(0, 1); // RX, TX
#define DWQ_NUM     5               //宏定义电位器数量
#define ADC_MIN_ADDR   0     //握紧时采集电位器值起始地址
#define ADC_MAX_ADDR   30    //张手时采集电位器值起始地址
#define JXB_PWM_RANGE  1000.0 //量程
#define Straightening   2    //手指伸直时为2
#define bend            1    //手指弯曲时为1

byte dwq_pin[DWQ_NUM] = {A6, A7, A0, A2, A3};               //定义电位器引脚数组
int  ADC_MAX[DWQ_NUM] = {0, 0, 0, 0, 0};  //默认握紧时采集电位器值
int  ADC_MIN[DWQ_NUM] = {0, 0, 0, 0, 0};  //默认张手时采集电位器值
int  ADC_MID[DWQ_NUM] = {0, 0, 0, 0, 0};            //默认电位器中值
int  pos[DWQ_NUM] = {0}, pos_x=0, pos_y=0;          //变量pos用来存储转化后的电位器数据
int dwq_output[DWQ_NUM] = {0,0,0,0,0};              //电位器输出数据
float pos_p[DWQ_NUM] = {1,1,1,1,1};                 //放大倍数

//定义函数区
/*
    读写eeprom数据
    addr 地址
    len 数据长度
    mydat 数据buf
*/
/*void eepromRead(u32 addr, u8 len, u8 *mydat){
    for(u8 i = 0; i < len; i++) {//求取旋钮平均值，存储到EEPROM
        mydat[i] = EEPROM.read(addr+i);
    }        
}*/
/*
    读写eeprom数据
    addr 地址
    len 数据长度
    mydat 数据buf
*/
/*void eepromWrite(u32 addr, u8 len, u8 *mydat){
    for(u8 i = 0; i < len; i++) {//求取旋钮平均值，存储到EEPROM
        EEPROM.write(addr+i, mydat[i]);
    }        
}//死亡不掉落
*/
void read_DWQ() {
    static u32 knob_value;
    //static int16_t ax_t[5],ay_t[5],az_t[5];
    for(int i=0;i<5;i++) {
        knob_value = 0;
        for(int j=0;j<10;j++) {
            knob_value+=analogRead(dwq_pin[i]);
            delayMicroseconds(100);
         }
         knob_value = knob_value/10.0;
         pos[i] = knob_value;
    } //读取电位计的模拟值，取连续10个值，最后求平均
}


void setup() {
    //eepromRead(ADC_MIN_ADDR, sizeof(ADC_MIN), (u8 *)ADC_MIN);//读取存储电位器最小值
    //eepromRead(ADC_MAX_ADDR, sizeof(ADC_MAX), (u8 *)ADC_MAX);//读取存储电位器最大值 
    for(int i=0;i<DWQ_NUM;i++) {
      ADC_MID[i] = (ADC_MIN[i] + ADC_MAX[i])/2;
      pos_p[i] = JXB_PWM_RANGE/abs(ADC_MIN[i] - ADC_MAX[i]);//放大倍数 
    } 
    Serial.begin(9600);
}

void loop() {
  read_DWQ();
  if(pos[1]>600 && pos[2]>600 && pos[3]<600){
    //进入张手模式，yyk负责
    Serial.println("ax=,ay=,az=");
    
    }
  else if(pos[1]<600 && pos[2]<600 && pos[3]>600){
    //握拳模式
    Serial.println("握拳模式");
    dwq_output[0] = 1;
    dwq_output[1] = 1;
    dwq_output[2] = 1;
    dwq_output[3] = 1;
    dwq_output[4] = 1;
    }
  else if(pos[1]>600 && pos[2]>600 && pos[3]>600){
    //抬起拇指
    Serial.println("抬起食指和中指");
    dwq_output[0] = 2;
    dwq_output[1] = 1;
    dwq_output[2] = 1;
    dwq_output[3] = 1;
    dwq_output[4] = 1;
    }
  else if(pos[1]>600 && pos[2]<600 && pos[3]>600){
    //抬起食指
    Serial.println("抬起食指");
    dwq_output[0] = 1;
    dwq_output[1] = 2;
    dwq_output[2] = 1;
    dwq_output[3] = 1;
    dwq_output[4] = 1;
    }
  else if(pos[1]<600 && pos[2]>600 && pos[3]>600){
    //抬起中指
    Serial.println("抬起中指");
    dwq_output[0] = 1;
    dwq_output[1] = 1;
    dwq_output[2] = 2;
    dwq_output[3] = 1;
    dwq_output[4] = 1;
    }
   else{
    Serial.println("其他情况");
    //Serial.println(pos[0]);Serial.println(pos[1]);Serial.println(pos[2]);Serial.println(pos[3]);Serial.println(pos[4]);
    }
}
