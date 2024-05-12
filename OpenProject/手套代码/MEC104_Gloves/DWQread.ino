void read_DWQ() {
    static u32 knob_value;
    static int16_t ax_t[5],ay_t[5],az_t[5];
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

void judgestate(){
  if(pos[1]>600 && pos[2]>600 && pos[3]<600){
      
      state=0;
         
      }
    else if(pos[1]<600 && pos[2]<600 && pos[3]>600){
      //握拳模式
//      Serial.println("握拳模式");
      state=1;
      dwq_output[0] = 1;
      dwq_output[1] = 1;
      dwq_output[2] = 1;
      dwq_output[3] = 1;
      dwq_output[4] = 1;
      }
    else if(pos[1]>600 && pos[2]>600 && pos[3]>600){
      //抬起拇指
//      Serial.println("抬起食指和中指");
      state=2;
      dwq_output[0] = 2;
      dwq_output[1] = 1;
      dwq_output[2] = 1;
      dwq_output[3] = 1;
      dwq_output[4] = 1;
      }
    else if(pos[1]>600 && pos[2]<600 && pos[3]>600){
      //抬起食指
//      Serial.println("抬起食指");
      state=3;
      dwq_output[0] = 1;
      dwq_output[1] = 2;
      dwq_output[2] = 1;
      dwq_output[3] = 1;
      dwq_output[4] = 1;
      }
    else if(pos[1]<600 && pos[2]>600 && pos[3]>600){
      //抬起中指
//      Serial.println("抬起中指");
      state=4;
      dwq_output[0] = 1;
      dwq_output[1] = 1;
      dwq_output[2] = 2;
      dwq_output[3] = 1;
      dwq_output[4] = 1;
      }
     else{
//      Serial.println("其他情况");
      state=5;
}
}
