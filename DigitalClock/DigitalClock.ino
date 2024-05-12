#define reset 3
#define pause 2
int inputs_ones[4] = {9,10,11,12}; 
int inputs_tens[4] = {5,6,7,8};
byte BCD[10][4] ={
{0,0,0,0},
{1,0,0,0},
{0,1,0,0},
{1,1,0,0},
{0,0,1,0},
{1,0,1,0},
{0,1,1,0},
{1,1,1,0},
{0,0,0,1},
{1,0,0,1},
// {0,1,0,1},
// {1,1,0,1},
// {0,0,1,1},
// {1,0,1,1},
// {0,1,1,1},
// {1,1,1,1}
}; //BCD code

void setup() 
{d:\学习\XJTLU-Y2\MEC104\Arduino\open project\CrazyMEC10\OpenProject\手套代码\KalmanFilter\KalmanFilter.ino
  // put your setup code here, to run once:
  for(int a = 0; a < 4; a++){
  pinMode(inputs_ones[a],OUTPUT);  
  pinMode(inputs_tens[a],OUTPUT);
  } //set outputs

  pinMode(reset,INPUT); //reset key
  pinMode(pause,INPUT);//Pause key
  Serial.begin(9600);
}

void loop() 
{
  // static int reset_key_state=1;
  static int number=0;
  //开关默认状态是HIGH
  while (digitalRead(pause) == LOW)
  {
    Serial.println("Waiting...,静默等待");
    delay(10);
  } 

 
  if (digitalRead(reset)==0){
    number=0;
  }

  if(number>=56)
  {
      
       number=0;
  }
// && reset_key_state==1&& (!digitalRead(reset))
  if(number>=0)
  {
    int ones=number%10;
    int tens=number/10;
    for(int c = 0; c < 4; c++){
      digitalWrite(inputs_ones[c], BCD[ones][c]);
    }
    for(int c = 0; c < 4; c++){
      digitalWrite(inputs_tens[c], BCD[tens][c]);
    }
    number++;
    Serial.print("输出十位数是");
    Serial.print(tens);
    Serial.print("输出个位数是");
    Serial.println(ones);
    delay(500);
  }
}
