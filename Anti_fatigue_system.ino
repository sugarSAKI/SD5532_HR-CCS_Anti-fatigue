//FINAL VERSION 1.0
#include "EMGFilters.h"
#include "HX711.h"
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include <Wire.h>

HX711 HX711_CH0(A3, A4, 400);
//SCK引脚用于arduino和HX711模块通讯的时序提供
//DT引脚用于从HX711读取AD的数据
int x0 = 160;
int y0 = 120;

HUSKYLENS huskylens;
SoftwareSerial mySerial(6, 5); // RX, TX
//HUSKYLENS green line >> Pin 5; blue line >> Pin 6
void printResult(HUSKYLENSResult result);

int Weight = 0;

const int EMG_PIN = A0;  // 定义EMG信号输入引脚
const int SAMPLE_SIZE = 20;  // 定义采样窗口大小
float emg_buffer[SAMPLE_SIZE];  // 定义EMG信号缓冲区
int emg_index = 0;  // 定义EMG信号缓冲区索引
float emg_rms = 0.0;
float emg_mav = 0.0;
float emg_workload = 0.0;

static float wl_down_threshold = 0.50;
static float wl_up_threshold = 3.00;
static float ts_threshold = 200;

EMGFilters myFilter;
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
unsigned long long interval = 1000000ul / sampleRate;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

char state_v = '0';


void setup() {
  Wire.begin();
  Serial.begin(115200);  // 初始化串口通信
  mySerial.begin(9600);
  
  //camera初始化
  while (!huskylens.begin(mySerial))
  {
    Serial.println(huskylens.begin(Wire));
    Serial.println("Huskylens initialization.");
  }
  
  // emg数据过滤初始化函数
  myFilter.init(sampleRate, humFreq, true, true, true);
  // tension 传感器初始化
  HX711_CH0.begin(); //读取传感器支架毛重
  delay(3000);
  HX711_CH0.begin(); //重新读取传感器支架毛重用于后续计算
  // emg传感器初始化
  do{
    update_emg();
    emg_rms = get_emg_rms();
    emg_mav = get_emg_mav();
    emg_workload = emg_rms * emg_mav;
    Serial.print("workload initialization: ");
    Serial.println(emg_workload);
    delay(20);
  }while(emg_workload >= 3);
  
  Serial.println("workload initialization finished.");

  
}

void loop() {
  unsigned long long timeStamp = micros();
  
  update_emg();

  // 计算EMG信号的RMS值
  emg_rms = get_emg_rms();
  
  // 计算EMG信号的MAV值
  emg_mav = get_emg_mav();
  
  // 用RMS和MAV指代肌肉的workload
  emg_workload = emg_rms * emg_mav;

  // 计算拉力传感器承重
  Weight = HX711_CH0.Get_Weight();
  
  if(wl_down_threshold != 0 && wl_up_threshold !=0  && ts_threshold != 0){
    if(emg_workload > wl_down_threshold && Weight > ts_threshold){
      state_v = '1';
      Serial.print("down, workload: ");
      Serial.print(emg_workload);
      Serial.print(", tension: ");
      Serial.print(Weight);
      Serial.print(",");
    }
    else if(emg_workload > wl_up_threshold && Weight <= ts_threshold){
      state_v = '2';
      Serial.print("up, workload: ");
      Serial.print(emg_workload);
      Serial.print(", tension: ");
      Serial.print(Weight);
      Serial.print(",");
    }
    else{
      state_v = '0';
      Serial.print("stable, workload: ");
      Serial.print(emg_workload);
      Serial.print(", tension: ");
      Serial.print(Weight);
      Serial.print(",");
    }
  }
  else{
      Serial.print("Debugging, workload: ");
      Serial.print(emg_workload);
      Serial.print(", tension: ");
      Serial.print(Weight);
      Serial.println(",");
  }

  int wl = map(emg_workload, 0, 40, 0, 240);
  int it = map(Weight, -300, 1000, 0, 240);
  Serial.print(wl);
  Serial.print(",");
  Serial.print(it);
  Serial.print(",");

  //水平方向运动
  //Serial.println(huskylens.available());
  if (!huskylens.request()){
    Serial.print(false);
    Serial.println(", 0, 0, 0, 0, 0");
    //Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  }
  else if(!huskylens.isLearned()){
    Serial.print(false);
    Serial.println(", 0, 0, 0, 0, 0");
    //Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  }
  else if(!huskylens.available()){
    Serial.print(false);
    Serial.println(", 0, 0, 0, 0, 0");
    //Serial.println(F("No block or arrow appears on the screen!"));
  }
  else
  {
      //Serial.println(F("###########"));
      while (huskylens.available())
      {
          HUSKYLENSResult result = huskylens.read();
          printResult(result);
      }    
  }
  
  delay(20);  // 稍作延时，避免过快采样
}

void update_emg(){
  int emg_value = analogRead(EMG_PIN);  // 读取EMG信号 

  //filter processing
  float emg_filtered = myFilter.update(emg_value);

  if(abs(emg_filtered) <=100){
    emg_buffer[emg_index] = emg_filtered;  // 将EMG信号添加到缓冲区
    emg_index = (emg_index + 1) % SAMPLE_SIZE;  // 更新缓冲区索引
  }
}

float get_emg_rms(){
  float emg_sum = 0.0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    emg_sum += emg_buffer[i] * emg_buffer[i];
  }
  float rms = sqrt(emg_sum / SAMPLE_SIZE);
  return rms;
}

float get_emg_mav(){
  float emg_abs_sum = 0.0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    emg_abs_sum += abs(emg_buffer[i]);
  }
  float mav = emg_abs_sum / SAMPLE_SIZE;
  return mav;
}


void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
      float opposite = result.yCenter - y0; // 直角三角形中与该角相对的直角边长度
      float adjacent = result.xCenter - x0; // 直角三角形中与该角相邻的直角边长度
      int oppositeZF = opposite / abs(opposite);
      int adjacentZF = adjacent / abs(adjacent);
      // 算一个偏移程度
      int h_distance = sq(opposite) + sq(adjacent);
      float angle = atan(opposite / adjacent) * (180 / PI); // 计算角度值
      float absangle = fabs(angle);
      //Serial.println(angle); // 输出角度值
        //Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
      bool h_flag = false;
        if (abs(opposite)<20 & abs(adjacent)<20){
          h_flag = false;
        }
        else{
          h_flag = true;
        }
      Serial.print(h_flag);
      Serial.print(",");
      Serial.print(result.ID);
      Serial.print(",");
      Serial.print(oppositeZF);
      Serial.print(",");
      Serial.print(adjacentZF);
      Serial.print(",");
      Serial.print(absangle);
      Serial.print(",");
      Serial.println(h_distance);
    }
    else{
      Serial.print(false);
      Serial.println(", 0, 0, 0, 0, 0");
    }
    delay(100);
}
