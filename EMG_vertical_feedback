#include "EMGFilters.h"
#include "HX711.h"

HX711 HX711_CH0(A3, A4, 400);
//SCK引脚用于arduino和HX711模块通讯的时序提供
//DT引脚用于从HX711读取AD的数据
//GapValue用于校准输出的重量值，如果数值偏大就加大该值，如果数据偏小就减小该值
long Weight = 0;    //定义一个变量用于存放承重的重量，单位为g

const int EMG_PIN = A0;  // 定义EMG信号输入引脚
const int SAMPLE_SIZE = 20;  // 定义采样窗口大小
float emg_buffer[SAMPLE_SIZE];  // 定义EMG信号缓冲区
int emg_index = 0;  // 定义EMG信号缓冲区索引
float emg_rms = 0.0;  // 定义EMG信号的RMS值
float emg_mav = 0.0;  // 定义EMG信号的MAV值
float emg_amplitude = 0.0; //定义EMG信号的振幅
float emg_workload = 0.0;

#define TIMING_DEBUG 1

static float wl_down_threshold = 0.50;
static float wl_up_threshold = 6.00;
static float ts_threshold = 200;

#define TIMING_DEBUG 0;
EMGFilters myFilter;
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
unsigned long long interval = 1000000ul / sampleRate;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

void setup() {
  Serial.begin(115200);  // 初始化串口通信
  
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

  // 计算EMG信号的振幅
  emg_amplitude = get_emg_amplitude();

  // 用RMS和MAV指代肌肉的workload
  emg_workload = emg_rms * emg_mav;

  // 计算拉力传感器承重
  Weight = HX711_CH0.Get_Weight();
  
  if(wl_down_threshold != 0 && wl_up_threshold !=0  && ts_threshold != 0){
    if(emg_workload > wl_down_threshold && Weight > ts_threshold){
      Serial.print("down, workload: ");
      Serial.print(emg_workload);
      Serial.print(", tension: ");
      Serial.print(Weight);
      Serial.println(" g");
    }
    else if(emg_workload > wl_up_threshold && Weight <= ts_threshold){
      do{
        Serial.print("up, workload: ");
        Serial.print(emg_workload);
        Serial.print(", tension: ");
        Serial.print(Weight);
        Serial.println(" g");
        Weight = HX711_CH0.Get_Weight();
        delay(20);
      }while(Weight <= 0);
    }
    else{
        Serial.print("stable, workload: ");
        Serial.print(emg_workload);
        Serial.print(", tension: ");
        Serial.print(Weight);
        Serial.println(" g");
    }
  }
  else{
      Serial.print("Debugging, workload: ");
      Serial.print(emg_workload);
      Serial.print(", tension: ");
      Serial.print(Weight);
      Serial.println(" g");
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

float get_emg_amplitude(){
  float emg_abs_sum = 0.0;
  for (int i=0; i<SAMPLE_SIZE; i++){
    emg_abs_sum += abs(emg_buffer[i]);
  }
  float amplitude = emg_abs_sum / SAMPLE_SIZE;
  return amplitude;
}
