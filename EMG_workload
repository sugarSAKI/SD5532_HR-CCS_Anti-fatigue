#include "EMGFilters.h"

const int EMG_PIN = A0;  // 定义EMG信号输入引脚
const int SAMPLE_SIZE = 100;  // 定义采样窗口大小
int emg_buffer[SAMPLE_SIZE];  // 定义EMG信号缓冲区
int emg_index = 0;  // 定义EMG信号缓冲区索引
float emg_rms = 0.0;  // 定义EMG信号的RMS值
float emg_mav = 0.0;  // 定义EMG信号的MAV值
float workload = 0.0;

#define TIMING_DEBUG 1
#define SensorInputPin A3 // input pin number

int Down = A1;
int Up = A2;

static float wl_threshold = 2;
static int Threshold = 36;

#define TIMING_DEBUG 0;
EMGFilters myFilter;
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
unsigned long long interval = 1000000ul / sampleRate;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

void setup() {
  myFilter.init(sampleRate, humFreq, true, true, true);
  
  Serial.begin(115200);  // 初始化串口通信
  
  pinMode(Down, OUTPUT);
  pinMode(Up, OUTPUT);
}

void loop() {
  unsigned long long timeStamp = micros();
  
  int emg_value = analogRead(EMG_PIN);  // 读取EMG信号
  int Value = analogRead(SensorInputPin);
  

  //filter processing
  emg_value = myFilter.update(emg_value);
  int DataAfterFilter = myFilter.update(Value);
  int envlope = sq(DataAfterFilter);
  envlope = (envlope > Threshold) ? envlope : 0;
  
  emg_buffer[emg_index] = emg_value;  // 将EMG信号添加到缓冲区
  emg_index = (emg_index + 1) % SAMPLE_SIZE;  // 更新缓冲区索引

  // 计算EMG信号的RMS值
  float emg_sum = 0.0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    emg_sum += emg_buffer[i] * emg_buffer[i];
  }
  emg_rms = sqrt(emg_sum / SAMPLE_SIZE);

  // 计算EMG信号的MAV值
  float emg_abs_sum = 0.0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    emg_abs_sum += abs(emg_buffer[i]);
  }
  emg_mav = emg_abs_sum / SAMPLE_SIZE;

  workload = emg_rms * emg_mav;
  
  if(workload > wl_threshold){
    analogWrite(Up, 255);
  }
  else{
    analogWrite(Up, 0);
  }
  if(envlope > 0){
    analogWrite(Down, 255);
  }
  else{
    analogWrite(Down, 0);
  }
  
  Serial.print(workload);
  Serial.print(" ,");
  Serial.println(envlope);
  
  delay(10);  // 稍作延时，避免过快采样
}
