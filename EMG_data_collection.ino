int EMGPin = A0;
int status_label = 0; //当前实验状态
int Max_emg = 320;
int Min_emg = 280;
String emg_value;

int get_EMG_data(){
  long sum = 0;
  for(int i=0; i<32; i++){
    int value = analogRead(EMGPin);
    
    if(value > 320){
      sum += 320;
    }
    else if(value < 280){
      sum += 280;
    }
    else{
      sum += analogRead(EMGPin);
    }
    
    //sum += value;
  }
  int emg_value = sum >> 5;
  return emg_value;
}

void setup() {
  Serial.begin(115200);
  //analogReference(EXTERNAL); //AREF
}

void loop() {
  int emg_data;
 
  emg_data = get_EMG_data();
  Serial.print(emg_data);
  Serial.print(",");
  Serial.print(status_label);
  //Serial.print(", 280, 320");
  Serial.println();
  //delay(100);

}
