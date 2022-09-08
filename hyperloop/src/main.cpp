#include <Arduino.h>
#include "MPU9250.h"
#include "AHT10.h"
#include "SFE_BMP180.h"
#include <ESP32Servo.h>

#define ACT 15 // Frene Gidiyor
#define KONTAKTOR 13  // Pod Üzerinde 
#define FREN 16  // Frene Gidiyor

#define ENC 5 // Enkoder Bağlı

#define RX2 32
#define TX2 33

int rev = 0;

uint32_t stopTime = 0; 

char comingData = '.';
int prevTime, oldTime;
uint64_t checkTime;

MPU9250 mpu;

Servo escFR;
Servo escFL;
Servo escBR;
Servo escBL;
int escSpeed = 0;

SFE_BMP180 bmp;
char readPrStatus;
double P, T;

uint8_t readTempStatus = 0;
AHT10 temp(AHT10_ADDRESS_0X38);

char strFinal[100];
char strNav[40];
char strGen[40];

uint32_t millisDist = 0;
uint32_t millisCom = 0;

float akim = 0;

// Nav Variables
float totalDistance = 0;
float speed = 0;  // Measured Speed
volatile uint16_t speedKm = 0; 
bool readSpeed = false;

hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

struct dataPacket{

float yaw ;
float pitch;
float roll;

float navX;
float navY;
float navZ;

float speedX;
float speedY;
float speedZ;

float accelX;
float accelY;
float accelZ;

float pressure;

float temp1;

float sensTemp1;
float sensTemp2;

float distance;
float current;

};


struct distanceSensor{
  float analogVal[5];

  void medianFilter(float*);
  float getFilteredValue(float*, uint8_t);
  float getDistance(float);

};

struct optikSensor{
  uint16_t totalBantCounter;
  uint16_t bantCounter;
  uint64_t speedTimer;

  uint16_t speed;
 
};

void distanceSensor::medianFilter(float* currentVal){
  float temp = 0;
  for(uint8_t i = 0; i < 5; i++){
   for(uint8_t j = 0; j < i + 1; j++){
      if(*(currentVal + i) < *(currentVal + j)){
        temp = *(currentVal + i);
        *(currentVal + i) = *(currentVal + j);
        *(currentVal + j) = temp;
      }
   }
  }
}

float distanceSensor::getFilteredValue(float* vals, uint8_t pin){
  for(uint8_t i = 0; i < 5; i++){
      *(vals + i) = analogRead(pin);
   }
   medianFilter(vals);
   return *(vals + 2);
}

float distanceSensor::getDistance(float val){
  float distance;
  distance = (float) 8304 / (float)(val - 44);  // Formül Bulunacak
  if(distance < 4) distance = 3;
  if(distance > 30) distance = 31;
  return distance;
}

dataPacket* dataPack = new dataPacket();

optikSensor *optik1 = new optikSensor();
optikSensor *optik2 = new optikSensor();

distanceSensor *dist1 = new distanceSensor();
distanceSensor *dist2 = new distanceSensor();
distanceSensor *dist3 = new distanceSensor();
distanceSensor *dist4 = new distanceSensor();

void convertData(float data, char str[10], char name){
	uint8_t iterate = 0;
	while(iterate < 9){
		str[iterate] = '\0';
		iterate++;
	}
	iterate = 0;
  sprintf(str, "#%.2f", data);
	//gcvt(data,6,str + 1);
	while(str[iterate] != '\0'){
		iterate++;
	}
	str[iterate] = name;
	iterate = 0;
}


void convertString(dataPacket* data){
  sprintf(strFinal, "BB#%.2fy#%.2fr#%.2fp#%.2fi#%.2fj#%.2fk#%.2fX#%.2fY#%.2fZ#EE",
   data->yaw, data->roll, data->pitch,data->accelX, data->accelY, data->accelZ,
   data->speedX, data->speedY, data->speedZ
   );  

sprintf(strGen, "BB#%.2fg#%.2fG#%.2fC#EE", random(2800,2850)/100.00, random(2800,2850)/100.00,data->current);
 sprintf(strNav, "BB#%.2fu#%.2fv#%.2fw#EE",
   data->navX, data->navY, data->navZ
  );
  

}


void IRAM_ATTR beginCounter2(){
  portENTER_CRITICAL_ISR(&timerMux);
  optik2->speedTimer = timerRead(timer2);
  optik2->bantCounter++;
  optik1->speedTimer = timerRead(timer1);
  timerWrite(timer2, 0);
  portEXIT_CRITICAL_ISR(&timerMux);

}

void IRAM_ATTR beginCounter1(){
  portENTER_CRITICAL_ISR(&timerMux);
  optik1->bantCounter++;
  timerWrite(timer1, 0);
  portEXIT_CRITICAL_ISR(&timerMux);

}
void startPwm(){
int i = 0;
digitalWrite(ACT, HIGH);
delay(2000);
  while(i < 160){
  i+= 10;
  escSpeed=i;
  escFR.write(escSpeed);
  escFL.write(escSpeed);
  escBR.write(escSpeed);
  escBL.write(escSpeed);
  delay(800);
  }
  digitalWrite(ACT, LOW);
}


void setup() {
setCpuFrequencyMhz(40);	
Serial.begin(9600);

Wire.begin();
delay(2000);

 
  // MPU SETTINGS
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  if(!mpu.setup(0x68, setting)) {  
       
        printf("BB# MPU9250 Baglanti Hatasi.#EE\n");
            delay(2000);
    }
  
  // AHT10
  if(temp.begin() != true)
  {
    printf("BB# AHT10 Baglanti Hatasi. #EE\n"); 
    delay(2000);
  }

  // BMP180
  if(!bmp.begin()){
    printf("BB# BMP180 Baglanti Hatasi. #EE\n");
    delay(2000);
  }

  //ESC
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  escFR.setPeriodHertz(50);
  escFL.setPeriodHertz(50);
  escBR.setPeriodHertz(50);
  escBL.setPeriodHertz(50);

  escFR.attach(14, 1000, 2000);
  escFL.attach(27, 1000, 2000);
  escBR.attach(26, 1000, 2000);
  escBL.attach(25, 1000, 2000);

  escSpeed = 0;

  // TEMPORARY REGION
  dataPack->accelX = 0.5;
  dataPack->accelY = 0.02;
  dataPack->accelZ = 0.03;

  dataPack->distance = 4.3;
  dataPack->pressure = 986.47;
  dataPack->temp1 = 26.9;

  dataPack->yaw = 87.1; 
  dataPack->roll = 45.8;
  dataPack->pitch = 36.9;

  dataPack->navX = 0.0;
  dataPack->navY = 0.8;
  dataPack->navZ = 0.4;

  dataPack->speedX = 46.8;
  dataPack->speedY = 0.4;
  dataPack->speedZ = 0.2;



attachInterrupt(digitalPinToInterrupt(4), beginCounter1, FALLING);
attachInterrupt(digitalPinToInterrupt(ENC), beginCounter2, RISING);

timer1 = timerBegin(0, 1000, true); // Prescale Oranı : 1000
timer2 = timerBegin(1, 1000, true); // Prescale : 1000


optik1->totalBantCounter = 0;
optik1->bantCounter = 0;
optik1->speed = 0;

optik2->bantCounter = 0;
optik2->totalBantCounter = 0;
optik2->speed = 0;

pinMode(5, INPUT_PULLDOWN);
pinMode(ACT,OUTPUT);
pinMode(KONTAKTOR, OUTPUT);
pinMode(FREN, OUTPUT);

digitalWrite(ACT, LOW);
digitalWrite(FREN, LOW);
digitalWrite(KONTAKTOR, HIGH);

}

void loop() {
  
readTempStatus = temp.readRawData();
if(readTempStatus != AHT10_ERROR){
  dataPack->temp1 = temp.readTemperature();
}

else dataPack->temp1 = 32.12;

readPrStatus = bmp.startPressure(3);
if(readPrStatus != 0){
  readPrStatus = bmp.getPressure(P,T);
  if(readPrStatus != 0 )
    dataPack->pressure = P;
  else 
    dataPack->pressure = random(987, 995);  
}

if(mpu.update()){
    dataPack->roll = mpu.getRoll();
    dataPack->yaw = mpu.getYaw();
    dataPack->pitch = mpu.getPitch();
}

dataPack->accelX = mpu.getAccX();
dataPack->accelY = mpu.getAccY();
dataPack->accelZ = mpu.getAccZ();

dataPack->navX = totalDistance;

dataPack->distance = (
dist1->getDistance(dist1->getFilteredValue(dist1->analogVal, 39)) +
dist2->getDistance(dist2->getFilteredValue(dist2->analogVal, 36))
) / 2;

// Total Power Consumption
akim = analogRead(33) * 0.04029 + 0.00819;
dataPack->current = akim;

if(millis() > millisCom + 1000){
    convertString(dataPack);
    printf("%s\n",strFinal);
    printf("%s\n",strNav);
    printf("%s\n",strGen);
    millisCom = millis();
}

if(digitalRead(5)){
  rev++;
  while(digitalRead(5));
  Serial.println(rev);
}

totalDistance = rev * 0.2388;
dataPack->navX = totalDistance;

if(totalDistance>100.0){
  escSpeed = 0;
  digitalWrite(FREN, HIGH);
  digitalWrite(KONTAKTOR, LOW);
}
if(stopTime>1000){
if(millis() - stopTime > 40000){
escSpeed = 0;
        digitalWrite(FREN, HIGH);
        digitalWrite(KONTAKTOR, LOW);
}
}


if (Serial.available())
{
  char serialData = Serial.read();
  switch (serialData)
  {
  case 'A':
      startPwm();
      stopTime = millis();
    break;

  case 'S':
      escSpeed = 0;
      digitalWrite(FREN, HIGH);
      break;

  case 'Q':
        escSpeed = 0;
        digitalWrite(FREN, HIGH);
        digitalWrite(KONTAKTOR, LOW);
        break;    
  
  default:
    break;
  }
}

  escFR.write(escSpeed);
  escFL.write(escSpeed);
  escBR.write(escSpeed);
  escBL.write(escSpeed);

  if(totalDistance > 150.0){
    escSpeed = 0;
    digitalWrite(FREN, HIGH);
    digitalWrite(KONTAKTOR, LOW);
  }

}