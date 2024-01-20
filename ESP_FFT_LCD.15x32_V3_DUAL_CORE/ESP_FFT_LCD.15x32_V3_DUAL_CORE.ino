TaskHandle_t FFTcompute;
//TaskHandle_t LCDmanage;

#include "arduinoFFT.h"
#include "LcdFonts.h"
#define SAMPLES 128 
#define SAMPLING_FREQUENCY 38000
#define ALIAS 1 // 0 brak , 1 ostry , 2 wykladniczy
#define DEBUG 0

#define xd 4
#define xl 16
#define xc 17
#define yd 5
#define yl 18
#define yc 19

#define C1 12
#define C2 14
#define C3 27
#define C4 26

arduinoFFT FFT = arduinoFFT();

double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long milli = 0;
unsigned int SF = 26; // ~38 000 Hz
unsigned long microsec = 0x00000000;;
int sum; // wartosc z AnalogRead
byte mapped[15];

byte modes = 0; // 0-FFT / 1-Wave / 2-String
int adcRead[4];
bool showMode = 0;

void draw(uint16_t rf);
void zero();
void decay();

bool lcd [15][32] ={
{1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1},
{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1},
{1,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,1},
{1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
{1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1},
};

void setup() {

  xTaskCreatePinnedToCore(
                    FFTcomputeCode,   /* Task function. */
                    "FFTcompute",     /* name of task. */
                    10000,            /* Stack size of task */
                    NULL,             /* parameter of the task */
                    1,                /* priority of the task */
                    &FFTcompute,      /* Task handle to keep track of created task */
                    0);               /* pin task to core 0 */                  
  delay(250); 

/* // loop dla core1 - to samo co base loop
  xTaskCreatePinnedToCore(
                    LCDmanageCode,   
                    "LCDmanage",    
                    10000,      
                    NULL,        
                    1,          
                    &LCDmanage,      
                    1);         
    delay(250); 
*/    
Serial.begin(115200);
analogReadResolution(12);
analogSetAttenuation(ADC_11db);
adcAttachPin(15);

adcAttachPin(C1);
adcAttachPin(C2);
adcAttachPin(C3);
adcAttachPin(C4);
pinMode(4,OUTPUT);
pinMode(16,OUTPUT);
pinMode(17,OUTPUT);
pinMode(5,OUTPUT);
pinMode(18,OUTPUT);
pinMode(19,OUTPUT);
zero();
}
// KONIEC SETUP

void FFTcomputeCode( void * pvParameters ){
  Serial.print("Obliczanie FFT na core: ");
  Serial.println(xPortGetCoreID());

  // GLOWNA PETLA 0 RDZENIA //
  for(;;){

    if (modes==0 && showMode == 0){
    for(int i=0; i<SAMPLES; i++)
{
        sum = (analogRead(15)-2000);
        if (sum > -60 && sum < 60)
        {sum = 0;}
        vReal[i] = sum;
        vImag[i] = 0;

     delayMicroseconds(SF);
     //while (micros()<(microsec+SF)) {draw(1);}
     //microsec=micros();
}
    /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    //double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

    
    #if ALIAS == 1
    for (int i = 0; i < SAMPLES/2; i++)
    {
     //vReal[i] = round(vReal[i]);
      if (vReal[i] < 800) {vReal[i]=vReal[i]*1.8;}
     //else if (vReal[i] >= 1000 && vReal[i] <1500) {vReal[i] / 10;}
     //else if (vReal[i]>=1500 && vReal[i] < 2000) {vReal[i] / 100 ;}
      else if (vReal[i]>=1990) {vReal[i] = (vReal[i]*0.25) ;}
    }
    #endif

   for (int i = 0; i < 32; i++)
    {
      vReal[i] = round(vReal[i]);
      if (vReal[i]>2000) vReal[i]=2000;
      mapped[i] = map(vReal[i],0,2000,0,15);
    }

  for (byte i=0; i<32; i++)
  {
    for (byte cx=0; cx<15; cx++)
    {
      lcd[cx][i]=0;
    }

    for (byte fx=1; fx<=mapped[i]; fx++)
    {
      lcd[fx-1][i]=1;
    }
  }}
  
  vTaskDelay(10);
  }
}

/*
void LCDmanageCode( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){

  }
}
*/
// KONIEC TASK

void zero()
{
  for (byte x=0; x<15; x++)
  {
    digitalWrite(xl,LOW);
    digitalWrite(xc,LOW);
    digitalWrite(xd,LOW);
    digitalWrite(xc,HIGH);
  }
  for (byte y=0; y<32; y++)
  {
    digitalWrite(yl,LOW);
    digitalWrite(yc,LOW);
    digitalWrite(yd,HIGH);
    digitalWrite(yc,HIGH);
  }
  digitalWrite(xl,HIGH);
  digitalWrite(yl,HIGH);
}

void draw(uint16_t rf)
{
  for (byte i=0; i<rf; i++){
  for (byte y=0; y<32; y++)
  {  
    digitalWrite(xl,LOW);
    digitalWrite(yl,LOW);

    digitalWrite(yc,LOW);
    if (y==0) {digitalWrite(yd,LOW);}
    else {digitalWrite(yd,HIGH);}
    digitalWrite(yc,HIGH);
    
    for (byte x=0; x<15; x++)
    {
        digitalWrite(xc,LOW);
        digitalWrite(xd,lcd[x][y]);
        digitalWrite(xc,HIGH);
    }     
    digitalWrite(xl,HIGH);
    digitalWrite(yl,HIGH);    
          
    //delayMicroseconds(4);

  }}
  
}

void decay()
{
  for (byte y=0; y<32; y++)
  {
    for (byte x=14; x>=0; x--)
    {
      if (lcd[x][y]==1) 
      {
        lcd[x][y]=0;
        break;
      }
    }
  }
  draw(10);
}

void loop() {
    //zero();
    #if DEBUG == 1
    Serial.println("ZACZYNAM OKIENKO:");
    for(int i=4; i<(SAMPLES/2); i++)
    {
        //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
        //Serial.print(" ");
        Serial.print("BIN: ");
        Serial.print(i);
        Serial.print(" -> ");
        Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
    }  
    Serial.println("KONCZE OKIENKO:");
    #endif

    adcRead[0] = analogRead(C1);
    adcRead[1] = analogRead(C2);
    adcRead[2] = analogRead(C3);
    adcRead[3] = analogRead(C4);

    modes = map(adcRead[0],0,4096,0,4);

  draw(80); // rysuj pixele z macierzy
}
