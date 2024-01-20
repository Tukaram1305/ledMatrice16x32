#include "arduinoFFT.h"
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

arduinoFFT FFT = arduinoFFT();

double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long milli = 0;
int del = 100; // opoznienie mapowania 1/1000
uint8_t divider = 0;
uint16_t treshold = 200; // th za transformata
unsigned int SF = 26; // ~38 000 Hz
unsigned long microsec = 0x00000000;;
byte lcd_rf = 1;

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
Serial.begin(115200);
analogReadResolution(12);
analogSetAttenuation(ADC_11db);
adcAttachPin(15);
pinMode(4,OUTPUT);
pinMode(16,OUTPUT);
pinMode(17,OUTPUT);
pinMode(5,OUTPUT);
pinMode(18,OUTPUT);
pinMode(19,OUTPUT);
zero();
}

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

int sum;

void loop() {
/*SAMPLING*/  


    for(int i=0; i<SAMPLES; i++)
{

      //Serial.println(analogRead(15));
        
        sum = (analogRead(15)-2000);
        if (sum > -60 && sum < 60)
        {sum = 0;}
        vReal[i] = sum;
        vImag[i] = 0;
 
     //while (micros()<microsec+SF){}
     //microsec=micros();

     //draw(10);
     //else if (i>112){decay();}
     //delayMicroseconds(SF);
     while (micros()<(microsec+SF)) {draw(1);}
     microsec=micros();
}
zero();
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
     // else if (vReal[i]>=1500 && vReal[i] < 2000) {vReal[i] / 100 ;}
      else if (vReal[i]>=1990) {vReal[i] = (vReal[i]*0.25) ;}

    }
    #endif

    #if ALIAS == 2
    divider = (SAMPLES/2)/16; // 8 dla 128 / 16 dla 256(512sps)
        for (int i = 0; i < SAMPLES/2; i++)
    {
      if (vReal[i] < treshold) {vReal [i] = 0;}
      vReal[i] = round(vReal[i]);
      vReal[i] = vReal[i] / divider;
      if ((i+1)%16==0 && divider >1){divider--;}
    }

    #endif
    
    //double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    //Serial.println(peak);     //Print out what frequency is the most dominant.
    
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
    
      //vReal[0]=vReal[0]/2;
      //vReal[1]=vReal[1]-400;
      //vReal[2]=vReal[2]-250;
      
        for (int i = 0; i < 32; i++)
    {
      vReal[i] = round(vReal[i]);
      if (vReal[i]>2000) vReal[i]=2000;
      vReal[i] = map(vReal[i],0,2000,0,15);
    }

  for (byte i=0; i<32; i++)
  {
    for (byte cx=0; cx<15; cx++)
    {
      lcd[cx][i]=0;
    }

    for (byte fx=1; fx<=vReal[i]; fx++)
    {
      lcd[fx-1][i]=1;
    }
  }

  draw(80);
}
