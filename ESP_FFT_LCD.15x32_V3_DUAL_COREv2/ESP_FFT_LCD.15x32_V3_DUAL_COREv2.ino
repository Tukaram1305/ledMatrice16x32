TaskHandle_t FFTcompute;
//TaskHandle_t LCDmanage;

#include "arduinoFFT.h"
#include "LedCD.h"

#define samples_per_read
#define SAMPLES 128
#define SAMPLING_FREQUENCY 38000
#define ALIAS 2 // 0 brak , 1 ostry , 2 wykladniczy / 4-treshold
#define DEBUG 0
#define samples_per_cycle 64

#define xd 4
#define xl 16
#define xc 17
#define yd 5
#define yl 18
#define yc 19

#define AudioLine 15
#define C1 33
#define C2 32
#define C3 35
#define C4 34

arduinoFFT FFT = arduinoFFT();

double         vReal[SAMPLES];
double         vImag[SAMPLES];
unsigned long  milli = 0;
unsigned int   SF = 26; // ~38 000 Hz
unsigned long  microsec = 0x00000000;;
int            sum; // wartosc z AnalogRead
byte           mapped[SAMPLES / 2];

byte           modes = 0; // 0-FFT / 1-Wave / 2-String
int            adcRead[4];
int            adcReadPrev[4];
byte           cAdcCh = 0;
bool           showMode = 0;
int            menuDelay = 2500; // opoznienie menu w milisec
uint32_t       refreshDelayMicro = 80;
byte           noise_reduction = 80;
int            gain_trshld = 800;

float normalizer = 0.74; // base 0.74
float norm_gain = 0.14;  // base~0.05

//char txtest [7] = {"tst"};

bool lcd [15][32] = {
  {1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1},
  {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
  {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
  {1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1},
  {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
  {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
  {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
};

void draw(int rf);
void zero();
void clearM ();
void drawSCharAt(int sx, int sy, char sch);
void drawSStrAt(int sx, int sy, char str[]);
void shiftMat(byte dir, byte line, int steps);
void drawVal(byte line, int val);
void checkAdc();

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


  Serial.begin(115200);
  analogReadResolution(12);
  analogSetCycles(samples_per_cycle); // sample na konwersje (default 8)
  analogSetAttenuation(ADC_11db);
  adcAttachPin(AudioLine); // audio in

  adcAttachPin(C1);
  adcAttachPin(C2);
  adcAttachPin(C3);
  adcAttachPin(C4);

  analogSetPinAttenuation(C1, ADC_11db);
  analogSetPinAttenuation(C2, ADC_11db);
  analogSetPinAttenuation(C3, ADC_11db);
  analogSetPinAttenuation(C4, ADC_11db);

  // zarzadzanie rejestrami
  pinMode(4, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  zero();
}
// KONIEC SETUP

void FFTcomputeCode( void * pvParameters ) {

  // GLOWNA PETLA 0 RDZENIA //
  for (;;) {

    if (showMode == 0) {
      for (int i = 0; i < SAMPLES; i++)
      {
        sum = (analogRead(15) - 2000);
        if (sum > -noise_reduction && sum < noise_reduction) {
          sum = 0;
        }
        vReal[i] = sum;
        vImag[i] = 0;

        delayMicroseconds(SF);
      }
      /*FFT*/
      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);


#if ALIAS == 1
      for (int i = 0; i < SAMPLES / 2; i++)
      {
        //vReal[i] = round(vReal[i]);
        if (vReal[i] < 800) {
          vReal[i] = vReal[i] * 1.8;
        }
        //else if (vReal[i] >= 1000 && vReal[i] <1500) {vReal[i] / 10;}
        //else if (vReal[i]>=1500 && vReal[i] < 2000) {vReal[i] / 100 ;}
        else if (vReal[i] >= 1990) {
          vReal[i] = (vReal[i] * 0.25) ;
        }
      }
#endif

#if ALIAS == 4 // program (custom) A2
      for (int i = 0; i < SAMPLES / 2; i++)
      {
        if (vReal[i] < gain_trshld - gain_trshld / 2 ) {
          vReal[i] = (vReal[i] * 0.35) ;
        }
        else if (vReal[i] < gain_trshld && vReal[i] < 1989) {
          vReal[i] = vReal[i] * 1.6;
        }
        else if (vReal[i] >= 1990) {
          vReal[i] = (vReal[i] * 0.26) ;
        }
      }
#endif

#if ALIAS == 2 
      normalizer = 0.74;
      for (int i = 0; i < SAMPLES / 2; i++)
      {
        normalizer+=norm_gain;
        vReal[i] = vReal[i]*normalizer;
      }
#endif


      for (int i = 0; i < SAMPLES / 2; i++)
      {
        vReal[i] = round(vReal[i]);
        if (vReal[i] > 2000) vReal[i] = 2000;
        mapped[i] = map(vReal[i], 0, 2000, 0, 15);
      }

      clearM();

      // GLOWNE WYPELNIANIE MACIERZY FFT //
      byte FFT_offset = adcRead[3]; // offset spektrum 0-64 (SAMPLES/2)

      for (byte y = FFT_offset; y < (FFT_offset + 32); y++)
      {
        for (byte x = 1; x <= mapped[y]; x++)
        {
          lcd[x - 1][y - FFT_offset] = 1;
        }
      }
    }

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
  for (byte x = 0; x < 15; x++)
  {
    digitalWrite(xl, LOW);
    digitalWrite(xc, LOW);
    digitalWrite(xd, LOW);
    digitalWrite(xc, HIGH);
  }
  for (byte y = 0; y < 32; y++)
  {
    digitalWrite(yl, LOW);
    digitalWrite(yc, LOW);
    digitalWrite(yd, HIGH);
    digitalWrite(yc, HIGH);
  }
  digitalWrite(xl, HIGH);
  digitalWrite(yl, HIGH);
}

void draw(int rf)
{
  for (byte i = 0; i < rf; i++) {
    for (byte y = 0; y < 32; y++)
    {
      digitalWrite(xl, LOW);
      digitalWrite(yl, LOW);

      digitalWrite(yc, LOW);
      if (y == 0) {
        digitalWrite(yd, LOW);
      }
      else {
        digitalWrite(yd, HIGH);
      }
      digitalWrite(yc, HIGH);

      digitalWrite(xl, LOW);
      for (byte x = 0; x < 15; x++)
      {
        digitalWrite(xc, LOW);
        digitalWrite(xd, lcd[x][31 - y]);
        digitalWrite(xc, HIGH);
      }
      digitalWrite(xl, HIGH);
      digitalWrite(yl, HIGH);
      delayMicroseconds(refreshDelayMicro);

    }
  }

}

void clearM ()
{

  for (byte y = 0; y < 32; y++)
  {
    for (byte x = 0; x < 15; x++)
    {
      lcd[x][y] = 0;
    }
  }

}

void shiftMat(byte dir, byte line, int steps)
/*
  dir   [0-left ; 1-right]
  line  [0-top ; 1-bot]
  steps -xxx,
*/
{
  // LEWO - TOP
  if (dir == 0 && line == 0) {
    for (int stp = 0; stp < steps; stp++) {
      for (byte x = 8; x < 15; x++)
      {
        for (byte y = 1; y < 32; y++)
        {
          if (y != 31)
          {
            lcd[x][y - 1] = lcd[x][y];
          }
          else
          {
            lcd[x][y] = 0;
          }
        }
      }
    }
  } // END

  // PEAWO - TOP
  if (dir == 1 && line == 0) {
    for (int stp = 0; stp < steps; stp++) {
      for (byte x = 8; x < 15; x++)
      {
        for (byte y = 31; y >= 0; y--)
        {
          if (y != 0)
          {
            lcd[x][y] = lcd[x][y - 1];
          }
          else
          {
            lcd[x][y] = 0;
          }
        }
      }
    }
  } // END

  // LEWO - BOTTOM
  if (dir == 0 && line == 1) {
    for (int stp = 0; stp < steps; stp++) {
      for (byte x = 0; x < 7; x++)
      {
        for (byte y = 1; y < 32; y++)
        {
          if (y != 31)
          {
            lcd[x][y - 1] = lcd[x][y];
          }
          else
          {
            lcd[x][y] = 0;
          }
        }
      }
    }
  } // END

  // PEAWO - BOTTOM
  if (dir == 1 && line == 1) {
    for (int stp = 0; stp < steps; stp++) {
      for (byte x = 0; x < 7; x++)
      {
        for (byte y = 31; y >= 0; y--)
        {
          if (y != 0)
          {
            lcd[x][y] = lcd[x][y - 1];
          }
          else
          {
            lcd[x][y] = 0;
          }
        }
      }
    }
  } // END

}

void drawSCharAt(int sx, int sy, char sch)
{
  switch (sch)
  {
    case '0':
      {
        for (int x = 0; x < 7; x++)
        {
          for (int y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_0[6 - x][y];
          }
        }
        break;
      }//case

    case '1':
      {
        for (int x = 0; x < 7; x++)
        {
          for (int y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_1[6 - x][y];
          }
        }
        break;
      }//case

    case '2':
      {
        for (int x = 0; x < 7; x++)
        {
          for (int y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_2[6 - x][y];
          }
        }
        break;
      }//case

    case '3':
      {
        for (byte x = 0; x < 7; x++)
        {
          for (byte y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_3[6 - x][y];
          }
        }
        break;
      }//case

    case '4':
      {
        for (byte x = 0; x < 7; x++)
        {
          for (byte y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_4[6 - x][y];
          }
        }
        break;
      }//case

    case '5':
      {
        for (byte x = 0; x < 7; x++)
        {
          for (byte y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_5[6 - x][y];
          }
        }
        break;
      }//case

    case '6':
      {
        for (byte x = 0; x < 7; x++)
        {
          for (byte y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_6[6 - x][y];
          }
        }
        break;
      }//case

    case '7':
      {
        for (byte x = 0; x < 7; x++)
        {
          for (byte y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_7[6 - x][y];
          }
        }
        break;
      }//case

    case '8':
      {
        for (byte x = 0; x < 7; x++)
        {
          for (byte y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_8[6 - x][y];
          }
        }
        break;
      }//case

    case '9':
      {
        for (byte x = 0; x < 7; x++)
        {
          for (byte y = 0; y < 4; y++)
          {
            lcd[sx + x][sy + y] = slf_9[6 - x][y];
          }
        }
        break;
      }//case
    default: break;
  }//switch
}//fx

void drawSStrAt(int sx, int sy, char str[])
{
  byte len = strlen(str);
  byte last_len;
  char curr_char;
  byte curs_x = sx;
  byte curs_y = sy;

  for (int i = 0; i < len; i++)
  {
    curr_char = str[i];

#if DEBUG == 1
    Serial.println("---- DRAW STRING ----");
    Serial.print("LEN: ");
    Serial.println(len);
    Serial.print("CUR CHAR: ");
    Serial.println(curr_char);
    Serial.print("CURS_Y: ");
    Serial.println(curs_y);
    Serial.print("LAST_LEN: ");
    Serial.println(last_len);
    Serial.println("---- DRAW STRING END ----");
#endif

    switch (curr_char)
    {
      case ' ':
        {
          last_len = sizeof(slf_space[0]) / sizeof(slf_space[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_space[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }
      //CASE

      case '0':
        {
          last_len = sizeof(slf_0[0]) / sizeof(slf_0[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_0[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE

      case '1':
        {
          last_len = sizeof(slf_1[0]) / sizeof(slf_1[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_1[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE

      case '2':
        {
          last_len = sizeof(slf_2[0]) / sizeof(slf_2[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_2[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE

      case '3':
        {
          last_len = sizeof(slf_3[0]) / sizeof(slf_3[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_3[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE

      case '4':
        {
          last_len = sizeof(slf_4[0]) / sizeof(slf_4[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_4[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE

      case '5':
        {
          last_len = sizeof(slf_5[0]) / sizeof(slf_5[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_5[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '6':
        {
          last_len = sizeof(slf_6[0]) / sizeof(slf_6[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_6[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '7':
        {
          last_len = sizeof(slf_7[0]) / sizeof(slf_7[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_7[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '8':
        {
          last_len = sizeof(slf_8[0]) / sizeof(slf_8[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_8[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '9':
        {
          last_len = sizeof(slf_9[0]) / sizeof(slf_9[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_9[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      // ------------- LETTERS ------------------------ //
      case 'a':
        {
          last_len = sizeof(slf_a[0]) / sizeof(slf_a[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_a[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'b':
        {
          last_len = sizeof(slf_b[0]) / sizeof(slf_b[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_b[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'c':
        {
          last_len = sizeof(slf_c[0]) / sizeof(slf_c[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_c[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'd':
        {
          last_len = sizeof(slf_d[0]) / sizeof(slf_d[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_d[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'e':
        {
          last_len = sizeof(slf_e[0]) / sizeof(slf_e[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_e[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'f':
        {
          last_len = sizeof(slf_f[0]) / sizeof(slf_f[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_f[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'g':
        {
          last_len = sizeof(slf_g[0]) / sizeof(slf_g[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_g[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'h':
        {
          last_len = sizeof(slf_h[0]) / sizeof(slf_h[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_h[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'i':
        {
          last_len = sizeof(slf_i[0]) / sizeof(slf_i[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_i[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'j':
        {
          last_len = sizeof(slf_j[0]) / sizeof(slf_j[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_j[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'k':
        {
          last_len = sizeof(slf_k[0]) / sizeof(slf_k[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_k[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'l':
        {
          last_len = sizeof(slf_l[0]) / sizeof(slf_l[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_l[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'm':
        {
          last_len = sizeof(slf_m[0]) / sizeof(slf_m[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_m[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'n':
        {
          last_len = sizeof(slf_n[0]) / sizeof(slf_n[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_n[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'o':
        {
          last_len = sizeof(slf_o[0]) / sizeof(slf_o[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_o[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'p':
        {
          last_len = sizeof(slf_p[0]) / sizeof(slf_p[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_p[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'r':
        {
          last_len = sizeof(slf_r[0]) / sizeof(slf_r[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_r[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 's':
        {
          last_len = sizeof(slf_s[0]) / sizeof(slf_s[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_s[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 't':
        {
          last_len = sizeof(slf_t[0]) / sizeof(slf_t[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_t[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'u':
        {
          last_len = sizeof(slf_u[0]) / sizeof(slf_u[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_u[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'w':
        {
          last_len = sizeof(slf_w[0]) / sizeof(slf_w[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_w[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'x':
        {
          last_len = sizeof(slf_x[0]) / sizeof(slf_x[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_x[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'y':
        {
          last_len = sizeof(slf_y[0]) / sizeof(slf_y[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_y[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case 'z':
        {
          last_len = sizeof(slf_z[0]) / sizeof(slf_z[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_z[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      /* - - - - - ZNAKI - - - - - */
      case '!':
        {
          last_len = sizeof(slf_$1[0]) / sizeof(slf_$1[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$1[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '?':
        {
          last_len = sizeof(slf_$2[0]) / sizeof(slf_$2[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$2[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '>':
        {
          last_len = sizeof(slf_$3[0]) / sizeof(slf_$3[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$3[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '<':
        {
          last_len = sizeof(slf_$4[0]) / sizeof(slf_$4[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$4[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '(':
        {
          last_len = sizeof(slf_$5[0]) / sizeof(slf_$5[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$5[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case ')':
        {
          last_len = sizeof(slf_$6[0]) / sizeof(slf_$6[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$6[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '-':
        {
          last_len = sizeof(slf_$7[0]) / sizeof(slf_$7[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$7[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '+':
        {
          last_len = sizeof(slf_$8[0]) / sizeof(slf_$8[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$8[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case ':':
        {
          last_len = sizeof(slf_$9[0]) / sizeof(slf_$9[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$9[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '.':
        {
          last_len = sizeof(slf_$a[0]) / sizeof(slf_$a[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$a[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '/':
        {
          last_len = sizeof(slf_$c[0]) / sizeof(slf_$c[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$c[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '}':
        {
          last_len = sizeof(slf_$d[0]) / sizeof(slf_$d[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$d[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '{':
        {
          last_len = sizeof(slf_$e[0]) / sizeof(slf_$e[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$e[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '"':
        {
          last_len = sizeof(slf_$h[0]) / sizeof(slf_$h[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$h[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
      case '#':
        {
          last_len = sizeof(slf_$j[0]) / sizeof(slf_$j[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$j[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
              case '%':
        {
          last_len = sizeof(slf_$l[0]) / sizeof(slf_$l[0][0]);
          for (int x = 0; x < 7; x++)
          {
            for (int y = 0; y < last_len; y++)
            {
              lcd[x + curs_x][y + curs_y] = slf_$l[6 - x][y];
            }
          }
          curs_y = (curs_y + last_len + 1);
          break;
        }//CASE
    }//SWITCH
  }//LENGHT
}//fx draw string

void drawVal(byte x, byte y, int val)
{
  char cVal[7];
  itoa(val, cVal, 10);
  byte len = strlen(cVal);
  //clearM();
  //Serial.println(String("DIGIT LEN: : ")+len+String(" / Char = ")+cVal);
  drawSStrAt(x, y, cVal);
}

void checkAdc()
{
  byte a0of = 100; // 
  byte a1of = 2; // 
  byte a2of = 5; // 160 noise
  byte a3of = 2; // 31  offset

  // 4 kanaly ADC / 3x POT , 1x SLIDER ch3 in4
  // sprawdzanie zmiany kanalow
  if (adcRead[0] > (adcReadPrev[0] + a0of) || adcRead[0] < (adcReadPrev[0] - a0of))
  {
    adcReadPrev[0] = adcRead[0];
    showMode = 1;
    cAdcCh = 1;
    milli = millis();
  }
  if (adcRead[1] > (adcReadPrev[1] + a1of) || adcRead[1] < (adcReadPrev[1] - a1of))
  {
    adcReadPrev[1] = adcRead[1];
    showMode = 1;
    cAdcCh = 2;
    milli = millis();
  }
  if (adcRead[2] > (adcReadPrev[2] + a2of) || adcRead[2] < (adcReadPrev[2] - a2of))
  {
    adcReadPrev[2] = adcRead[2];
    showMode = 1;
    cAdcCh = 3;
    milli = millis();
    noise_reduction = adcRead[2];
  }

  if (adcRead[3] > (adcReadPrev[3] + a3of) || adcRead[3] < (adcReadPrev[3] - a3of))
  {
    adcReadPrev[3] = adcRead[3];
    showMode = 1;
    cAdcCh = 4;
    milli = millis();
  }

  // akcje po wykryciu zmiany
  if (showMode == 1 && cAdcCh == 1) {
    clearM();
    drawVal(0, 7, adcRead[0]);
    drawSStrAt(8, 1, "ref-hz");
    refreshDelayMicro = map(adcRead[0], 0, 4095, 80, 100000);
    menuDelay = 3000;
  }
  if (showMode == 1 && cAdcCh == 2) {
    clearM();
    drawVal(0, 7, adcRead[1]);
    drawSStrAt(8, 1, "gain");
    menuDelay = 2500;
    norm_gain = adcRead[1]/100;
    //gain_trshld = adcRead[1];
  }
  if (showMode == 1 && cAdcCh == 3) {
    clearM();
    drawVal(0, 7, adcRead[2]);
    drawSStrAt(8, 1, "noise");
    menuDelay = 2000;
  }
  if (showMode == 1 && cAdcCh == 4) {
    clearM();
    drawVal(0, 7, adcRead[3]);
    drawSStrAt(8, 4, "ofset");
    menuDelay = 1500;
  }
}

void debug()
{

  Serial.println("ZACZYNAM OKIENKO:");
  for (int i = 4; i < (SAMPLES / 2); i++)
  {
    Serial.print("BIN: ");
    Serial.print(i);
    Serial.print(" -> ");
    Serial.println(vReal[i], 1);
  }
  Serial.println("KONCZE OKIENKO:");
  Serial.print("Obliczanie FFT na core: ");
  Serial.println(xPortGetCoreID());

  Serial.println(String("ADC_1 = ") + adcRead[0]);
  Serial.println(String("ADC_2 = ") + adcRead[1]);
  Serial.println(String("ADC_3 = ") + adcRead[2]);
  Serial.println(String("ADC_3 = ") + adcRead[3]);
  Serial.println(String("ShowMenu: ") + showMode);

  for (int x = 0; x < 15; x++)
  {
    for (int y = 0; y < 32; y++)
    {
      Serial.print(lcd[x][y]);
    }
    Serial.println("");
  }
  Serial.println("-------------------------");
}

void loop() {

  //zero();


  adcRead[0] = analogRead(C1);
  adcRead[1] = map(analogRead(C2), 0, 4095, 0, 30);
  adcRead[2] = map(analogRead(C3), 0, 4095, 0, 160);
  adcRead[3] = map(analogRead(C4), 0, 4095, 0, 31);


  checkAdc();
  draw(60); // rysuj pixele z macierzy


  if (showMode == 1 && millis() > (milli + menuDelay)) {
    showMode = 0;
    cAdcCh = 0;
  }

#if DEBUG == 1
  debug();
#endif

}
