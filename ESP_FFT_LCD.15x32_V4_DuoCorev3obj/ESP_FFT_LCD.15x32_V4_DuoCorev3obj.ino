TaskHandle_t FFTcompute;
//TaskHandle_t LCDmanage;

#include "arduinoFFT.h"
#include "LedCD.h"

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
#define CA 33
#define CB 32
#define CD 35
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

  adcAttachPin(CA);
  adcAttachPin(CB);
  adcAttachPin(CD);
  //adcAttachPin(C4);
  analogSetPinAttenuation(CA, ADC_6db);
  analogSetPinAttenuation(CB, ADC_6db);
  analogSetPinAttenuation(CD, ADC_6db);
  //analogSetPinAttenuation(pin, attenuation)

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
  Serial.print("Obliczanie FFT na core: ");
  Serial.println(xPortGetCoreID());

  // GLOWNA PETLA 0 RDZENIA //
  for (;;) {

    if (showMode == 0) {
      for (int i = 0; i < SAMPLES; i++)
      {
        sum = (analogRead(15) - 2000);
        if (sum > -60 && sum < 60)
        {
          sum = 0;
        }
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

      for (int i = 0; i < SAMPLES / 2; i++)
      {
        vReal[i] = round(vReal[i]);
        if (vReal[i] > 2000) vReal[i] = 2000;
        mapped[i] = map(vReal[i], 0, 2000, 0, 15);
      }

      clearM();
      for (byte i = 0; i < 32; i++)
      {
        for (byte fx = 1; fx <= mapped[i ]; fx++)
        {
          lcd[fx - 1][i] = 1;
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
    }//SWITCH
  }//LENGHT
}//fx draw

void drawVal(byte line, int val)
{
  char cVal[7];
  itoa(val, cVal, 10);
  byte len = strlen(cVal);
  //clearM();
  //Serial.println(String("DIGIT LEN: : ")+len+String(" / Char = ")+cVal);
 drawSStrAt(line, 2, cVal);

/*
  if (line == 0) {
    for (byte i = 0; i < len; i++)
    {
      drawSStrAt(0, 2, cVal);
    }
  }

  if (line == 1) {
    for (byte i = 0; i < len; i++)
    {
      drawSCharAt(0, 4 + i * 5, cVal[i]);
    }
  }
  
  //draw(40);*/
}

void checkAdc()
{

  
  if (adcRead[0] > (adcReadPrev[0] + 5) || adcRead[0] < (adcReadPrev[0] - 5))
  {
    adcReadPrev[0] = adcRead[0];
    showMode = 1;
    cAdcCh = 1;
    milli = millis();
  }
 if (adcRead[1] > (adcReadPrev[1] + 2) || adcRead[1] < (adcReadPrev[1] - 2))
  {
    adcReadPrev[1] = adcRead[1];
    showMode = 1;
    cAdcCh = 2;
    milli = millis();
  }
 if (adcRead[2] > (adcReadPrev[2] + 2) || adcRead[2] < (adcReadPrev[2] - 2))
  {
    adcReadPrev[2] = adcRead[2];
    showMode = 1;
    cAdcCh = 3;
    milli = millis();
  }

  if (showMode == 1 && cAdcCh == 1) {
    clearM();
    drawVal(0, adcRead[0]);
    drawSStrAt(8, 1, "adc a");
  }
  if (showMode == 1 && cAdcCh == 2) {
    clearM();
    drawVal(0, adcRead[1]);
    drawSStrAt(8, 1, "adc b");
  }
  if (showMode == 1 && cAdcCh == 3) {
    clearM();
    drawVal(0, adcRead[2]);
    drawSStrAt(8, 1, "adc c");
  }
}

void loop() {

  //zero();
#if DEBUG == 1
  Serial.println("ZACZYNAM OKIENKO:");
  for (int i = 4; i < (SAMPLES / 2); i++)
  {
    Serial.print("BIN: ");
    Serial.print(i);
    Serial.print(" -> ");
    Serial.println(vReal[i], 1);
  }
  Serial.println("KONCZE OKIENKO:");
#endif

  adcRead[0] = map(analogRead(CA), 0, 4095, 0, 100);
  adcRead[1] = map(analogRead(CB), 0, 4095, 0, 15);
  adcRead[2] = map(analogRead(CD), 0, 4095, 0, 32);
  //adcRead[3] = analogRead(C4);

  checkAdc();

  /*
    clearM();
    for (int x=0; x<3; x++)
    {
    for (int y=0; y<3; y++)
    {
      lcd[x+adcRead[1]][y+adcRead[2]] = test[x][y];
    }
    }
  */
#if DEBUG == 1
  Serial.println(String("ADC_1 = ") + adcRead[0]);
  Serial.println(String("ADC_2 = ") + adcRead[1]);
  //Serial.println(adcRead[2]);
  //Serial.println(adcRead[3]);
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
#endif


  draw(60); // rysuj pixele z macierzy
  
  if (showMode == 1 && millis() > (milli + menuDelay)) {
    showMode = 0;
    cAdcCh = 0;
  }
}
