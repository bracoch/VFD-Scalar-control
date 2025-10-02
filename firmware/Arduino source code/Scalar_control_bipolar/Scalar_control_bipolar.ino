#include <Wire.h>
#include <LiquidCrystal.h>
#include <ACS712.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

const int rs = 22, en = 23, d4 = 24, d5 = 25, d6 = 26, d7 = 27;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
ACS712 ACS(A2, 5.0, 1023, 100);

// Arduino Mega has 5.0 volts max ADC value of 1023 steps
// ACS712 5A  uses 185 mV per A   //
// ACS712 20A uses 100 mV per A  // Constants for ASC712 depending on the type
// ACS712 30A uses 66 mV per A   //


volatile uint8_t numR;  // pointer to the value in the sine table
bool smer = false;      // flag for direction animation
int Animacija;          // variable for animation
int Flag_tajmer_za_mer_napona = 0;
float Amp;                      // variable for amplitude
int freq;                       // reading AD0 gives frequency
float napon;                    // variable for voltage measurement
int DC_MAX;                     // overvoltage
int DC_MIN;                     // undervoltage
int I_MAX;                      // maximum current
float I_MAX_eeprom;             // store maximum current in EEPROM
int adresa_float_struja = 8;    // EEPROM address for current
int freq_motora_nom;            // motor frequency from nameplate 50–60 Hz
long mA;                        // current measurement
long mA_temp;                   // current measurement
long mA_temp1;                  // current measurement
long mA_Prikaz;                 // current measurement (to display)
float freqF;                    // calculation of output frequency
float dfreq;                    // calculation of output frequency
float freqregulatora;           // calculation of output frequency
float freqscala;                // calculation of output frequency
int PhasaR;                     // Output phase R
int PhasaS;                     // Output phase S
int PhasaT;                     // Output phase T
int releykon = 32;              // pin for switching on mains capacitor relay
int releynapR = 35;             // pin for switching on voltage relay
int releynap0 = 34;             // pin for switching on voltage relay
int enableIGBT = 33;            // block/unblock output transistors
int kontrola = 36;              // frequency control
float speedup = 7;              // min frequency
int rampa_gore;                 // variable for acceleration
int rampa_dole;                 // variable for deceleration
bool flag_podesavanje = false;  // flag for entering settings
int izbor_ekrana = 6;           // number of items for settings
bool flag_ekran = false;        // variable for settings
bool strujna_zastita = false;   // overcurrent protection
int eeprom_prazan;              // EEPROM check
///////////////////////////////

//////////////////////////////////////////////////////////////
int naponsum = 0;           //
int napon_uzorak_broj = 0;  // voltage measurement
////////////////////////////////////////////////////////////
int KeyNumber;             //
int LCD_Buttons_ADC;       //
volatile byte StateVar;    // Variables for keyboard and direction status
volatile int Revers;       //
volatile byte LCD_Button;  //
//////////////////////////////////////////////////////////

//////////////////////////////////////////////
volatile byte PWM_Soft_A;  //
volatile byte PWM_Soft_B;  // Temporary register for PWM duration values
volatile byte PWM_Soft_C;  //
//////////////////////////////////////////////

////////////////////////////////////////////////////
volatile word phaseShiftA = 0;  //
volatile word phaseShiftB = 0;  // Variables for changing rotation direction
volatile word phaseShiftC = 0;  //
///////////////////////////////////////////////////

int start = 0;            // regulator status
#define NAPON_UZORAKA 10  // voltage measurement
#define R_0 46            //
#define S_0 45            // Output pins U-V-W
#define T_0 44            //
////////////////////////////////////////////// Assigning PWM registers to variables (U-V-W)
#define FAZA_U OCR5A
#define FAZA_V OCR5B
#define FAZA_W OCR5C
////////////////////////////////////////////
#define FREQ_POT A0  // analog input for output frequency
int FREQ_MIN = 7;    // default: 0
int FREQ_MAX = 50;   // default: 50
#define POT_MIN 0
#define POT_MAX 1023  // default: 1023

///////////////////////////////////////////////////////////////////////////////////////
// character for LCD direction left
byte levo[8] = {
  0x02, 0x06, 0x0E, 0x1E, 0x0E, 0x06, 0x02, 0x00
};
// character for LCD direction right
byte desno[8] = {
  0x08, 0x0C, 0x0E, 0x0F, 0x0E, 0x0C, 0x08, 0x00
};
//////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Three sine tables with 120° offset
const byte PROGMEM sine[][39] = {
  { 1, 4, 9, 16, 24, 34, 45, 57, 70, 83, 96, 109, 121, 132, 142, 150, 157, 162, 165, 166, 165, 162, 157, 150, 142, 132, 121, 109, 96, 83, 70, 57, 45, 34, 24, 16, 9, 4, 1 },
  { 132, 142, 150, 157, 162, 165, 166, 165, 162, 157, 150, 142, 132, 121, 109, 96, 83, 70, 57, 45, 34, 24, 16, 9, 4, 1, 1, 4, 9, 16, 24, 34, 45, 57, 70, 83, 96, 109, 121 },
  { 121, 109, 96, 83, 70, 57, 45, 34, 24, 16, 9, 4, 1, 1, 4, 9, 16, 24, 34, 45, 57, 70, 83, 96, 109, 121, 132, 142, 150, 157, 162, 165, 166, 165, 162, 157, 150, 142, 132 },
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(kontrola, OUTPUT);      // frequency control
  digitalWrite(kontrola, LOW);    // frequency control
  pinMode(enableIGBT, OUTPUT);    // protection against unwanted start
  digitalWrite(enableIGBT, LOW);  // force LOW state of digital output
  pinMode(releynapR, OUTPUT);     // relay for switching on voltage
  digitalWrite(releynapR, HIGH);
  pinMode(releynap0, OUTPUT);  // relay for switching on voltage
  digitalWrite(releynap0, HIGH);
  pinMode(releykon, OUTPUT);  // relay for switching on capacitor
  digitalWrite(releykon, LOW);
  lcd.createChar(0, levo);   // creating character for left direction display
  lcd.createChar(1, desno);  // creating character for right direction display

  lcd.begin(20, 4);
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print(" Fakultet tehnickih");
  lcd.setCursor(3, 1);
  lcd.print("nauka u Cacku");
  lcd.setCursor(3, 2);
  lcd.print("Univerziteta u");
  lcd.setCursor(4, 3);
  lcd.print("Kragujevcu");
  delay(100);
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Diplomski rad:");
  lcd.setCursor(0, 1);
  lcd.print("Skalarno upravljanje");
  lcd.setCursor(0, 2);
  lcd.print("karakter. trofaznog");
  lcd.setCursor(1, 3);
  lcd.print("asinhronog motora");
  delay(100);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Aleksandar");
  lcd.setCursor(0, 1);
  lcd.print("Brakocevic");
  lcd.setCursor(0, 2);
  lcd.print("broj indeksa:");
  lcd.setCursor(0, 3);
  lcd.print("1220/17");
  delay(100);
  lcd.clear();
  /////////////////////////////////////////////////////////// EEPROM check after system reset and writing parameters into working registers
  eeprom_prazan = EEPROM.read(0);
  if (eeprom_prazan == 255) {
    flag_podesavanje = true;
    podesavanje();
  }
  FREQ_MAX = EEPROM.read(1);
  rampa_gore = EEPROM.read(2);
  rampa_dole = EEPROM.read(3);

  DC_MAX = citanje_INT_from_eeprom(4);
  DC_MIN = citanje_INT_from_eeprom(6);
  freq_motora_nom = EEPROM.read(8);
  I_MAX = citanje_INT_from_eeprom(9);
  ////////////////////////////////////////////////////////////
  lcd.setCursor(0, 0);
  lcd.print("  Current sensor");
  lcd.setCursor(5, 1);
  lcd.print("calibration");
  ACS.autoMidPoint();
  delay(200);
  lcd.setCursor(7, 2);
  lcd.print("OK!!!");
  delay(200);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Switching voltage on");
  delay(200);
  digitalWrite(releynapR, LOW);
  digitalWrite(releynap0, LOW);
  delay(200);
  //lcd.setCursor(0,2);

  //lcd.print("OK!!!");
  delay(200);
  lcd.setCursor(0, 2);
  lcd.print("Capacitor charging");

  delay(200);
  digitalWrite(releykon, HIGH);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Voltage check!!!");
  delay(200);
  lcd.setCursor(7, 3);
  lcd.print("        !!!");
  merenje_DCnapon();
provera_napona:
  delay(200);
  if (napon < DC_MIN) {
    digitalWrite(releynapR, LOW);
    digitalWrite(releynap0, HIGH);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DC voltage below ");
    lcd.setCursor(0, 1);
    lcd.print("limit!!!");
    lcd.setCursor(0, 2);
    lcd.print("Turn off breaker");
    lcd.setCursor(0, 3);
    lcd.print("and check supply.");
    delay(100);
    goto provera_napona;
  }
  if (napon > DC_MAX) {
    digitalWrite(releynapR, LOW);
    digitalWrite(releynap0, HIGH);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DC voltage above ");
    lcd.setCursor(0, 1);
    lcd.print("limit!!!");
    lcd.setCursor(0, 2);
    lcd.print("Turn off breaker");
    lcd.setCursor(0, 3);
    lcd.print("and check supply.");
    delay(1000);
    goto provera_napona;
  } else

    delay(200);
  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("SYSTEM READY!!!");
  delay(500);

  pinMode(R_0, OUTPUT);  // PWM 3A R_0

  pinMode(T_0, OUTPUT);  // PWM 3C S_0

  pinMode(S_0, OUTPUT);  // PWM 4B T_0

  lcd.clear();


  cli();                // stop interrupts
                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  TCCR5A = 0b10101010;  // phase-correct PWM mode // Description is in the diploma thesis
  TCCR5B = 0b00010010;  // no prescaler // Description is in the diploma thesis
  ICR5 = 166;
  OCR5A = 0;
  OCR5B = 0;  // Timer 5 settings
  OCR5C = 0;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // timer 1 settings
  TCCR1A = 0;           // reset the value
  TCCR1B = 0;           // reset the value
  TCNT1 = 0;            // reset the value
  OCR1A = 58608;        // initial value
  TCCR1B = 0b00001001;  // WGM12 bit is 1 for CTC mode and no prescaler
  ////////////////////////////////////////////////////////////////////////////
  // timer 3 settings
  TCCR3A = 0;   // reset the value
  TCCR3B = 0;   // reset the value
  TCNT3 = 0;    // reset the value
  OCR3A = 400;  //

  TCCR3B = 0b00001100;  // WGM12 bit is 1 for CTC mode without prescaler

  TIMSK1 |= (1 << OCIE1A);  // enable interrupts timer 1
  TIMSK3 |= (1 << OCIE3A);  // enable interrupts timer 3
  sei();                    //
}
////////-------------------------------------------------
ISR(TIMER1_COMPA_vect) {  // interrupt when timer 1 reaches OCR1A value
  digitalWrite(kontrola, !digitalRead(kontrola));
  if (StateVar == 0) { speedup = 0; }
  if (StateVar == 0) {
    cli();
    Motor_stop();
  }


  if (Amp > 1) {
    Amp = 1;
  }
  switch (StateVar) {
    case 0:
      PWM(phaseShiftA, phaseShiftB, phaseShiftC);
      break;
    case 1:
      phaseShiftA = 0;
      phaseShiftB = 1;
      phaseShiftC = 2;
      PWM(phaseShiftA, phaseShiftB, phaseShiftC);
      break;
    case 2:
      phaseShiftA = 0;
      phaseShiftB = 2;
      phaseShiftC = 1;
      PWM(phaseShiftA, phaseShiftB, phaseShiftC);
      break;
  }
}
void PWM(byte phaseShiftA, byte phaseShiftB, byte phaseShiftC) {
  // Refreshing the PWM registers
  PWM_Soft_A = ((pgm_read_byte_near(sine[phaseShiftA] + numR)) * Amp);  // Phase U update + amplitude
  PWM_Soft_B = ((pgm_read_byte_near(sine[phaseShiftB] + numR)) * Amp);  // Phase V update + amplitude
  PWM_Soft_C = ((pgm_read_byte_near(sine[phaseShiftC] + numR)) * Amp);  //  Phase W update + amplitude

  if (++numR > 38) {
    numR = 0;
  }


  FAZA_U = (PWM_Soft_A);
  FAZA_V = (PWM_Soft_B);
  FAZA_W = (PWM_Soft_C);

  OCR1A = freqscala;

  digitalWrite(kontrola, !digitalRead(kontrola));
}
ISR(TIMER3_COMPA_vect) {  // interrupt when timer 3 reaches OCR3A value
  if (start == 1) {
    OCR3A = 300 + (rampa_gore * 200);
    if (speedup < dfreq) {

      speedup = speedup + 0.25;
    }

    if (speedup > dfreq) {
      speedup = speedup - 0.25;
    }
  }
  if (start == 0) {
    OCR3A = 300 + (rampa_dole * 200);
    speedup = speedup - 0.25;
  }
}
void brzina() {
  freqregulatora = 1 / (1 / speedup / 39);
  freqscala = 16000000 / freqregulatora;
}



///////////////////////////////////////////////////////////////////////////////////////////////
void pisanje__citanje_INT_eeprom(int adress, int number) {
  EEPROM.write(adress, number >> 8);
  EEPROM.write(adress + 1, number & 0xFF);
}
int citanje_INT_from_eeprom(int adress) {
  return (EEPROM.read(adress) << 8) + EEPROM.read(adress + 1);
}
//////////////////////////////////////////////////////////////////////////////////////////////
void merenje_DCnapon() {
  while (napon_uzorak_broj < NAPON_UZORAKA) {
    naponsum += analogRead(A3);
    napon_uzorak_broj++;
    delay(1);
  }
  napon = ((float)naponsum / (float)napon_uzorak_broj * 5.0) / 1024.0;
  napon = (napon * 113.71);
  lcd.setCursor(12, 3);
  lcd.print("DC=");
  lcd.print(int(napon));
  lcd.print("V   ");
  naponsum = 0;
  napon_uzorak_broj = 0;
}

void prikaz() {
  lcd.setCursor(0, 0);
  lcd.print("Zadato:");
  lcd.print(int(dfreq));
  lcd.print("Hz ");
  lcd.setCursor(15, 0);
  lcd.print("SMER:");
  lcd.setCursor(15, 2);
  //lcd.print(Animacija);//(LCD_Buttons_ADC);
  lcd.print(" ");
  lcd.setCursor(0, 1);
  lcd.print("Ostvareno:");
  if (speedup >= 7) {
    lcd.print(round(speedup));
    lcd.print("Hz ");
  }
  if (speedup <= 7) {
    lcd.setCursor(0, 1);
    lcd.print("Ostvareno:");
    lcd.print("0");
    lcd.print("Hz ");
  }
  if (speedup >= 7) {
    lcd.setCursor(0, 2);
    lcd.print("U out:");
    lcd.print(int(Amp * 100));
    lcd.print("% ");
  }
  if (speedup <= 7) {
    lcd.setCursor(0, 2);
    lcd.print("U out:");
    lcd.print("0");
    lcd.print("% ");
  }
  lcd.setCursor(0, 4);
  lcd.print("I:");
  lcd.print(mA_Prikaz);
  lcd.print("mA   ");


  if (StateVar == 0) {
    lcd.setCursor(15, 1);
    lcd.print("STOP!");
  }
  if (StateVar == 1) {
    if (smer == true) {
      lcd.setCursor(15, 1);
      lcd.print("     ");
      lcd.setCursor(17, 1);
      lcd.print((char)0x00);
      lcd.print((char)0x00);
    }
    if (smer == false) {
      lcd.setCursor(15, 1);
      lcd.print("     ");
      lcd.setCursor(15, 1);
      lcd.print((char)0x00);
      lcd.print((char)0x00);
    }
  }
  if (StateVar == 2) {
    if (smer == true) {
      lcd.setCursor(15, 1);
      lcd.print("     ");
      lcd.setCursor(15, 1);
      lcd.print((char)0x01);
      lcd.print((char)0x01);
    }
    if (smer == false) {
      lcd.setCursor(15, 1);
      lcd.print("     ");
      lcd.setCursor(17, 1);
      lcd.print((char)0x01);
      lcd.print((char)0x01);
    }
  }
}

void citanje_tastera() {

  LCD_Buttons_ADC = analogRead(A1);                // read the buttons
  if (LCD_Buttons_ADC > 1000) LCD_Button = 0;      // button NONE
  else if (LCD_Buttons_ADC < 50) LCD_Button = 3;   // button RIGHT
  else if (LCD_Buttons_ADC < 195) LCD_Button = 1;  // button UP
  else if (LCD_Buttons_ADC < 380) LCD_Button = 2;  // button DOWN
  else if (LCD_Buttons_ADC < 555) LCD_Button = 4;  // button LEFT
  else if (LCD_Buttons_ADC < 790) LCD_Button = 5;  // button SELECT
  else LCD_Button = 0;
}
void loop() {

  citanje_tastera();


  freq = map(analogRead(FREQ_POT), POT_MIN, POT_MAX, FREQ_MIN, FREQ_MAX);
  Amp = speedup / freq_motora_nom;  //50;//50 motor frequency
  freqF = freq;
  dfreq = freqF;

  prikaz();


  if (Flag_tajmer_za_mer_napona == 26) {
    merenje_DCnapon();
    Flag_tajmer_za_mer_napona = 0;
  }
  Flag_tajmer_za_mer_napona++;

provera_napona1:
  if (napon < DC_MIN) {
    digitalWrite(enableIGBT, LOW);
    StateVar = 0;
    Revers = 0;
    TCCR5A = 0;
    TCCR5B = 0;
    speedup = 7;
    start = 0;
    digitalWrite(releynapR, LOW);
    digitalWrite(releynap0, HIGH);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DC voltage below ");
    lcd.setCursor(0, 1);
    lcd.print("limit!!!");
    lcd.setCursor(0, 2);
    lcd.print("Turn off breaker");
    lcd.setCursor(0, 3);
    lcd.print("and check supply.");
    delay(1000);
    goto provera_napona1;
  }
  if (napon > DC_MAX) {
    digitalWrite(enableIGBT, LOW);
    StateVar = 0;
    Revers = 0;
    TCCR5A = 0;
    TCCR5B = 0;
    speedup = 7;
    start = 0;
    digitalWrite(releynapR, LOW);
    digitalWrite(releynap0, HIGH);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DC voltage above ");
    lcd.setCursor(0, 1);
    lcd.print("limit!!!");
    lcd.setCursor(0, 2);
    lcd.print("Turn off breaker");
    lcd.setCursor(0, 3);
    lcd.print("and check supply.");
    delay(1000);
    goto provera_napona1;
  }
  mA = ACS.mA_AC();

  if (mA > I_MAX) {
    start = 0;

    strujna_zastita = true;
  } else {
    strujna_zastita = false;
  }
  if (strujna_zastita == true) {
    StateVar = 0;
    Revers = 0;
    TCCR5A = 0;
    TCCR5B = 0;
    speedup = 7;
    start = 0;
    delay(500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Overcurrent");
    lcd.setCursor(0, 1);
    lcd.print("protection activated");
    lcd.setCursor(0, 2);
    lcd.print("Turn off breaker,");
    lcd.setCursor(0, 3);
    lcd.print("check motor.");
    delay(500);
    lcd.clear();
  }

  mA_temp = mA_temp + mA;

  if (mA_temp1 > 20) {
    mA_Prikaz = mA_temp / mA_temp1;
    mA_temp = 0;
    mA_temp1 = 0;
  }
  mA_temp1++;
  if (Revers == 0) {
    if (LCD_Button == 5)  // Stop
    {
      start = 0;
    }
    if (LCD_Button == 5 && StateVar == 0 && Animacija == 20) {
      flag_podesavanje = true;
      podesavanje();
    }



    if (speedup < 7) {
      digitalWrite(enableIGBT, LOW);
      StateVar = 0;
      Revers = 0;
      TCCR5A = 0;
      TCCR5B = 0;
      speedup = 7;
      start = 0;
    }
    if (LCD_Button == 3 && StateVar == 0 && start == 0)  // Forward
    {
      digitalWrite(enableIGBT, HIGH);
      speedup = 7;
      StateVar = 1;
      Revers = 0;
      start = 1;
      TCCR1B = 0b00001001;
      TCCR5A = 0b10101010;
      TCCR5B = 0b00010010;
    }                                                    //sei();}
    if (LCD_Button == 4 && StateVar == 0 && start == 0)  // Backward
    {
      digitalWrite(enableIGBT, HIGH);
      speedup = 7;
      StateVar = 2;
      Revers = 0;
      start = 1;
      TCCR1B = 0b00001001;
      TCCR5A = 0b10101010;
      TCCR5B = 0b00010010;
    }  //sei();}
  }

  brzina();
  dfreq = speedup;




  if (Animacija > 20) {

    smer = !smer;
    Animacija = 0;
  }
  Animacija++;
}
void Motor_stop() {
  TCCR1A = 0;  // reset the value
  TCCR1B = 0;  // reset the value
  TCNT1 = 0;   // reset the value
  TCCR5A = 0;
  TCCR5B = 0;
  speedup = 7;
  start = 0;
  numR = 0;
  phaseShiftA = 0;
  phaseShiftB = 0;
  phaseShiftC = 0;
}
/////////////////////////////////////////////////////////////////////////////////
void podesavanje() {
  lcd.clear();
  while (flag_podesavanje == true) {
    lcd.setCursor(0, 0);
    lcd.print("Podesavanje:");
    if (izbor_ekrana == 6) {
      lcd.setCursor(0, 1);
      lcd.print("Freq Max  Out       ");
      lcd.setCursor(0, 2);
      lcd.print(FREQ_MAX);
      lcd.print("Hz     ");
    }
    if (izbor_ekrana == 7) {
      lcd.setCursor(0, 1);
      lcd.print("Struja motora nom.           ");
      lcd.setCursor(0, 2);
      I_MAX_eeprom = I_MAX;
      I_MAX_eeprom = I_MAX_eeprom / 1000;
      lcd.print(I_MAX_eeprom);
      lcd.print("A     ");
    }
    if (izbor_ekrana == 8) {
      lcd.setCursor(0, 1);
      lcd.print("Uzlazna rampa (sec)");
      lcd.setCursor(0, 2);
      lcd.print(rampa_gore);
      lcd.print("s     ");
    }
    if (izbor_ekrana == 9) {
      lcd.setCursor(0, 1);
      lcd.print("Silazna rampa (sec)");
      lcd.setCursor(0, 2);
      lcd.print(rampa_dole);
      lcd.print("s    ");
    }
    if (izbor_ekrana == 10) {
      lcd.setCursor(0, 1);
      lcd.print("DC prenapon      ");
      lcd.setCursor(0, 2);
      lcd.print(DC_MAX);
      lcd.print("V     ");
    }
    if (izbor_ekrana == 11) {
      lcd.setCursor(0, 1);
      lcd.print("DC podnapon     ");
      lcd.setCursor(0, 2);
      lcd.print(DC_MIN);
      lcd.print("V     ");
    }
    if (izbor_ekrana == 12) {
      lcd.setCursor(0, 1);
      lcd.print("Freq motora nomi.     ");
      lcd.setCursor(0, 2);
      lcd.print(freq_motora_nom);
      lcd.print("Hz      ");
    }
    if (Animacija > 20) {

      Animacija = 0;
    }
    Animacija++;

    if (LCD_Button == 5 && Animacija == 20) {
      EEPROM.write(0, 1);
      EEPROM.write(1, FREQ_MAX);
      EEPROM.write(2, rampa_gore);
      EEPROM.write(3, rampa_dole);
      pisanje__citanje_INT_eeprom(4, DC_MAX);
      pisanje__citanje_INT_eeprom(6, DC_MIN);
      EEPROM.write(8, freq_motora_nom);
      pisanje__citanje_INT_eeprom(9, I_MAX);

      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print(" Saved to EEPROM!!");
      delay(2000);
      lcd.clear();
      izbor_ekrana = 6;
      flag_podesavanje = false;
    }
    delay(150);
    citanje_tastera();

    switch (LCD_Button) {
      case 1:
        if (izbor_ekrana == 6) {
          FREQ_MAX++;
          if (FREQ_MAX > 70) {
            FREQ_MAX = 70;
          }
        }
        if (izbor_ekrana == 7) {
          I_MAX = I_MAX + 100;
          if (I_MAX > 20000) {
            I_MAX = 20000;
          }
        }
        if (izbor_ekrana == 8) {
          rampa_gore++;
          if (rampa_gore > 20) {
            rampa_gore = 20;
          }
        }
        if (izbor_ekrana == 9) {
          rampa_dole++;
          if (rampa_dole > 20) {
            rampa_dole = 20;
          }
        }
        if (izbor_ekrana == 10) {
          DC_MAX++;
          if (DC_MAX > 400) {
            DC_MAX = 400;
          }
        }
        if (izbor_ekrana == 11) {
          DC_MIN++;
          if (DC_MIN > 320) {
            DC_MIN = 320;
          }
        }
        if (izbor_ekrana == 12) {
          freq_motora_nom = 60;
        }
        delay(10);
        break;
      case 2:
        if (izbor_ekrana == 6) {
          --FREQ_MAX;
          if (FREQ_MAX < 30) {
            FREQ_MAX = 30;
          }
        }
        if (izbor_ekrana == 7) {
          I_MAX = I_MAX - 100;
          if (I_MAX < 100) {
            I_MAX = 100;
          }
        }
        if (izbor_ekrana == 8) {
          --rampa_gore;
          if (rampa_gore < 1) {
            rampa_gore = 1;
          }
        }
        if (izbor_ekrana == 9) {
          --rampa_dole;
          if (rampa_dole < 1) {
            rampa_dole = 1;
          }
        }
        if (izbor_ekrana == 10) {
          --DC_MAX;
          if (DC_MAX < 250) {
            DC_MAX = 250;
          }
        }
        if (izbor_ekrana == 11) {
          --DC_MIN;
          if (DC_MIN > 250) {
            DC_MIN = 250;
          }
        }
        if (izbor_ekrana == 12) {
          freq_motora_nom = 50;
        }
        delay(10);
        break;
      case 4:
        izbor_ekrana++;
        if (izbor_ekrana > 12) {
          izbor_ekrana = 6;
          delay(10);
          break;

          case 3:
            --izbor_ekrana;
            if (izbor_ekrana < 6) {
              izbor_ekrana = 12;
              delay(10);
              break;
            }
        }
        lcd.clear();
    }
  }
}
