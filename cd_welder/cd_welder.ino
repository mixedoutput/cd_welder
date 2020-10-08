/*
 * Experiment board testcode
 * Experiment board ver 1.2.1
 * 
 * Pin assignment:
 * D0/D1 (RX/TX) - BT
 * D2 - key2
 * D3 - Piezo
 * D4 - key4
 * D5 - Assign LED/P-MOSFET
 * D6 - White LED/NPN transistor
 * D7 - Yellow LED
 * D8 - Blue LED
 * D9 - Green LED/N-MOSFET
 * D10 - Red LED
 * D11 - IR-LED
 * D12 - key12/Rot.enc Btn
 * D13 - key13
 * A0 - keyA0
 * A1 - keyA1
 * A2 - Rot.enc B
 * A3 - Rot.enc A
 * A4 - SDA (I2C)
 * A5 - SCL (I2C)
 * A6 - NC
 * A7 - Pot
 */

//LED definition
#define NR_OF_LEDS  7
#define LED_X       5   //Custom led
#define LED_W       6  //white
#define LED_Y       7   //yellow
#define LED_B       8   //blue
#define LED_G       9   //green
#define LED_R       10   //red
#define LED_IR      11  //Infra-Red

//Button definition
#define NR_OF_KEYS  6
#define KEY_A0      A0
#define KEY_A1      A1
#define KEY_2       2
#define KEY_4       4
#define KEY_12      12
#define KEY_13      13

//Potentiometers
#define POT         A7

//Rotery encoder
#define ENCA        A3
#define ENCB        A2
#define ENC_BTN     12

#include <RotaryEncoder.h>

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(A2, A3);

//Below code and associated rotary code is taken from AcceleratedRotator example sketch
//For more info and explanation see sketch

// Define some constants.
// at 500ms, there should be no acceleration.
constexpr const unsigned long kAccelerationLongCutoffMillis = 500;
// at 4ms, we want to have maximum acceleration
constexpr const unsigned long kAccelerationShortCutffMillis = 4;
// linear acceleration: incline
constexpr static const float m = -0.16;
// linear acceleration: y offset
constexpr static const float c = 84.03;

//LCD
#define LCD_SDA A4
#define LCD_SCL A5
#define MSG1    "Hi Robin!" //Exp-board   v0,1
#define MSG2    "I work!"      //Hello, world!

//Amplifiers
#define NR_OF_AMPS  3
#define NPN         6   //NPN tranistor
#define NMOS        9   //N-channel mosfet
#define PMOS        5   //P-channel mosfet

//BUZZER
#define BUZZER      3   //Piezzo
#define NR_OF_TONES 7
#define TONE_A      3520
#define TONE_B      3951
#define TONE_C      4186
#define TONE_D      4699
#define TONE_E      5274
#define TONE_F      5588
#define TONE_G      6272


#include <LiquidCrystal_I2C.h>

byte key_array[] = {KEY_13, KEY_A0, KEY_A1, KEY_2, KEY_4, KEY_12}; 
byte led_array[] = {LED_X, LED_W, LED_Y, LED_B, LED_G, LED_R, LED_IR};
byte pot_array[] = {POT, POT};
byte amp_array[] = {NPN, NMOS, PMOS};
unsigned int tone_array[] = {TONE_A, TONE_B, TONE_C, TONE_D, TONE_E, TONE_F, TONE_G};

int encoder_pos = 0;
byte encoder_A = LOW;
byte encoder_A_last = LOW;

byte onoff = 0;

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

float read_voltage();
void write_lcd(String);

void self_test(){
  //LCD ON
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(MSG1);
  lcd.setCursor(0,3);             // second row starts at 3 for 1602
  lcd.print(MSG2);
  lcd.setCursor(0,2);             // 2004 LCD
  lcd.print("2004 LCD");
  Serial.println("Experiment board v1.2");
  
  /*
  //Test buzzer
  for(int i=0; i<3; i++){
    digitalWrite(BUZZER,HIGH);
    delay(100);
    digitalWrite(BUZZER,LOW);
    delay(50);
  }
  */
  //test leds
  for(int i=0; i<NR_OF_LEDS; i++){
    digitalWrite(led_array[i], HIGH);
    delay(150);
    digitalWrite(led_array[i], LOW);
  }

  Serial.println("Ready!");
}

void read_encoder(){
    static int pos = 0;
  static RotaryEncoder::Direction lastMovementDirection = RotaryEncoder::Direction::NOROTATION;
  encoder.tick();

  int newPos = encoder.getPosition();
  if (pos != newPos) {

    // compute linear acceleration
    RotaryEncoder::Direction currentDirection = encoder.getDirection();
    if (currentDirection == lastMovementDirection && 
        currentDirection != RotaryEncoder::Direction::NOROTATION &&
        lastMovementDirection != RotaryEncoder::Direction::NOROTATION) {
      // ... but only of the direction of rotation matched and there
      // actually was a previous rotation.
      unsigned long deltat = encoder.getMillisBetweenRotations();

      if (deltat < kAccelerationLongCutoffMillis) {
        if (deltat < kAccelerationShortCutffMillis) {
          // limit to maximum acceleration
          deltat = kAccelerationShortCutffMillis;
        }

        float ticksActual_float = m * deltat + c;
        // Round by adding 1
        // Then again remove 1 to determine the actual delta to the encoder
        // value, as the encoder already ticked by 1 tick in the correct
        // direction. Thus, just cast to an integer type.
        long deltaTicks = (long)ticksActual_float;

        // Adjust sign: Needs to be inverted for counterclockwise operation
        if (currentDirection == RotaryEncoder::Direction::COUNTERCLOCKWISE) {
          deltaTicks = -(deltaTicks);
        }

        newPos = newPos + deltaTicks;
        encoder.setPosition(newPos);
      }
    }

    Serial.print(newPos);
    Serial.println();
    pos = newPos;
  } // if
}

void setup() {
  Serial.begin(9600);
  Serial.println("setup...");

  for(int i=0; i<NR_OF_KEYS; i++) pinMode(key_array[i], INPUT);
  for(int i=0; i<NR_OF_LEDS; i++) pinMode(led_array[i], OUTPUT);
  for(int i=0; i<NR_OF_AMPS; i++) pinMode(amp_array[i], OUTPUT);

  pinMode(BUZZER, OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(POT, INPUT);
  pinMode(A6, INPUT);

  lcd.init();                      // initialize the lcd

  self_test();
}

void loop() {
  //read_encoder();

  
  Serial.println(read_voltage());
  //blink_led();
  //delay(3000);

  if(digitalRead(KEY_13)){
    //digitalWrite(NPN, HIGH);
    blink_led();
    delay(1000);
  }
  //else digitalWrite(NPN, LOW);
  
  //delay(10);

}

float read_voltage(){
  float voltage=0.0;
  int voltage_read=0;
  voltage_read = analogRead(A6);
  return voltage_read * (16.14 / 1023.0);
}

void blink_led(){
  String str;
  if(onoff) str = "ON";
  else str = "OFF";
  write_lcd(str);
  digitalWrite(PMOS, onoff);
  digitalWrite(NPN, onoff);
  if(onoff) onoff = 0;
  else onoff = 1;
}

void write_lcd(String str){
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print(str);
}

