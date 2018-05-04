
/*
 * led_beacon
 * 
 * Airplane like beacon and strobe light effect for miniature airplane model.
 * Based on Arduino Nano board. 
 * Fuctions are
 * - Sequencial lighting for Beacon and Strobe light
 * - PWM dimming light for Landing, Navigation, Interior, LOGO light
 * - Touch switch controlled
 * 
 * created 19 Mar 2018
 * by Kyuhyong You
 * 
 * This code is available in github.
 * 
 * https://github.com/kyuhyong/arduino_beacon
 * 
 * PWM pins:
- Pins 5 and 6: controlled by Timer 0
- Pins 9 and 10: controlled by timer 1
- Pins 11 and 3: controlled by timer 2
 */
const int buttonPin = 2;
int buttonState = 0;
int buttonCnt = 0;
int interruptState = FALLING;
int buttonDetect = 0;
int flg_buttonShort = 0;
int flg_buttonLong = 0;

const int motorPWM_Pin = 9;
int motorTimer = 0;
int motorSet = 0;
#define SET_MOTOR_OUT_PWM   30

const int landingPWM_Pin = 10;
unsigned int landingSet = 0;
#define LANDING_LIGHT_PWM   60

const int navPWM_Pin = 11;
unsigned int navSet = 0;
#define NAV_LIGHT_PWM  60

const int interriorPWM_Pin = 3;
unsigned int interriorSet = 0;
#define INTERRIOR_LIGHT_PWM  60

const int beaconPin = 8;
int beaconTimer[4] = {2000, 100, 2000, 100};//Timing sequence each OFF-ON-OFF-ON
int beaconCurrentTimer = 0; //Current timer interval left
unsigned int beaconState = 0; //Beacon State (0 - 3)
unsigned short beaconSet = 0;

const int strobePin = 7;
int strobeTimer[4] = {2000, 60, 100, 60};//Timing sequence each OFF-ON-OFF-ON
int strobeCurrentTimer = 0; //Current timer interval left
unsigned int strobeState = 0; //Beacon State (0 - 3)
unsigned short strobeSet = 0;

#define BATTERY_MONITOR_INTERVAL 2500     //Monitoring battery interval set to 1000ms
const int batMonTriggerPin = 4;
const int batMonAnalogPin = 1;
unsigned int batMonTimerCnt = BATTERY_MONITOR_INTERVAL;
unsigned int batMonReadVal = 0;
int flg_batMonitor = 0;

#define PROCESS_INDC_TIMER  500
unsigned short process_state = 0;
unsigned short process_Set = 0;
int process_indCnt = 0;
int flg_process_ind = 0;
int process_indState = 0;

void BeaconReset() {
  beaconCurrentTimer = beaconTimer[0];
  beaconState = 0;
  beaconSet = 0;
  Serial.println("BCN_RESET");
  digitalWrite(beaconPin,LOW);
}
void BeaconSet() {
  beaconCurrentTimer = beaconTimer[0];
  beaconState = 1;
  beaconSet = 1;
  Serial.println("BCN_SET");
  digitalWrite(beaconPin,LOW);
}
void StrobeReset() {
  strobeCurrentTimer = strobeTimer[0];
  strobeState = 0;
  strobeSet = 0;
  Serial.println("STRB_RESET");
  digitalWrite(strobePin,LOW);
}
void StrobeSet() {
  strobeCurrentTimer = strobeTimer[0];
  strobeState = 1;
  strobeSet = 1;
  Serial.println("STRB_SET");
  digitalWrite(strobePin,LOW);
}
void MotorSet() {
  motorSet = 1;
  Serial.println("MOTOR_SET");
  analogWrite(motorPWM_Pin, SET_MOTOR_OUT_PWM);
}
void MotorReset() {
  motorSet = 0;
  Serial.println("MOTOR_RESET");
  analogWrite(motorPWM_Pin, 0);
}
void LandingSet() {
  landingSet = 1;
  Serial.println("LAND_SET");
  analogWrite(landingPWM_Pin, LANDING_LIGHT_PWM);
}
void LandingReset() {
  landingSet = 0;
  Serial.println("LAND_RESET");
  analogWrite(landingPWM_Pin, 0);
}
void NavSet() {
  navSet = 1;
  Serial.println("NAV_SET");
  analogWrite(navPWM_Pin, NAV_LIGHT_PWM);
}
void NavReset() {
  navSet = 0;
  Serial.println("NAV_RESET");
  analogWrite(navPWM_Pin, 0);
}
void InterriorSet() {
  interriorSet = 1;
  Serial.println("INT_SET");
  analogWrite(interriorPWM_Pin, INTERRIOR_LIGHT_PWM);
}
void InterriorReset() {
  interriorSet = 0;
  Serial.println("INT_RESET");
  analogWrite(interriorPWM_Pin, 0);
}
ISR(TIMER0_COMPA_vect){//timer0 interrupt 1kHz
  if(buttonState == 1)
  {
    buttonCnt++;
  }
  if(batMonTimerCnt-- < 1) {
    flg_batMonitor = 1;
    batMonTimerCnt = BATTERY_MONITOR_INTERVAL;
  }
  if(process_Set == 1) {
    if(flg_process_ind!=1){
      if(process_indCnt-- < 1) {
        flg_process_ind = 1;
      }
    }
  }
  switch(beaconState)
  {
    case 0:   //Standby state
      if(digitalRead(beaconPin) == HIGH) {
        digitalWrite(beaconPin,LOW);
      }
      break;
    case 1:
      if(beaconCurrentTimer-- < 1)
      {
        beaconState++;  
        beaconCurrentTimer = beaconTimer[1];
        digitalWrite(beaconPin,HIGH);
      }
      break;
    case 2:
      if(beaconCurrentTimer-- < 1)
      {
        beaconState++;  
        beaconCurrentTimer = beaconTimer[2];
        digitalWrite(beaconPin,LOW);
      }
      break;
    case 3:
      if(beaconCurrentTimer-- < 1)
      {
        beaconState++;  
        beaconCurrentTimer = beaconTimer[3];
        digitalWrite(beaconPin,HIGH);
      }
      break;
    case 4:
      if(beaconCurrentTimer-- < 1)
      {
        beaconState=1;  
        beaconCurrentTimer = beaconTimer[0];
        digitalWrite(beaconPin,LOW);
      }
      break;
  }
  switch(strobeState)
  {
    case 0:   //Standby state
      if(digitalRead(strobePin) == HIGH) {
        digitalWrite(strobePin,LOW);
      }
      break;
    case 1:
      if(strobeCurrentTimer-- < 1)
      {
        strobeState++;  
        strobeCurrentTimer = strobeTimer[1];
        digitalWrite(strobePin,HIGH);
      }
      break;
    case 2:
      if(strobeCurrentTimer-- < 1)
      {
        strobeState++;  
        strobeCurrentTimer = strobeTimer[2];
        digitalWrite(strobePin,LOW);
      }
      break;
    case 3:
      if(strobeCurrentTimer-- < 1)
      {
        strobeState++;  
        strobeCurrentTimer = strobeTimer[3];
        digitalWrite(strobePin,HIGH);
      }
      break;
    case 4:
      if(strobeCurrentTimer-- < 1)
      {
        strobeState=1;  
        strobeCurrentTimer = strobeTimer[0];
        digitalWrite(strobePin,LOW);
      }
      break;
  }
}

ISR(INT0_vect) {
  if(interruptState == FALLING) {
    //digitalWrite(13, HIGH);
    buttonState = 1;
    buttonCnt = 0;
    interruptState = RISING;
    EICRA |= (1<<ISC01)|(1<<ISC00);   //Rising edge interrupt
  }else {
    //digitalWrite(13, LOW);
    buttonState = 0;
    buttonDetect = 1;
    interruptState = FALLING;
    EICRA &= ~(1<<ISC00); //Falling edge detect
  }
  //digitalWrite(13, !digitalRead(13));
}
void setup() {
  // put your setup code here, to run once:
  cli();
  EICRA |= (1<<ISC01);    //Falling edge interrupt
  EIMSK |= (1<<INT0);     //Enable INT0 interrupt

  pinMode(motorPWM_Pin, OUTPUT);
  pinMode(landingPWM_Pin, OUTPUT);
  pinMode(navPWM_Pin, OUTPUT);
  pinMode(interriorPWM_Pin, OUTPUT);
  pinMode(beaconPin, OUTPUT);
  pinMode(strobePin, OUTPUT);
  pinMode(batMonTriggerPin, OUTPUT);
  pinMode(batMonAnalogPin, INPUT);
  pinMode(13, OUTPUT);
  
  //set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 1khz increments
  OCR0A = 249;// = (16*10^6) / (1000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  
  sei();//allow interrupts
  BeaconReset();
  StrobeReset();
  NavReset();
  InterriorReset();
  Serial.begin(115200);  //initial the Serial
  Serial.println("Hello world");
}
void systemReset(){
    flg_buttonLong = 0;
    process_state = 0;
    analogWrite(motorPWM_Pin, 0);
    BeaconReset();
    StrobeReset();
    MotorReset();
    NavReset();
    InterriorReset();
    analogWrite(landingPWM_Pin, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(buttonDetect == 1)
  {
    Serial.print("BTN:");
    Serial.println(buttonCnt);
    if(buttonCnt > 200) {
      flg_buttonLong = 1;
    }
    else {
      flg_buttonShort =1;
    }
    buttonDetect = 0;
  }
  if(flg_buttonShort == 1)
  {
    flg_buttonShort = 0;
    if(process_state++ > 4) process_state = 0;  //Total number of Process is 5
    process_indCnt = PROCESS_INDC_TIMER;
    process_Set = 1;
    process_indState = process_state;
    flg_process_ind = 0;
  }
  /*
   * Process Indicator -- Blink LED per current process state
   */
  if(process_Set == 1)
  {
    if(flg_process_ind == 1) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      if(process_indState--<1) {
        process_Set = 0;
      }
      process_indCnt = PROCESS_INDC_TIMER;
      flg_process_ind = 0;
    }
  }
  switch(process_state)
  {
    case 0:
      if(flg_buttonLong == 1)
      {
        Serial.println(beaconSet);
        if(beaconSet != 1){
          BeaconSet();
        }
        else {
          BeaconReset();
        }
        flg_buttonLong = 0;
      }
      break;
    case 1:
      if(flg_buttonLong == 1)
      {
        if(strobeSet != 1) {
          StrobeSet();
        }
        else {
          StrobeReset();
        }
        flg_buttonLong = 0;
      }
      break;
    case 2:
      if(flg_buttonLong == 1)
      {
        if(navSet != 1) {
          NavSet();
        }
        else {
          NavReset();
        }
        flg_buttonLong = 0;
      }
      break;
    case 3:
      if(flg_buttonLong == 1)
      {
        if(motorSet != 1) {
          MotorSet();
        }
        else {
          MotorReset();
        }
        flg_buttonLong = 0;
      }
      break;
    case 4:
      if(flg_buttonLong == 1)
      {
        if(landingSet != 1) {
          LandingSet();
        }
        else {
          LandingReset();
        }
        flg_buttonLong = 0;
      }
      break;
    case 5:
      if(flg_buttonLong == 1)
      {
        if(interriorSet != 1) {
          InterriorSet();
        }
        else {
          InterriorReset();
        }
        flg_buttonLong = 0;
      }
      break;
  }
  if(flg_batMonitor == 1)
  {
    digitalWrite(batMonTriggerPin, HIGH);
    delay(10);
    digitalWrite(batMonTriggerPin, LOW);
    delay(1);
    batMonReadVal = analogRead(batMonAnalogPin);
    Serial.print("Battery:");
    Serial.println(batMonReadVal*0.00914);//Calibrated: 457 for 4.13V vcc
    flg_batMonitor = 0;
  }
}
