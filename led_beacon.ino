
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
 */
const int motor1Pin = 3;
int motorTimer = 0;
int motorSet = 0;

const int beaconPin = 13;
int beaconTimer[4] = {2000, 100, 2000, 100};//Timing sequence each OFF-ON-OFF-ON
int beaconCurrentTimer = 0; //Current timer interval left
unsigned int beaconState = 0; //Beacon State (0 - 3)

const int strobePin = 14;
int strobeTimer[4] = {2000, 60, 100, 60};//Timing sequence each OFF-ON-OFF-ON
int strobeCurrentTimer = 0; //Current timer interval left
unsigned int strobeState = 0; //Beacon State (0 - 3)

void beaconReset() {
  beaconCurrentTimer = beaconTimer[0];
  beaconState = 0;
  digitalWrite(beaconPin,LOW);
}
void beaconSet() {
  beaconCurrentTimer = beaconTimer[0];
  beaconState = 1;
  digitalWrite(beaconPin,LOW);
}
void strobeReset() {
  strobeCurrentTimer = strobeTimer[0];
  strobeState = 0;
  digitalWrite(strobePin,LOW);
}
void strobeSet() {
  strobeCurrentTimer = strobeTimer[0];
  strobeState = 1;
  digitalWrite(strobePin,LOW);
}

ISR(TIMER0_COMPA_vect){//timer0 interrupt 1kHz
  if(motorTimer++>1000)
  {
    motorTimer = 0;
    if(motorSet == 1) motorSet = 0;
    else motorSet = 1;
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

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1Pin, OUTPUT);
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
  beaconSet();
}



void loop() {
  // put your main code here, to run repeatedly:
  if(motorSet == 1)
  {
    analogWrite(motor1Pin, 20);
  }
  else analogWrite(motor1Pin, 0);
}
