
const int beaconPin = 13;
int beaconTimer[4] = {3000, 150, 150, 150};//Timing sequence each OFF-ON-OFF-ON
int beaconCurrentTimer = 0; //Current timer interval left
unsigned int beaconState = 0; //Beacon State (0 - 3)

void beaconInit() {
  beaconCurrentTimer = beaconTimer[0];
  beaconState = 0;
  digitalWrite(beaconPin,LOW);
}

ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
  switch(beaconState)
  {
    case 0:
      if(beaconCurrentTimer-- < 1)
      {
        beaconState++;  
        beaconCurrentTimer = beaconTimer[1];
        digitalWrite(beaconPin,HIGH);
      }
      break;
    case 1:
      if(beaconCurrentTimer-- < 1)
      {
        beaconState++;  
        beaconCurrentTimer = beaconTimer[2];
        digitalWrite(beaconPin,LOW);
      }
      break;
    case 2:
      if(beaconCurrentTimer-- < 1)
      {
        beaconState++;  
        beaconCurrentTimer = beaconTimer[3];
        digitalWrite(beaconPin,HIGH);
      }
      break;
    case 3:
      if(beaconCurrentTimer-- < 1)
      {
        beaconState=0;  
        beaconCurrentTimer = beaconTimer[0];
        digitalWrite(beaconPin,LOW);
      }
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  //set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  
  sei();//allow interrupts
  beaconInit();
}



void loop() {
  // put your main code here, to run repeatedly:
  
}
