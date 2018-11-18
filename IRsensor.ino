//baseline of this code was from http://dorkbotpdx.org/blog/scott_d/inexpensive_ir_based_temperature_sensor_for_microprocessors
//Modified by David_G and Bartimaeus

#define IR_CLK 2 //this is interrupt driven, pin 2 or 3
#define IR_DATA 12 //data pin

volatile int nbits = 0;
volatile byte hexbyte = 0;

volatile unsigned char message[4];
volatile int nbytes = 0;
volatile int message_waiting = 0;

unsigned long last_time = 0;

float temp= 99.00;
float ambient;
boolean light = false;

int irSetup = 1;

float latestReading = 0.0;


void setupTempSensor() {
  pinMode(IR_CLK, INPUT);
  pinMode(IR_DATA, INPUT);
  pinMode(13, OUTPUT); 
  attachInterrupt(0, readBit, FALLING); //0 -> pin2, 1 -> pin3
}  

/*
//attempt to filter out beginning false readings. Works fairly well but still has a bug or two
int isStartup(){
  if(irSetup){
    if((int)temp != 99){
      irSetup = 0;  
    }
  }
  return irSetup;
}
*/

void temp_output() {
   //updateTempSensor();
   float tmp = getFreshTemp(); //gets the temperature reading
   //float tmp = getFeshAmb(); //gets the ambient temperature  
   //if(!(isStartup())){    //filters out the bad data at the beginning
     float fahrenheit = (tmp*1.8) + 32;
     Serial.print(tmp);
     Serial.print(" C");
     Serial.print('\t');
     Serial.print(fahrenheit);
     Serial.println(" F");
   //}
 
   /*
      //debug information for cmdline 
       Serial.print("DD: "); 
       Serial.print(message[0]);
       Serial.print(message[1]);         
       Serial.print(message[2]); 
       Serial.print(" H:");
       Serial.print(hexbyte);
       Serial.println(';');
   */
 
}

void updateTempSensor() {
  if (message_waiting == 1) {
    last_time = millis();
    if (message[0] == 0x4c) { //from zytemp
      int t = message[1]<<8 | message[2];
      temp = t/16.0 -273.15;
    } 
    else if (message[0] == 0x66) {
      int t = message[1]<<8 | message[2];
      ambient = t/16.0 -273.15;
    }
    message_waiting = 0;
  }

  //update evry second
  if (millis() - last_time > 1000) {
    nbits = 0;
    nbytes = 0;
    hexbyte = 0;
    message_waiting = 0;
    last_time = millis();
  }
  
}

// Interupt routine for handling IR sensor clock trailing edge
void readBit() {
  int val = digitalRead(IR_DATA);  //read the data from the IR module
  if(!light) digitalWrite(13, HIGH);   // set the LED on
  else digitalWrite(13, LOW);
  light = !light;  //change light state when a bit was read in
  nbits++;
  int bit = (val == HIGH) ? 1 : 0; 
  hexbyte = (hexbyte << 1) | bit; //compound bytes so that we creat a 8-bit key
  if (nbits == 8) { 
    if (hexbyte == 0xd) {
      nbytes = 0;
      message_waiting = 1;
    } 
    else if (message_waiting == 0) {
      if (nbytes < 4) {
        message[nbytes] = hexbyte;
      }
      nbytes++;
    }
    hexbyte = 0;
    nbits = 0;
  }
}

float getFreshTemp() { 
  return temp;
}

float getFeshAmb() {
   return ambient; 
}

void setup() {
   Serial.begin(9600);
   setupTempSensor(); //start temperature
}

void loop() {
  updateTempSensor();  //allwas update the temp sensor each cycle
  temp_output();
  delay(100);
    
}
