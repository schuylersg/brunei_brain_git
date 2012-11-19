//Testing GIT - this should only be in USB4000_settings branch
#include <SPI.h>
#include <SD.h>
#include <RTC_DS3234.h>
#include <TLC_5916.h>
#include <avr/sleep.h>

//Constants
const unsigned int MAX_INTEGRATION_TIME = 10000;
const unsigned int MIN_INTEGRATION_TIME = 10;
const unsigned int BOXCAR_WIDTH = 5;   //Values greater than 3 slow down transfer speeds
const unsigned int PIXEL_TRANSFER_SPACING = 5;

const byte ACK = 6;
const byte NAK = 21;

//PIN ASSIGNMENTS
const byte SpecRx = 0;
const byte SpecTx = 1;
const byte batteryChargerStatus = 2;
const byte rtcAlarm = 3;
const byte ledLatchEnable = 4;
const byte ledOutputEnable = 5;
const byte selfReset = 6;
const byte csSD = 8;
const byte csRTC = 9;

const byte fiveVoltEnable = A0;
const byte twelveVoltEnable = A1;

const byte redLED = 7;
const byte greenLED = A2;
const byte blueLED = A3;
const byte batteryVoltage48 = A4;
const byte batteryVoltage37 = A5;


//GLOBAL VARIABLES
int readingIteration = 0;
boolean justWokeUp = true;
boolean receivingData = false;

//index of LED_NAMES + 1 is the bit address of that LED on the TLC_5916
char* const LED_NAMES [] ={"LED365", "LED430", "LED405", "LED390", "LED370", "Abs"};  
uint8_t const LED_CURRENT_GAIN [] = {CG_20, CG_30, CG_30, CG_100, CG_30, CG_60, CG_70, CG_80};
uint16_t ledIntegrationTime [] = {5000, 5000, 3000, 2000, 1000, 100};  //array for initial integration times for each LED
                                                                          //the last one is for absorbance source

RTCDateTime currentTime;  //global variable to store the current date and time
RTCDateTime alarmTime;    //global variable to store the current alarm time

char dateStr[ ] = "00/00/00 00:00:00";          //a character array for printing date/time
char dataFileName[ ] = "files/MM_DD_YY.txt";	//a character array for the file name
File dataFile;
//File logFile;

char dataBuffer[8];
int incomingBytes[6];
char data = 0;
int openTries = 1;    

void setup(){
  //check if 48volt connected
  
  Serial.begin(9600);
  
  //Initialize pins
  pinMode(fiveVoltEnable, OUTPUT);
  
  pinMode(selfReset, OUTPUT);
  digitalWrite(selfReset, LOW);
  
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  
  Blink(redLED, 300, 1);
  Blink(greenLED, 300, 1);
  Blink(blueLED, 300, 1);
  
  //Initialize chip select control pinspins
  pinMode(10, OUTPUT);    //Arduino defined chip select
  
  pinMode(csSD, OUTPUT);
  digitalWrite(csSD, HIGH);
  
  pinMode(csRTC, OUTPUT);
  digitalWrite(csRTC, HIGH);
  
  //turn on the 5V DC-DC converter
  digitalWrite(fiveVoltEnable, HIGH);
    
  readingIteration = 0;  
    
  SPI.begin();  //start SPI communication

  //see if the SD card is present and can be initialized:

  if (!SD.begin(csSD)) {
    Blink(redLED, 300, 3);
    digitalWrite(selfReset, HIGH);
  }else{
    Blink(greenLED, 300, 1);    //1st green blink
  }
  
  RTC.begin(csRTC);                    //initialize RTC with chip select pin
  RTC.clearAlarmFlags();               //clear the alarm flags because we don't know what state the RTC was in
  currentTime = RTC.getRTCDateTime();  //need to do this initial read - otherwise strange bug occurs
                                       //where SD.open() fails the first time.
  dateTime2String(currentTime);        //convert date and time to human readable string
                                       //which is stored in the variable dateStr
    
  //set the file name to the current date
  dataFileName [6] = currentTime.months/10 + '0';
  dataFileName [7] = currentTime.months%10 + '0';
  dataFileName [9] = currentTime.days/10 + '0';
  dataFileName [10] = currentTime.days%10 + '0';  
  dataFileName [12] = currentTime.year/10 + '0';
  dataFileName [13] = currentTime.year%10 + '0';    
    
  dataFile = SD.open(dataFileName, FILE_WRITE);  
  openTries = 1;

  
  while(!dataFile && openTries < 3){                //try to open the datafile 3 times
    dataFile = SD.open(dataFileName, FILE_WRITE);
    openTries++;
  }
  
  if(!dataFile){                        //If unable to open the file
    Blink(redLED, 300, 3);
    digitalWrite(selfReset, HIGH);      //try doing a restart
  }
  
  dataFile.print("Measurement at: ");
  dataFile.println(dateStr);
  dataFile.close();
  
  //logFile = SD.open("log.txt", FILE_WRITE);
  
  pinMode(rtcAlarm, INPUT);            //Interrupt pin for RTC alarm
  
  //Start the TLLC5916 LED driver with pins 4 and 5 controlling the SPI communications
  tlc5916.begin(ledOutputEnable,ledLatchEnable);
  
  //Wait for Spec
  Blink(greenLED, 500, 6);
  clearSerial();
  
  //Set boxcar average
  Serial.write('B');
  Serial.write(highByte(BOXCAR_WIDTH));
  Serial.write(lowByte(BOXCAR_WIDTH));
  delay(10);
  clearSerial();

  //set Pixel Mode
  Serial.write('P');
  Serial.write(highByte(0));
  Serial.write(lowByte(1));
  Serial.write(highByte(PIXEL_TRANSFER_SPACING));
  Serial.write(lowByte(PIXEL_TRANSFER_SPACING));
  delay(10);
  clearSerial();
  
  Serial.write('a');                    //change the spectrometer to ASCII mode
  Serial.write('A');
  Serial.flush();
  delay(10);
  clearSerial(); 
}

void loop(){

  Serial.write('b');                    //change the spectrometer to ASCII mode
  Serial.write('B');
  delay(10);
  clearSerial(); 
  
  //Set the spectrometer integration time
  Serial.print("I");
  Serial.write(highByte(ledIntegrationTime[readingIteration]));
  Serial.write(lowByte(ledIntegrationTime[readingIteration]));
  delay(10);
  clearSerial();
  
  Serial.write('a');                    //change the spectrometer to ASCII mode
  Serial.write('A');
  delay(10);
  clearSerial(); 

  //wait until the spectrometer is ready
  clearSerial();
  
  if(!USB4000Ready()){
    Blink(redLED, 1000, 1);  
    goto endloop;
  }
  
  tlc5916.disableOutput();
  tlc5916.ezSetCurrentConfigurationCode(LED_CURRENT_GAIN[readingIteration]);
  tlc5916.ezSetPinsOnOff(1 << readingIteration);
  tlc5916.enableOutput();
  delay(10);
  
    
  //Start the scan
  digitalWrite(blueLED, HIGH);
  Serial.write('S');
  Serial.flush();
  delay(10);
  //The first value is 'S' echo
  //The second value is etx or stx
  Serial.readBytes(dataBuffer, 2);

  if(dataBuffer[0] != 'S' || dataBuffer[1] == 3){  //error with Spec
    Blink(redLED, 1000, 1);
    receivingData = false;
  }else{
    receivingData = true;  
    dataFile = SD.open(dataFileName, FILE_WRITE);  
    openTries = 1;    

    while(!dataFile && openTries < 3){                //try to open the datafile 3 times
      dataFile = SD.open(dataFileName, FILE_WRITE);
      openTries++;
    }  
    if(!dataFile){                        //If unable to open the file
      Blink(redLED, 300, 3);
      digitalWrite(selfReset, HIGH);      //try doing a restart
    }

  }
  
  //wait here until spectrum begins
  while(Serial.available() == 0){
  }
  
  digitalWrite(blueLED, LOW);
  tlc5916.disableOutput();  //turn off LED once scan has been captured
  while(receivingData){
   while(Serial.available() > 0){
     digitalWrite(greenLED, HIGH);
     data = Serial.read();
     if(data == '\n'){
      receivingData = false;
     }else{ 
      dataFile.write(data);
     }
   }
   digitalWrite(greenLED, LOW);
  }
  dataFile.println("");
  dataFile.close();
 
  readingIteration++;
  if(readingIteration == 5){
    Blink(blueLED, 500, 3);
    RTC.clearAlarmFlags();         //clear the alarm flags so that they can be triggered later
    currentTime = RTC.getRTCDateTime();
    setRTCAlarm(15, 0, 0);          //set the alarm for 20 minutes from now
    digitalWrite(fiveVoltEnable, LOW);
    sleepNow();
    digitalWrite(fiveVoltEnable, HIGH);
    delay(10);
    digitalWrite(selfReset, HIGH);    //After wake up, reset the arduino
  }
  
endloop:
  digitalWrite(greenLED, LOW);    
}

//convert the RTCDateTime structure to human readable string
void dateTime2String(RTCDateTime dt){

  //"00 / 00 / 00   00 : 00 : 00"
  // 01 2 34 5 67 8 90 1 23 4 56
  dateStr[6] = dt.year/10 + '0';
  dateStr[7] = dt.year%10 + '0';

  dateStr[0] = dt.months/10 + '0';
  dateStr[1] = dt.months%10 + '0';

  dateStr[3] = dt.days/10 + '0';
  dateStr[4] = dt.days%10 + '0';    

  dateStr[9] = dt.hours/10 + '0';
  dateStr[10] = dt.hours%10 + '0';    

  dateStr[12] = dt.minutes/10 + '0';
  dateStr[13] = dt.minutes%10 + '0';    

  dateStr[15] = dt.seconds/10 + '0';
  dateStr[16] = dt.seconds%10 + '0';    
}

//Interrupt service routine called when the RTC alarm interrupt fires
void rtcAlarmISR(){
  //nothing to do here
}

//Function to help set the RTC alarm
//Sets the alarm to a time (seconds, minutes, hours) from the current time
//seconds: 0-59
//minutes: 0-59
//hours: 0-23
void setRTCAlarm(uint8_t ss, uint8_t mm, uint8_t hh){
  alarmTime.seconds = currentTime.seconds + ss;
  alarmTime.minutes = currentTime.minutes + mm;
  alarmTime.hours = currentTime.hours + hh;

  //cheack that alarmSecond is valid
  if(alarmTime.seconds > 59){
    alarmTime.minutes  = alarmTime.minutes  + 1;
    alarmTime.seconds = alarmTime.seconds - 60;
  }

  //check that alarmMinute is valid
  if(alarmTime.minutes >59){
    alarmTime.minutes = alarmTime.minutes - 60;
    alarmTime.hours = alarmTime.hours + 1;
  }

  //check that alarmHour is valid
  if(alarmTime.hours> 23){
    alarmTime.hours = alarmTime.hours - 24;
  } 

  //set the alarm
  RTC.setAlarm1(alarmTime.seconds, alarmTime.minutes, alarmTime.hours);
}

void sleepNow(){         // here we put the arduino to sleep
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and 
   * wake up sources are available in which sleep mode.
   *
   * In the avr/sleep.h file, the call names of these sleep modes are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings 
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * For now, we want as much power savings as possible, so we 
   * choose the according 
   * sleep mode: SLEEP_MODE_PWR_DOWN
   * 
   */

  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin 

  attachInterrupt(1, rtcAlarmISR, LOW); // use interrupt 1 (pin 3) and run function
                                        // wakeUpNow when pin 3 gets LOW 

  sleep_mode();          // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(1);      // disables interrupt 1 on pin 3 so the 
                           // wakeUpNow code will not be executed 
                           // during normal running time.
}

boolean USB4000Ready(){
  Serial.write(' ');
  Serial.flush();   //wait for transmission to complete
  delay(50);
  incomingBytes[0] = Serial.read();
  incomingBytes[1] = Serial.read();
  incomingBytes[2] = Serial.read();
  incomingBytes[3] = Serial.read();
  incomingBytes[4] = Serial.read();
  incomingBytes[5] = Serial.read();
  
  /*
  logFile.println(incomingBytes[0]);
  logFile.println(incomingBytes[1]);
  logFile.println(incomingBytes[2]);
  logFile.println(incomingBytes[3]);
  logFile.println(incomingBytes[4]);
  logFile.println(incomingBytes[5]);
  logFile.flush();
  */
  if(incomingBytes[1] == 21){
//    digitalWrite(redLED, LOW);
//    delay(300);
//    digitalWrite(blueLED, LOW);
    return true;
  }else{
//    digitalWrite(blueLED, LOW);
//    delay(300);
//    digitalWrite(redLED, LOW);
    return false;
  }
}

void clearSerial(){
    //logFile.println("CS");
  while(Serial.available() > 0){        //clear the serial port
    //logFile.println(Serial.available());
    Serial.read();
    delay(5);
  }
}

void Blink (byte pin, int timing, int blinks){
  for(int i = 0 ; i< blinks; i++){
    digitalWrite(pin , HIGH);
    delay(timing);
    digitalWrite(pin , LOW);  
    delay(timing);
  }
}

