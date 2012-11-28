/*Instructions for trigger mode 3 - external hardware trigger

To take a scan send
'S' with no carriage return or line feed
This makes the USB4000 ready to be triggerd from the external signal
Once it is triggered, it will integrate for the given time, and then return the data once it's done
*/

#include <SPI.h>
#include <SD.h>
#include <RTC_DS3234.h>
#include <TLC_5916.h>
#include <avr/sleep.h>

//Constants
const unsigned int BATTERY48_NOT_CONNECTED_LEVEL = 100;   // Minimum ADC value that we assume means battery is not connected
const unsigned int BATTERY48_WARNING_LEVEL = 570;         // Set to 43 volts - but not actually implemented yet 
const unsigned int MAX_INTEGRATION_TIME = 10000;          // The maximum allowed USB4000 integration time
const unsigned int MIN_INTEGRATION_TIME = 10;             // The minimum allowed USB4000 integration time
const uint8_t MAX_LOOP_ERRORS = 5;                        // The number of errors in loop() before power cycling

const byte ACK = 6;      //ASCII ack used by USB4000
const byte NAK = 21;     //ASCII nak used by USB4000

//Colors
const byte RED =     0b00000001;
const byte GREEN =   0b00000010;
const byte BLUE =    0b00000100;
const byte YELLOW =  0b00000011;
const byte TEAL =    0b00000110;
const byte PURPLE =  0b00000101;
const byte WHITE =   0b00000111;

//Error values
const uint8_t SD_FAIL = (1<<3) | RED;
const uint8_t FILE_FAIL = (3<<3) | RED;
const uint8_t USB4000_FAIL = (5<<3) | RED;
const uint8_t BATTERY48_NOT_CONNECTED = (3<<3) | YELLOW;
const uint8_t USB4000_ERROR = (3<<3) | PURPLE;

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

const byte specTrigger = 7;
uint8_t const rgbLEDs [] = {7, A2, A3};

const byte greenLED = A2;
const byte blueLED = A3;
const byte batteryVoltage48 = A4;
const byte batteryVoltage37 = A5;


//GLOBAL VARIABLES
uint8_t loopIteration = 0;       // keeps track of the number of times through loop()
uint8_t loopErrors = 0;          // counts the number of times an error is encounterd in loop()
uint8_t errorValue = 0;          // holds an error value between 1 and 31, and an LED color
                                 // error value is stored in bits 7 to 3
                                 // color is stored in bits 2 to 0
                  
//USB4000 settings
unsigned int BOXCAR_WIDTH = 5;   //Values greater than 3 slow down transfer speeds
unsigned int PIXEL_TRANSFER_SPACING = 10;

//index of LED_NAMES + 1 is the bit address of that LED on the TLC_5916
char* const LED_NAMES [] ={"LED365", "LED430", "LED405", "LED390", "LED370", "AbsALL"};  
uint8_t const LED_CURRENT_GAIN [] = {CG_20, CG_30, CG_30, CG_100, CG_30, CG_10};
uint16_t ledIntegrationTime [] = {1000, 1000, 1000, 1000, 1000, 1000};  //array for initial integration times for each LED
                                                                        //the last one is for absorbance source

RTCDateTime currentTime;  //global variable to store the current date and time
RTCDateTime alarmTime;    //global variable to store the current alarm time

char dateStr[ ] = "00/00/00 00:00:00";          //a character array for printing date/time
char dataFileName[ ] = "files/MM_DD_YY.txt";	//a character array for the file name
File dataFile;
File logFile;

//helper variables
int bytesRead;
char dataBuffer[8];
int incomingBytes[6];
int incomingByte;
char data = 0;
uint8_t openTries = 0;  
uint8_t usb4000Tries = 0;
uint16_t maxValue;
uint16_t tempValue;

/*************************************************************************
Setup() runs once on startup. It does the foloowing:
1. Initializes all I/O pins
2. Initializes serial and SPI communication
3. Gets the current date/time from the real-time clock
4. Begins communication with the SD card. Opens or creates the data file
5. Reads from a config file
6. Initializes communication with the USB4000 spectrometer. Sets the boxcar,
     pixel mode, and trigger modes for spectrum collection.
7. Performs basic error checking to make sure everything is working properly
**************************************************************************/
void setup(){
  Serial.begin(9600); 
  
  //Initialize pins
  pinMode(fiveVoltEnable, OUTPUT);
  pinMode(selfReset, OUTPUT);
  digitalWrite(selfReset, LOW);
  pinMode(specTrigger, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  
  //Cycle through colors
  Blink(RED, 300, 1);
  Blink(GREEN, 300, 1);
  Blink(BLUE, 300, 1);
  Blink(YELLOW, 300, 1);
  Blink(TEAL, 300, 1);
  Blink(PURPLE, 300, 1);
  Blink(WHITE, 300, 1);

  //TODO  
  //Check 48 Volt Battery Connection and Level
  /*
  int battery48 = analogRead(batteryVoltage48)
  if(battery48 < BATTERY48_NOT_CONNECTED_LEVEL){
    errorValue =
    goto initializeError;
  } else if(battery48 < BATTERY48_WARNING_LEVEL){
    errorValue =
  }
  */
  
  //Initialize chip select control pinspins
  pinMode(10, OUTPUT);        //Arduino defined chip select  
  pinMode(csSD, OUTPUT);      //SD card chip select
  digitalWrite(csSD, HIGH);
  pinMode(csRTC, OUTPUT);     //Real time clock chip select
  digitalWrite(csRTC, HIGH);
  
  //turn on the 5V DC-DC converter. This automatically turns on the USB4000
  digitalWrite(fiveVoltEnable, HIGH); 
  
  SPI.begin();  //start SPI communication

  //see if the SD card is present and can be initialized:
  if (!SD.begin(csSD)) {
    errorValue = SD_FAIL;
    goto initializeError;
  }else{
    Blink(GREEN, 300, 1);    //1st green blink
  }
  
  RTC.begin(csRTC);                    //initialize RTC with chip select pin
  RTC.clearAlarmFlags();               //clear the alarm flags because we don't know what state the RTC was in
  currentTime = RTC.getRTCDateTime();  //need to do this initial read - otherwise strange bug occurs
                                       //where SD.open() fails the first time.
  dateTime2String(&currentTime);        //convert date and time to human readable string
                                       //which is stored in the variable dateStr
  
  //If a configuration file exists, load its info  
  if(SD.exists("config.txt")){
    dataFile = SD.open("config.txt");
    while(dataFile.available()){
      data = dataFile.read();
      if (data == 'P'){
        PIXEL_TRANSFER_SPACING = dataFile.parseInt();
      }else if (data == 'B'){
        BOXCAR_WIDTH = dataFile.parseInt();
      }else if(data >= 'a' && data <= 'f'){
        ledIntegrationTime[data - 'a'] = dataFile.parseInt();  
      }
    }
    dataFile.close();
  }
  
  //error check to make sure integration times are in correct range
  for(int i = 0; i<6; i++){
   ledIntegrationTime[i] = constrain(ledIntegrationTime[i], MIN_INTEGRATION_TIME, MAX_INTEGRATION_TIME);
  }
  
  //set the file name to the current date
  dataFileName [6] = currentTime.months/10 + '0';
  dataFileName [7] = currentTime.months%10 + '0';
  dataFileName [9] = currentTime.days/10 + '0';
  dataFileName [10] = currentTime.days%10 + '0';  
  dataFileName [12] = currentTime.year/10 + '0';
  dataFileName [13] = currentTime.year%10 + '0';     
      
  openTries = 0;
  
  //If the directory 'files' does not exist, create it
  if(!SD.exists("files")){
    SD.mkdir("files/");
  }
  
  //Try to open the data file for writing/appending new data
  do{
    dataFile = SD.open(dataFileName, FILE_WRITE);
    openTries++;
  }while(!dataFile && openTries < 4);

  if(!dataFile){                        //If unable to open the file, alert to an error
    errorValue = FILE_FAIL;
    goto initializeError;
  }
  
  
  //dataFile.print("Measurement at: ");
  //dataFile.println(dateStr);
  //dataFile.flush();
  
  pinMode(rtcAlarm, INPUT);            //Interrupt pin for RTC alarm
  
  //Start the TLLC5916 LED driver with pins 4 and 5 controlling the SPI communications
  tlc5916.begin(ledOutputEnable,ledLatchEnable);
  
  //Wait for Spec
  Blink(TEAL, 500, 6);
  
  usb4000Tries = 0;
  do{
    clearSerial();
    Serial.write(' ');
    Serial.flush();
    delay(10);
    incomingByte = Serial.read();
    Blink(TEAL, 500<<usb4000Tries, 1);
    usb4000Tries++;
  }while(incomingByte != NAK && usb4000Tries < 4);
  
  if(incomingByte != NAK){
    errorValue = USB4000_FAIL;
    goto initializeError; 
  }
  
  //Set boxcar average
  Serial.write('B');
  Serial.write(highByte(BOXCAR_WIDTH));
  Serial.write(lowByte(BOXCAR_WIDTH));
  delay(10);
  if(Serial.read() != ACK){
    errorValue = USB4000_ERROR;
    goto initializeError;   
  }

  //set Pixel Mode
  Serial.write('P');
  Serial.write(0);
  Serial.write(1);
  Serial.write(highByte(PIXEL_TRANSFER_SPACING));
  Serial.write(lowByte(PIXEL_TRANSFER_SPACING));
  Serial.flush();
  delay(10);
  if(Serial.read() != ACK){
    errorValue = USB4000_ERROR;
    goto initializeError;   
  }
  
  //Set trigger to 2 external synchronization
  Serial.write('T');
  Serial.write(0);
  Serial.write(3);
  Serial.flush();
  delay(10);
  if(Serial.read() != ACK){
    errorValue = USB4000_ERROR;
    goto initializeError;   
  }
  
initializeError:
  if(errorValue > 0){
    Blink(errorValue & 0b00000111, 500, errorValue >> 3);
    digitalWrite(selfReset, HIGH);  
  }
  
  //If program gets here, then everything is working fine
  Blink(WHITE, 300, 3);
}

void loop(){

  Serial.write('b');                    //change the spectrometer to ASCII mode
  Serial.write('B');
  Serial.flush();
  delay(10);
  clearSerial(); 
  
  //Set the spectrometer integration time
  Serial.write('I');
  Serial.write(highByte(ledIntegrationTime[loopIteration]));
  Serial.write(lowByte(ledIntegrationTime[loopIteration]));
  Serial.flush();
  delay(10);
  if(Serial.read() != ACK){
    errorValue = USB4000_ERROR;
    Blink(errorValue && 0b00000111, 500, errorValue >> 3);   
  }  

  Serial.write('a');                    //change the spectrometer to ASCII mode
  Serial.write('A');
  Serial.flush();
  delay(10);
  clearSerial(); 
  
  dataFile.print(LED_NAMES[loopIteration]);
  dataFile.write(' ');
  dataFile.print(dateStr);  
  dataFile.write(' ');
  
  //Check that the USB4000 is ready to take a scan
  if(!USB4000Ready()){
    Blink(RED, 500, 1);
    loopErrors++;
    goto endloop;
  }
  
  if(loopIteration < 5){
    tlc5916.disableOutput();
    tlc5916.ezSetCurrentConfigurationCode(LED_CURRENT_GAIN[loopIteration]);
    tlc5916.ezSetPinsOnOff(1 << loopIteration);
    tlc5916.enableOutput();
  } else{
    digitalWrite(twelveVoltEnable, HIGH);  //turn on lamp
    delay(1000); // give time to warm up
  }
    
  //Start the scan
  Serial.write('S');
  Serial.flush();
  delay(10);
  
  //USB4000 should immediately respond with 2 bytes
  //The first value is 'S' echo
  //The second value is etx or stx
  Serial.readBytes(dataBuffer, 2);

  if(dataBuffer[0] != 'S' || dataBuffer[1] == 3){  //error with Spec
    Blink(RED, 300, 3);
    loopErrors++;
    goto endloop;
  }

  Serial.setTimeout(ledIntegrationTime[loopIteration]+1000);
  
  //Trigger the spectrometer to start scan 
  digitalWrite(specTrigger, HIGH);
  delay(1);
  digitalWrite(specTrigger, LOW);
  
  //wait here until spectrum begins transmitting - will timeout if nothing received
  bytesRead = Serial.readBytes(dataBuffer, 1);
  tlc5916.disableOutput();  //turn off LED once scan has been captured
  digitalWrite(twelveVoltEnable, LOW); //turn off absorbance source
  
  if(bytesRead == 0){ //the read operation timedout so there must have been an error
    Blink(RED, 500, 1);
    loopErrors++;
    goto endloop; 
  }

  Serial.setTimeout(20);    //set the serial timeout to 20ms (probably can be shorter)
  tempValue = 6;
  maxValue = 0;
  while(Serial.readBytes(dataBuffer, 1)){
   digitalWrite(greenLED, HIGH);
   if(dataBuffer[0] = ' '){
     dataFile.write(' ');
     if(tempValue > maxValue && tempValue < 32768){
       maxValue = tempValue;
     } 
   }else if(dataBuffer[0] != '\n' && dataBuffer[0] != '>'){
     dataFile.write(dataBuffer[0]);
     tempValue = tempValue*10 + dataBuffer[0]-'0';
   }
   digitalWrite(greenLED, LOW);
  }
  dataFile.println("");
  dataFile.flush();  //make sure all the data was written to the SD card

/*  
  //check if saturating the detector
  if(maxValue >= 32767 && ledIntegrationTime[loopIteration] < MAX_INTEGRATION_TIME){
    ledIntegrationTime[loopIteration] = ledIntegrationTime[loopIteration]>>1;  //divide integration time by 2
    loopIteration--;
  }else if (maxValue < 16384){             //check if more signal possible
    ledIntegrationTime[loopIteration] = ledIntegrationTime[loopIteration]<<1; //multiply integration time by 2
    loopIteration--;
  }
*/  
  loopIteration++;
  if(loopIteration == 6){
    dataFile.close();
    writeConfigFile();
    Blink(WHITE, 500, 3);
    RTC.clearAlarmFlags();         //clear the alarm flags so that they can be triggered later
    currentTime = RTC.getRTCDateTime();
    setRTCAlarm(15, 0, 0);          //set the alarm for 20 minutes from now
    digitalWrite(fiveVoltEnable, LOW);
    sleepNow();
    digitalWrite(fiveVoltEnable, HIGH);
    delay(10);
    digitalWrite(selfReset, HIGH);    //After wake up, reset the arduino
  }
  loopErrors = 0;
  
endloop:
  //If there have been repeated loop errors, we will try to restart
  if(loopErrors >= MAX_LOOP_ERRORS){
    dataFile.close();
    writeConfigFile();
    digitalWrite(selfReset, HIGH);
  }
  digitalWrite(greenLED, LOW);    
}

void writeConfigFile(){
  SD.remove("config.txt");  //remove the current config file
  dataFile = SD.open("config.txt", FILE_READ);
  dataFile.print("P=");
  dataFile.println(PIXEL_TRANSFER_SPACING);
  dataFile.print("B=");
  dataFile.println(BOXCAR_WIDTH);
  for(int i = 0; i < 6; i ++){
    dataFile.print('a' + i);
    dataFile.print('=');
    dataFile.println(ledIntegrationTime[i]);    
  }
  dataFile.close();
}

//convert the RTCDateTime structure to human readable string
void dateTime2String(RTCDateTime * dt){

  //"00 / 00 / 00   00 : 00 : 00"
  // 01 2 34 5 67 8 90 1 23 4 56
  dateStr[6] = dt->year/10 + '0';
  dateStr[7] = dt->year%10 + '0';

  dateStr[0] = dt->months/10 + '0';
  dateStr[1] = dt->months%10 + '0';

  dateStr[3] = dt->days/10 + '0';
  dateStr[4] = dt->days%10 + '0';    

  dateStr[9] = dt->hours/10 + '0';
  dateStr[10] = dt->hours%10 + '0';    

  dateStr[12] = dt->minutes/10 + '0';
  dateStr[13] = dt->minutes%10 + '0';    

  dateStr[15] = dt->seconds/10 + '0';
  dateStr[16] = dt->seconds%10 + '0';    
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
//    digitalWrite(blueLED, LOW);
//    delay(300);
//    digitalWrite(blueLED, LOW);
    return true;
  }else{
//    digitalWrite(blueLED, LOW);
//    delay(300);
//    digitalWrite(blueLED, LOW);
    return false;
  }
}

void clearSerial(){
  logFile = SD.open("logFiles.log", FILE_WRITE); 
  logFile.println("CS");
  while(Serial.available() > 0){        //clear the serial port
    //logFile.println(Serial.available());
    logFile.write(Serial.read());
    delay(5);
  }
  logFile.close();
}

void Blink (uint8_t color, int timing, int blinks){
  for(int i = 0 ; i< blinks; i++){
    digitalWrite(rgbLEDs[0], bitRead(color, 0));
    digitalWrite(rgbLEDs[1], bitRead(color, 1));
    digitalWrite(rgbLEDs[2], bitRead(color, 2));
    delay(timing);
    digitalWrite(rgbLEDs[0] , LOW);
    digitalWrite(rgbLEDs[1] , LOW);
    digitalWrite(rgbLEDs[2] , LOW);
    delay(timing);
  }
}

