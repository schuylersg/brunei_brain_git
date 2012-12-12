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
const uint16_t MAX_INTEGRATION_TIME = 10000;              // The maximum allowed USB4000 integration time
const uint16_t MIN_INTEGRATION_TIME = 10;                 // The minimum allowed USB4000 integration time
const uint8_t MAX_LOOP_ERRORS = 5;                        // The number of errors in loop() before power cycling
const uint16_t LAMP_WARM_UP_TIME = 1000;                  // Lamp warm up time in milliseconds
const uint8_t MINIMUM_ALARM_SECCONDS = 5;                 // The minimum time the alarm can be set for in seconds

const byte ACK = 6;      //ASCII ack used by USB4000
const byte NAK = 21;     //ASCII nak used by USB4000

const uint8_t WITH_CRLF = 1;
const uint8_t NO_CRLF = 0;

//Colors
const uint8_t RED =     0b00000001;
const uint8_t GREEN =   0b00000010;
const uint8_t BLUE =    0b00000100;
const uint8_t YELLOW =  0b00000011;
const uint8_t TEAL =    0b00000110;
const uint8_t PURPLE =  0b00000101;
const uint8_t WHITE =   0b00000111;

//Error values
const uint8_t SD_FAIL = (1<<3) | RED;
const uint8_t FILE_FAIL = (3<<3) | RED;
const uint8_t USB4000_FAIL = (5<<3) | RED;
const uint8_t BATTERY48_NOT_CONNECTED = (3<<3) | YELLOW;
const uint8_t USB4000_ERROR_1 = (1<<3) | TEAL;
const uint8_t USB4000_ERROR_2 = (2<<3) | TEAL;
const uint8_t USB4000_ERROR_3 = (3<<3) | TEAL;
const uint8_t USB4000_ERROR_4 = (4<<3) | BLUE;
const uint8_t USB4000_ERROR_5 = (5<<3) | TEAL;

//PIN ASSIGNMENTS
const uint8_t SpecRx = 0;
const uint8_t SpecTx = 1;
const uint8_t batteryChargerStatus = 2;
const uint8_t rtcAlarm = 3;
const uint8_t ledLatchEnable = 4;
const uint8_t ledOutputEnable = 5;
const uint8_t selfReset = 6;
const uint8_t csSD = 8;
const uint8_t csRTC = 9;

const uint8_t fiveVoltEnable = A0;
const uint8_t twelveVoltEnable = A1;

const uint8_t specTrigger = 7;
const uint8_t rgbLEDs [] = {7, A2, A3};

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
//Data logging settings
uint8_t dataLogSeconds = 15;
uint8_t dataLogMinutes = 0;
uint8_t dataLogHours = 0;
            
//USB4000 settings
uint16_t BOXCAR_WIDTH = 5;   //Values greater than 3 slow down transfer speeds
uint16_t PIXEL_TRANSFER_SPACING = 10;
uint16_t currentIntegrationTime = 0;

//index of LED_NAMES + 1 is the bit address of that LED on the TLC_5916
char* const LED_NAMES [] ={"LED365", "LED430", "LED405", "LED390", "LED370", "AbsALL", "Turbid"};  
uint8_t const LED_CURRENT_GAIN [] = {CG_20, CG_30, CG_30, CG_100, CG_30, CG_10, CG_10};
uint16_t ledIntegrationTime [] = {1000, 1000, 1000, 1000, 1000, 50, 50};  //array for initial integration times for each LED
                                                                        //the last one is for absorbance source
boolean recordingDarkSpectrum = false;
uint8_t darkSpectrum = 0b0;            // each bit refers to a dark spectrum recorded for 
                                       // the integration time specified by that bit index
                                       // So for the default values, it should end up being
                                       // 0b00100001

boolean configParamsChanged = false;

RTCDateTime currentTime;  //global variable to store the current date and time
RTCDateTime alarmTime;    //global variable to store the current alarm time

char dateStr[ ] = "00/00/00 00:00:00";          //a character array for printing date/time
char dataFileName[ ] = "files/MM_DD_YY.txt";	//a character array for the file name
File dataFile;

//helper variables
int bytesRead;
char dataBuffer[8];
int incomingByte;
char data = 0;
uint8_t openTries = 0;  
uint8_t usb4000Tries = 0;
uint8_t startLoc = 0;

uint16_t bv37Analog;
uint16_t bv48Analog;

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
  pinMode(twelveVoltEnable, OUTPUT);
  pinMode(selfReset, OUTPUT);
  digitalWrite(selfReset, LOW);
  pinMode(specTrigger, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  
  //read the battery voltages
  analogReference(INTERNAL);  //set the analog reference to the internal 1.1 volts
  delay(10);
  for(uint8_t i = 0; i<5; i++){     // The arduino website claims that after changing reference sources
    analogRead(batteryVoltage37);   // the first few analog reads can be off
  }
  delay(10);
  bv37Analog = analogRead(batteryVoltage37);
  
  analogRead(batteryVoltage48);
  delay(10);
  bv48Analog = analogRead(batteryVoltage48);
  
  
  //Cycle through colors
  Blink(RED, 300, 1);
  Blink(GREEN, 300, 1);
  Blink(BLUE, 300, 1);
  Blink(YELLOW, 300, 1);
  Blink(TEAL, 300, 1);
  Blink(PURPLE, 300, 1);
  Blink(WHITE, 300, 1);
  
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
  //Try to open the config file for reading
  openTries = 0;
  do{
    dataFile = SD.open("CONFIG.TXT", FILE_READ);
    openTries++;
  }while(!dataFile && openTries < 4);

  if(dataFile){
    Blink(GREEN, 300, 1);    //2nd green blink
    while(dataFile.available()){
      data = dataFile.read();
      if (data == 'P'){
        PIXEL_TRANSFER_SPACING = (uint16_t) dataFile.parseInt();
      }else if (data == 'B'){
        BOXCAR_WIDTH = (uint16_t)dataFile.parseInt();
      }else if (data == 'S'){
        dataLogSeconds = (uint16_t)dataFile.parseInt();
      }else if (data == 'M'){
        dataLogMinutes = (uint16_t)dataFile.parseInt();
      }else if (data == 'H'){
        dataLogHours = (uint16_t)dataFile.parseInt();
      }else if(data >= 'a' && data <= 'g'){
        ledIntegrationTime[data - 'a'] = (uint16_t) dataFile.parseInt();  
      }else if(data == '*'){
        break;
      }
    }
    dataFile.close();
  }else{
    Blink(PURPLE, 300, 1); 
  }
  
  //error check to make sure integration times are in correct range
  for(int i = 0; i<7; i++){
    int j = constrain(ledIntegrationTime[i], MIN_INTEGRATION_TIME, MAX_INTEGRATION_TIME);
    if(j != ledIntegrationTime[i]){
     ledIntegrationTime[i] = j;
     configParamsChanged = true;
    }
  }

  //Open the logfile - if this is the first time, then add a header line
  dataFile = SD.open("LOGFILE.TXT", FILE_WRITE);  
 //if(logFile.size() < 5){
 //   dataFile.println("Date\tTime\t3.7V Battery\t48V Battery\tCharging\tErrors");  
 // }
  
  //print some status information
  dataFile.println("");
  dataFile.print(dateStr);  //Measurement date and time
  dataFile.print('\t');
  dataFile.print(bv37Analog);  //3.7 battery voltage
  dataFile.print('\t');
  dataFile.print(bv48Analog);  //48 battery voltage
  dataFile.print('\t');
  dataFile.print(digitalRead(batteryChargerStatus));  //if the 3.7v battery is charging
  dataFile.print('\t');
  dataFile.close();
  
  //set the file name to the current date
  dataFileName [6] = currentTime.months/10 + '0';
  dataFileName [7] = currentTime.months%10 + '0';
  dataFileName [9] = currentTime.days/10 + '0';
  dataFileName [10] = currentTime.days%10 + '0';  
  dataFileName [12] = currentTime.year/10 + '0';
  dataFileName [13] = currentTime.year%10 + '0';     
  
  //If the directory 'files' does not exist, create it
  if(!SD.exists("files")){
    SD.mkdir("files/");
  }
  
  //Try to open the data file for writing/appending new data
  openTries = 0;
  do{
    dataFile = SD.open(dataFileName, FILE_WRITE);
    openTries++;
  }while(!dataFile && openTries < 4);

  if(!dataFile){                        //If unable to open the file, alert to an error
    errorValue = FILE_FAIL;
    goto initializeError;
  }
  
  pinMode(rtcAlarm, INPUT);            //Interrupt pin for RTC alarm
  
  //Start the TLLC5916 LED driver with pins 4 and 5 controlling the SPI communications
  tlc5916.begin(ledOutputEnable,ledLatchEnable);
  
  delay(500);
  //Wait for spectrometer to finish initializing
  //If everything goes well, green LED will blink 6 times
  Blink(GREEN, 500, 5);
  
  usb4000Tries = 0;
  do{
    Blink(GREEN, 500<<usb4000Tries, 1);
    clearSerial();
    Serial.write(' ');
    Serial.flush();
    delay(10);
    incomingByte = Serial.read();
    usb4000Tries++;
  }while(incomingByte != NAK && usb4000Tries < 4);
  
  //Check if there has been a major failure with the USB4000 startup - if so, restart
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
    errorValue = USB4000_ERROR_1;
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
    errorValue = USB4000_ERROR_2;
    goto initializeError;   
  }
  
  //Set trigger to 2 external synchronization
  Serial.write('T');
  Serial.write(0);
  Serial.write(3);
  Serial.flush();
  delay(10);
  if(Serial.read() != ACK){
    errorValue = USB4000_ERROR_3;
    goto initializeError;   
  }
  
  //change the spectrometer to ASCII mode
  Serial.write('a');
  Serial.write('A');
  Serial.flush();
  if(!asciiACKRead(NO_CRLF)){
    errorValue = USB4000_ERROR_4;
    goto initializeError;
  }
  
initializeError:
  if(errorValue > 0){
    Blink(errorValue & 0b00000111, 500, errorValue >> 3);
    digitalWrite(selfReset, HIGH);  
  }
  
  //If program gets here, then everything is working fine
  Blink(GREEN, 300, 3);
}

void loop(){
  recordingDarkSpectrum = false;
  //If we don't need to change the integration time, then we already
  //must have the dark spectrum as well, so skip ahead
  if(currentIntegrationTime == ledIntegrationTime[loopIteration]){
    goto doneSettingIntegration;
  }
  
  //check if the darkSpectrum bit has been set for this iteration
  if(!(1<<(loopIteration) & darkSpectrum)){
      recordingDarkSpectrum = true;  
  }

  //Go through and set the flag for all other scans with the same integration time
  for(uint8_t i = loopIteration; i < 7; i++){
   if(ledIntegrationTime[loopIteration] == ledIntegrationTime[i]){
    darkSpectrum = darkSpectrum | 1<<i;
   }
  }
   
  //Convert binary value into ASCII characters
  dataBuffer[0] = ledIntegrationTime[loopIteration]/10000;
  dataBuffer[1] = (ledIntegrationTime[loopIteration]%10000)/1000;
  dataBuffer[2] = (ledIntegrationTime[loopIteration]%1000)/100;
  dataBuffer[3] = (ledIntegrationTime[loopIteration]%100)/10;
  dataBuffer[4] = (ledIntegrationTime[loopIteration]%10);
  
  //Remove the leading zeros
  startLoc = 0;
  while(dataBuffer[startLoc] == 0){
    startLoc++;
  }

  //Set the spectrometer integration time
  Serial.write('I');
  Serial.flush();
  delay(10);
  
  Serial.read();

  while(startLoc<5){
    Serial.write(dataBuffer[startLoc] + '0');
    Serial.flush(); 
    delay(10);
    Serial.read();//read the echoed character
    startLoc++;
  }
  dataFile.flush();
  
  Serial.write(13);
  Serial.flush();
  delay(20);
  
  //check that command received successfully
  if(!asciiACKRead(WITH_CRLF)){
    errorValue = USB4000_ERROR_1;
    Blink(errorValue & 0b00000111, 500, errorValue >> 3);
    loopErrors++;
    clearSerial();
    goto endloop;  
  }
  
  currentIntegrationTime = ledIntegrationTime[loopIteration];
  delay(100);
  
doneSettingIntegration:

  if(recordingDarkSpectrum){
    dataFile.print("Dark  ");
  }else{
    dataFile.print(LED_NAMES[loopIteration]);
  }
  dataFile.write(' ');
  dataFile.print(dateStr);
  
  //Check that the USB4000 is ready to take a scan
  if(!USB4000Ready()){
    errorValue = USB4000_ERROR_2;
    Blink(errorValue & 0b00000111, 500, errorValue >> 3);
    loopErrors++;
    goto endloop;
  }
  
  if(recordingDarkSpectrum){
    //do nothing
  }else if(loopIteration < 5 ){
    tlc5916.disableOutput();
    tlc5916.ezSetCurrentConfigurationCode(LED_CURRENT_GAIN[loopIteration]);
    tlc5916.ezSetPinsOnOff(1 << loopIteration);
    tlc5916.enableOutput();
    dataFile.flush();
  }else if(loopIteration ==5){    //Absorbance measurement
    digitalWrite(twelveVoltEnable, HIGH);  //turn on lamp
    delay(LAMP_WARM_UP_TIME); // give time to warm up
  }else{  //turbidity measurement
    tlc5916.disableOutput();
    tlc5916.ezSetCurrentConfigurationCode(LED_CURRENT_GAIN[loopIteration]);
    tlc5916.ezSetPinsOnOff(0b00011111);    //turn on all LEDs
    tlc5916.enableOutput();
    dataFile.flush();
  }
  
  digitalWrite(blueLED, HIGH);
  //Start the scan
  Serial.write('S');
  Serial.flush();
  
  //USB4000 should immediately respond with 2 bytes
  //The first value is 'S' echo
  //The second value is etx or stx
  dataBuffer[0] = 0;
  dataBuffer[1] = 3;
  bytesRead = Serial.readBytes(dataBuffer, 2);

  if(dataBuffer[0] != 'S' || dataBuffer[1] == 3){  //error with Spec
    tlc5916.disableOutput();  //turn off LED
    digitalWrite(twelveVoltEnable, LOW); //turn off absorbance source
    errorValue = USB4000_ERROR_3;
    Blink(errorValue & 0b00000111, 500, errorValue >> 3);
    clearSerial();
    loopErrors++;
    goto endloop;
  }
  
  delay(30);  //need this for low integration times
  
  Serial.setTimeout(ledIntegrationTime[loopIteration] + 500);  //set the timeout to the integration time plus 0.5 second
  
  //Trigger the spectrometer to start scan 
  digitalWrite(specTrigger, HIGH);
  delay(1);
  digitalWrite(specTrigger, LOW);
  
  //wait here until spectrum begins transmitting - will timeout if nothing received
  bytesRead = Serial.readBytes(dataBuffer, 5);
  
  //Gets here once the USB4000 starts transmitting data
  tlc5916.disableOutput();  //turn off LED once scan has been captured
  digitalWrite(twelveVoltEnable, LOW); //turn off absorbance source
  digitalWrite(blueLED, LOW);
  
  if(bytesRead == 0){ //the read operation timedout so there must have been an error
    errorValue = USB4000_ERROR_4;
    Blink(errorValue & 0b00000111, 500, errorValue >> 3);
    clearSerial();
    loopErrors++;
    goto endloop;
  }
  
  Serial.setTimeout(20);    //set the serial timeout to 20ms (probably can be shorter)
  while(Serial.readBytes(dataBuffer, 1)){
   digitalWrite(greenLED, HIGH);
   if(dataBuffer[0] == 13 || dataBuffer[0] == '>' || dataBuffer[0] == 10){
      //do nothing 
   }else{
     dataFile.write(dataBuffer[0]);
   }
   digitalWrite(greenLED, LOW);
  }
  dataFile.println("");
  dataFile.flush();  //make sure all the data was written to the SD card
  
  if(!recordingDarkSpectrum){
    loopIteration++;
  }
  
  if(loopIteration >= 7){
    dataFile.close();
    
//    logFile.print("Charge Status = ");
//    logFile.println(digitalRead(batteryChargerStatus));
//    logFile.close();
    
    dataLogSeconds = dataLogSeconds;
    configParamsChanged = true;
    writeConfigFile();                  //update the config file with any changes
    Blink(WHITE, 100, 3);           
    RTC.clearAlarmFlags();              //clear the alarm flags so that they can be triggered later
    currentTime = RTC.getRTCDateTime();
    setRTCAlarm(dataLogSeconds, dataLogMinutes, dataLogHours);              //set the alarm for 20 minutes from now
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
  
  if(loopErrors > 0 && recordingDarkSpectrum){ 
    darkSpectrum &= ~(1<<loopIteration);
  }
  digitalWrite(greenLED, LOW);    
}

void writeConfigFile(){
  if(!configParamsChanged){
    return;
  }
  
  SD.remove("config.txt");  //remove the current config file
  dataFile = SD.open("config.txt", FILE_WRITE);
  dataFile.print("P=");
  dataFile.println(PIXEL_TRANSFER_SPACING);
  dataFile.print("B=");
  dataFile.println(BOXCAR_WIDTH);
  for(int i = 0; i < 7; i ++){
    dataFile.print((char)('a' + i));
    dataFile.print('=');
    dataFile.println(ledIntegrationTime[i]);    
  }
  dataFile.print("S=");
  dataFile.println(dataLogSeconds);
  dataFile.print("M=");
  dataFile.println(dataLogMinutes);
  dataFile.print("H=");
  dataFile.println(dataLogHours);
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
//input variables can be arbitrary numbers but limiited to 255 by variable size
//must be at least 5 seconds
void setRTCAlarm(uint8_t ss, uint8_t mm, uint8_t hh){
  if(hh == 0 && mm == 0 && ss < MINIMUM_ALARM_SECCONDS)
    ss=MINIMUM_ALARM_SECCONDS;
  
  alarmTime.seconds = currentTime.seconds + ss;
  alarmTime.minutes = currentTime.minutes + mm + alarmTime.seconds/60;  
  alarmTime.hours = currentTime.hours + hh + alarmTime.minutes/60;

  alarmTime.seconds = alarmTime.seconds%60;
  alarmTime.minutes = alarmTime.minutes%60;
  alarmTime.hours = alarmTime.hours%24;

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

//check if USB4000 is responding while in ASCII mode
boolean USB4000Ready(){
  char bytes [6];
  int numBytesRead;
  uint8_t expectedNumBytes = 6;  
  Serial.write(' ');
  Serial.flush();   //wait for transmission to complete
  Serial.setTimeout(100);  //set the timeout to something reasonable
  numBytesRead = Serial.readBytes(bytes, expectedNumBytes);
  if(numBytesRead != expectedNumBytes)
    return false;
  
  return (bytes[1] == 21);
}

void clearSerial(){
  while(Serial.available() > 0){        //clear the serial port
    Serial.read();
    delay(10);
  }
}

boolean asciiACKRead(uint8_t mode){
  char bytes [7];
  int numBytesRead;
  uint8_t expectedNumBytes = 7;
  if(mode == NO_CRLF)
    expectedNumBytes = 5;
 
  Serial.setTimeout(1000);  //I found that 300ms was not enough. 1000ms seems to be enough
  numBytesRead = Serial.readBytes(bytes, expectedNumBytes);
  if(numBytesRead != expectedNumBytes){
    return false;
  }

  if(mode == WITH_CRLF && bytes[2] == ACK)
    return true;
  if(mode == NO_CRLF && bytes[0] == ACK)
    return true;
    
  return false;
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
