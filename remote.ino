// ===== Libraries =====
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include "DHT.h"
#include "printf.h"

// ===== Declarations for OLED Display =====
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
 
// On an arduino UNO:       A4(SDA), A5(SCL)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define HEARTBEATLED 16
#define HUMIDLED 15
#define TEMPLED 14
#define LEDPIN 13
#define RE_SELECT 7
#define BACKBUTTON 6
#define RE_CLK 5
#define RE_DATA 4
#define BUTTONINT 2
#define DHTPIN 17

#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

// Initalize NRF24
RF24 radio(9,10);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

float h = 0.0;
float t = 0.0;
float f = 0.0;
float prev_t = 0.0;
float prev_h = 0.0;
float hic =0.0;
float hif = 0.0;

// The difference between the respective values is the hysteresis for that metric.
float onTemp = 24.0;
float offTemp = 21.0;
float onHumidity = 48.0;
float offHumidity = 40.0;

#define NONE 0
#define HUMIDITY 1
#define TEMPERATURE 2
#define REMOTE_ON 3
#define REMOTE_OFF 4
#define REMOTE_DELAY 5

// ===== Fan Modes =====
#define FAN_AUTO 0
#define FAN_ON 1
#define FAN_OFF 2
#define FAN_DELAY 3
#define GET_DATA 4
#define STOP_DATA 5

// ===== Pointer Position =====
#define ON 0
#define OFF 1
#define AUTO 2
int pointerPosition = 2; // Default to pointing at AUTO mode
uint8_t modeSelect = 2;

// ==== Variables set by remote communication =====
uint8_t currFanStatus = 0;
uint8_t fanMode = 0;  // Assume it is AUTO mode. Value is set by remote commands
uint8_t remoteCommand = 0;
uint8_t reason = 0;
// ================================================

bool serCom = false;
bool fanOn = false;
bool redLedState = false;
bool blueLedState = false;

bool newData = false;
bool dataComplete = false;

// ==== Structs ====
struct updating {
  // Display updates
  bool displayPointer;
  bool data;
  bool all;

  // Fan updates
  bool fanMode;
} updates = {false, false, false, false};

void setup() {

  pinMode(LEDPIN, OUTPUT);
  pinMode(TEMPLED, OUTPUT);
  pinMode(HUMIDLED, OUTPUT);
  pinMode(HEARTBEATLED, OUTPUT);
  pinMode(BACKBUTTON, INPUT_PULLUP);
  pinMode(BUTTONINT, INPUT_PULLUP);
  pinMode(RE_SELECT, INPUT_PULLUP);
  pinMode(RE_CLK, INPUT_PULLUP);
  pinMode(RE_DATA, INPUT_PULLUP);
  
  digitalWrite(LEDPIN, LOW);
  digitalWrite(TEMPLED, LOW);
  digitalWrite(HUMIDLED, LOW);
  digitalWrite(HEARTBEATLED, LOW);

  attachInterrupt(digitalPinToInterrupt(BUTTONINT), buttonRead, FALLING);
  
  dht.begin();
  radio.begin();  
  enableSerial(true); // Set to true to turn on reporting over serial com
  printf_begin();
  
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  
//  radio.setChannel(120);
  radio.setRetries(15,15);
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();
  radio.printDetails();
  sendMessage(GET_DATA);
  while(!radio.available());

  receiveData();
  
  displayInit(true);
}

void loop() 
{
  systemUpdates();
  
  if (newData)
    printReport();
  
  if(radio.available())
    receiveData();
}

// ====== ISR =====
void buttonRead(void)
{
  bool backButton = true;
  bool REselect = true;
  bool REclk = true;
  bool REdata = true;

  backButton = digitalRead(BACKBUTTON);
  REselect = digitalRead(RE_SELECT);
  REclk = digitalRead(RE_CLK);
  REdata = digitalRead(RE_DATA); 

  if (!REclk)
  {
    if (REclk == REdata) // CW rotation
    {
      pointerPosition++;

      if (pointerPosition >= 2)
        pointerPosition = 2;

      Serial.print(F("Pointer: "));
      Serial.println(pointerPosition);

      updates.displayPointer = true;
    }
    else if (REclk != REdata) // CCW rotation
    {
      pointerPosition--;

      if (pointerPosition <= 0)
        pointerPosition = 0;

      Serial.print(F("Pointer: "));
      Serial.println(pointerPosition);

      updates.displayPointer = true;
    }
  }

  if(!REselect)
  {
    Serial.println(F("Pointer Select"));
    modeSelect = pointerPosition;
    updates.fanMode = true;
  }

//  if(!backButton)
//  {
//    sendMessage(FAN_ON);
//  }
}

// ====== Funtions =====
void systemUpdates(void)
{
  if (updates.displayPointer)
  {
    printDataOLED();
    updates.displayPointer = false;
  }

  if (updates.fanMode)
  {
    switch (modeSelect)
    {
      case ON:
        sendMessage(FAN_ON);
      break;
      case OFF:
        sendMessage(FAN_OFF);
      break;
      case AUTO:
        sendMessage(FAN_AUTO);
      break;
      default:
        sendMessage(FAN_AUTO);
      break;
    }
    updates.fanMode = false;
  }
}

void serialEvent(void)
{
  String command = Serial.readStringUntil('\n');  // Read until newline char. The newline char is truncated from String.

  if (command.equals(F("FAN AUTO")))
    sendMessage(FAN_AUTO);
    
  if (command.equals(F("FAN ON")))
    sendMessage(FAN_ON);
    
  if (command.equals(F("FAN OFF")))
    sendMessage(FAN_OFF);
   
  if (command.equals(F("GET DATA")))
    sendMessage(GET_DATA);

  if (command.equals(F("STOP DATA")))
    sendMessage(STOP_DATA);
}

// ******Send a Message******
bool sendMessage(uint8_t v) 
{
  char payload[2] = {'c', 'a'};
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.stopListening();

  switch(v)
  {
    case FAN_AUTO:
      payload[1] = 'a';
    break;
    case FAN_ON:
      payload[1] = 'n';
    break;
    case FAN_OFF:
      payload[1] = 'f';;
    break;
    case FAN_DELAY:
      payload[1] = 'd';
    break;
    case GET_DATA:
      payload[1] = 'g';
    break;
    case STOP_DATA:
      payload[1] = 's';
    break;
    default:
      payload[1] = 'a';
    break;
  }
  
  bool ok = radio.write(&payload, sizeof(payload));

  if (ok)
  {
    switch(v)
    {
      case FAN_AUTO:
        Serial.println(F("Fan auto!"));
      break;
      case FAN_ON:
        Serial.println(F("Fan on!"));
        digitalWrite(HUMIDLED, HIGH);
        digitalWrite(TEMPLED, HIGH);
      break;
      case FAN_OFF:
        Serial.println(F("Fan off!"));
        digitalWrite(HUMIDLED, LOW);
        digitalWrite(TEMPLED, LOW);
      break;
      case FAN_DELAY:
        Serial.println(F("Fan delay: "));
      break;
      case GET_DATA:
        Serial.println(F("Begin Data Stream"));
      break;
      case STOP_DATA:
        Serial.println(F("Stopping Data Stream"));
      break;
    }
  }

  radio.startListening();
  return ok;
}

bool sendACK(void)
{
  uint8_t ack = 255;
  return radio.write(&ack, sizeof(uint8_t));
}

// ******Receive a Message******
void receiveData(void) 
{
  // Positions in the payload
  #define MESSAGE_TYPE 0
  #define DATA_TYPE 1
  #define STATUS_TYPE 1
  #define COMMAND 1
  #define STAT 2
  #define MODE 2

  /*
   *  The received data will arrive in either of the following forms
   *    1. Data
   *    2. Control
   *    3. Status
   *    
   *  What form it is, is determined by the first byte (char) of the payload. 
   *  Then the payload follows.
   *  
   *  For Data:
   *    message[0] = The message type ('d' for data)
   *    message[1] = The data type ('h' for humidity, 't' for temperature in Celcius, 'f' for temperature in Fahrenheit)
   *    message[2-5] = The actual data  
   *    
   *  For Command:
   *    message[0] = The message type ('c' for command) 
   *    message[1] = The actual command
   *    
   *  For Status:
   *    message[0] = The message type ('s' for status)
   *    message[1] = The status type ('m' for fan mode, 's' for current status)
   *    message[2] = The actual mode/current status 
   *                            mode: ('n' for the fan on via remote, 'f' for the fan off via remote, 'd' for the delay mode, 'a' for auto mode (normal)
   *                            current status: ('H' Humidity turned on the fan, 'T' Temperature turned on the fan, 'O' The fan is currently off
   */
  
  char message[10];
  radio.read(&message, sizeof(message)); 
  
  char messageType = message[MESSAGE_TYPE];
  
  if (messageType == 'c')  // It is an incoming command
  {
    remoteCommand = message[COMMAND];  

    if (remoteCommand == '!') // Exclamation point is used as the 'End of Transmission' EOT code
      newData = true;
  }

  else if (messageType == 's')  // It is an incoming status
  {
    char statusType = message[STATUS_TYPE];

    if (statusType == 'm') // Fan mode. 
    {
      char mode = message[MODE];
      
      if (mode == 'n') // Fan was turned on by the base station. ie. it will be on until a new mode command sent or powerloss
      {
        fanMode = FAN_ON;
        digitalWrite(HUMIDLED, HIGH);
        digitalWrite(TEMPLED, HIGH);
      }
      else if (mode == 'f')  // Fan was turned off by the base station. ie. it will be off until a new mode command sent or powerloss
      { 
        fanMode = FAN_OFF;
        digitalWrite(HUMIDLED, LOW);
        digitalWrite(TEMPLED, LOW);
      }
      else if (mode == 'd')  // Fan was placed into delay mode.
        fanMode = FAN_DELAY;
      else if (mode == 'a')  // Fan is in automatic (normal) mode
        fanMode = FAN_AUTO;
    }

    if (statusType == 's') // Current status.
    {
      char stat = message[STAT];
      
      if (stat == 'H')  // Fan is currently running due to humidity
      {
        currFanStatus = FAN_ON;
        reason = HUMIDITY;
        digitalWrite(HUMIDLED, HIGH);
      } else if (stat == 'T') // Fan is currently running due to temperature
      {
        currFanStatus = FAN_ON;
        reason = TEMPERATURE;
        digitalWrite(TEMPLED, HIGH);
      }
      else if (stat == 'O') // Fan is currently off
      {
        currFanStatus = FAN_OFF;
        reason = NONE;
      }
    }
  }

  else if (messageType == 'd')  // It is incoming data
  {
    uint32_t d = 0;
    
    d |= uint32_t(message[1]) << 24;
    d |= uint32_t(message[2]) << 16;
    d |= uint32_t(message[3]) << 8;
    d |= uint8_t(message[4]);
    h = float(d); // Humidity

    d = 0;
    d |= uint32_t(message[5]) << 24;
    d |= uint32_t(message[6]) << 16;
    d |= uint32_t(message[7]) << 8;
    d |= uint8_t(message[8]);
    t = float(d); // Temperature in Celcius
    f = (t * 1.8) + 32; // Convert to fahrenheit

    reason = uint8_t(message[9]);

    dataComplete = true;
  }

  if (dataComplete)
  {
    newData = true;
    dataComplete = false;
  }
  radio.startListening();
}

void enableSerial(bool en)
{
  if(en)
  {
    Serial.begin(57600);
    serCom = true;
  }
}

void printReport(void)
{
  // Compute heat index in Fahrenheit (the default)
  hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.print(F("°F"));
  Serial.print(F(" Reason: "));
  Serial.println(Reason());

  if (t != prev_t || h != prev_h)
    printDataOLED();

  heartBeat();

  prev_t = t;
  prev_h = h;
  
  newData = false;
}

String Reason(void)
{
  if (reason == NONE)
  {
    digitalWrite(HUMIDLED, LOW);
    digitalWrite(TEMPLED, LOW);
    return F("Idle");
  }
  else if (reason == HUMIDITY)
  {
    digitalWrite(HUMIDLED, HIGH);
    return F("Humidity");
  }
  else if (reason == TEMPERATURE)
  {
    digitalWrite(TEMPLED, HIGH);
    return F("Temperature");
  }
  else if (reason == REMOTE_ON)
    return F("Remote on");
  else if (reason == REMOTE_OFF)
    return F("Remote off");
  else if (reason == REMOTE_DELAY)
    return F("Remote delay");
  else
    return F("Idle");
}

void printDataOLED(void)
{
  displayInit(false);
  display.setCursor(75, 0);
  display.print(t);
  display.print(F(" C"));
  display.setCursor(57, 8);
  display.print(h);
  display.print(F(" %"));
  display.setCursor(69, 16);
  display.print(hic);
  display.print(F(" C")); 
  display.display();
}

void displayInit(bool startup)
{
  display.clearDisplay();
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(F("Temperature: "));
  display.println(F("Humidity: "));
  display.println(F("Heat Index: "));
  display.println(F("Mode:   On  Off  Auto"));

  if (startup)
  {
    if (reason == REMOTE_ON)
      pointerPosition = ON;
    else if (reason == REMOTE_OFF)
      pointerPosition = OFF;
    else
      pointerPosition = AUTO;
  }
  placePointer(pointerPosition);

  display.display();
  delay(1);
}

void placePointer(uint8_t pos)
{  
  
   if (pos == ON)
    display.setCursor(38, 24);

  else if (pos == OFF)
    display.setCursor(64, 24);

  else // pos == AUTO
    display.setCursor(95, 24);
    
  display.print(F("*"));
}

void heartBeat(void)
{
  digitalWrite(HEARTBEATLED, HIGH);
  delay(50);
  digitalWrite(HEARTBEATLED, LOW);
}
