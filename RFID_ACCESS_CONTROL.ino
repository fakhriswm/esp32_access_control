#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <PubSubClient.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define PROXIMITY      1.5 //in meter
#define FILTER_ID      "cb"
#define NUMBEROFBEACON 100
#define DETECT_COUNTER 3
#define UPDATE_MILLIS  500
#define BLE_TIMEOUT    5000

#define GATE_ID "/gate1"
#define MASTER_ACCESS "eyrodigitallabs"

#define POS_UP 90
#define POS_DOWN 0

// Arduino pin number where the button is connected.
#define BUTTON_PIN 4
#define RST_PIN    15          // Configurable, see typical pin layout above
#define SS_PIN     21         // Configurable, see typical pin layout above
#define LED0       2
#define LED1       5
#define BUZZER     22   
#define MASTER_CARD     "EB693E5B"

#define LED_ON(LED)    digitalWrite(LED, HIGH)
#define LED_OFF(LED)   digitalWrite(LED, LOW)

#define  BUZZER_ON()   digitalWrite(BUZZER, HIGH)
#define  BUZZER_OFF()  digitalWrite(BUZZER, LOW)

int scanTime = 1; //In seconds
unsigned long current_millis = 0;
unsigned long previous_millis = 0;

const char* ssid = "CUBE";
const char* password = "123456789";
const char* mqtt_server = "test.mosquitto.org";
const char* topicOut = "barrier_status/gate1";
const char* subscribeTopic = "barrier_command/#";

WiFiClient espClient;
PubSubClient client(espClient);

static const int servoPin = 14;
uint16_t pos;

Servo servo1;
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

int ledState = LOW;             // ledState used to set the LED

int duration = 2000;

int block=2;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;

BLEScan* pBLEScan;

typedef struct BLE {
  int minor;
  int rss;
  int counter;
  unsigned long time;
  bool flag_detect;
}BLE_t;

BLE_t b[NUMBEROFBEACON];

double calculateDistance(double rssi);

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      String newcome = advertisedDevice.toString().c_str();
      Serial.println(newcome);
      String stringOne = newcome.substring(newcome.lastIndexOf(','));
      String id = stringOne.substring(29,31);
      if(id == FILTER_ID){
          String major_hex = stringOne.substring(61,65);
          String minor_hex = stringOne.substring(65,69);
          int major = (int)strtol(major_hex.c_str(),NULL,16);
          int minor = (int)strtol(minor_hex.c_str(),NULL,16);
          double rssi = advertisedDevice.getRSSI()*-1.0;
          double distance = calculateDistance(rssi);

          if(distance <= PROXIMITY){
            for(int i=0; i<NUMBEROFBEACON; i++){
              if(minor == b[i].minor){
                b[i].time = millis();
                b[i].rss = rssi;
                b[i].counter += 1;
                Serial.print("Existing :"); Serial.print(b[i].minor); Serial.print("-"); Serial.print(b[i].counter); Serial.print("-"); Serial.println(b[i].time);
                return;
              }
            }
            for(int j=0; j<NUMBEROFBEACON; j++){
              if(b[j].minor == NULL){
                b[j].minor = minor;
                b[j].time = millis();
                b[j].rss = rssi;
                b[j].counter += 1;
                Serial.print("append success ->"); Serial.println(b[j].minor);
                return;
            }
          }
        }
      }
      
      else{
        return;
      }
    }
};

double calculateDistance(double rssi) {
    float txPower = -59.0;
    if (rssi == 0) {
        return -1.0; // if we cannot determine distance, return -1.
    }
    double ratio = rssi * 1.0 / txPower;

    if (ratio < 1.0) {
        return pow(ratio, 10);
    } else {
        double accuracy = (0.89976) * pow(ratio, 7.7095) + 0.111;
        return accuracy;
    }
}

void check_beacon(){
  if(current_millis - previous_millis >= UPDATE_MILLIS){
    previous_millis = current_millis;
    
    for(int j=0; j<NUMBEROFBEACON; j++){
     if(current_millis-b[j].time>BLE_TIMEOUT && b[j].minor!=NULL){
        Serial.print(b[j].minor); Serial.print("-"); Serial.println("out");
        b[j].minor = 0;
        b[j].rss = 0;
        b[j].time = 0;
        b[j].counter = 0;
        b[j].flag_detect = false;
     }
     else if( b[j].minor!=NULL){
        Serial.print(b[j].minor); Serial.print(" inrange with rssi = "); Serial.println(b[j].rss);
     }
    }

    for(int j=0; j<NUMBEROFBEACON; j++)
    {
      if(b[j].counter == DETECT_COUNTER && b[j].flag_detect == false){
          b[j].flag_detect = true;
          LED_ON(LED0);
          delay(2000);
          LED_OFF(LED0);
      }
    }
  }
}

void ble_init(){
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void buzzer(uint16_t t_on, uint16_t t_off, uint8_t count){
  for (uint8_t i = 0; i < count; i++)
  {
    digitalWrite(BUZZER, HIGH);
    delay(t_on);
    digitalWrite(BUZZER, LOW);
    delay(t_off);
  }
}

void servo_open(){
  LED_ON(LED0);
  servo1.write(POS_UP);
  delay(100);
}

void servo_close(){
  servo1.write(POS_DOWN);
  delay(100);
  LED_OFF(LED0);
}

String read_rfid(){
  String content = "";
  byte letter;

  if (  mfrc522.PICC_IsNewCardPresent())
  {
    if (  mfrc522.PICC_ReadCardSerial())
    {
      for (byte i = 0; i < mfrc522.uid.size; i++)
      {
        content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : ""));
        content.concat(String(mfrc522.uid.uidByte[i], HEX));
      }
      
      content.toUpperCase();    
      
      LED_ON(LED0);
      BUZZER_ON();
      delay(500);
      
      LED_OFF(LED0);
      BUZZER_OFF();

    }
  }
  return content;
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("barrierclient")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(subscribeTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len){
  String topic_s = topic;
  String subtopic = topic_s.substring(15);
  
  String payload_s = (char*)payload;
  payload_s = payload_s.substring(0,len);

  Serial.println("receive message");

  if(subtopic == GATE_ID){
    if(payload_s == "open"){
      Serial.println("gate open");
      servo_open();
      delay(5000);
      servo_close();
    }
   if(payload_s=="close"){
      Serial.println("gate close");
      servo_close();
    }
  }
}

int readBlock(int blockNumber, byte arrayAddress[]){
  int largestModulo4Number=blockNumber/4*4;
  int trailerBlock=largestModulo4Number+3;//determine trailer block for the sector
  
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
  
  MFRC522::StatusCode status;
  //authentication of the desired block for access
  status = (MFRC522::StatusCode)mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
         Serial.print("PCD_Authenticate() failed (read): ");
         Serial.println(mfrc522.GetStatusCodeName(status));
         return 3;//return "3" as error message
  }

//reading a block
byte buffersize = 18;//we need to define a variable with the read buffer size, since the MIFARE_Read method below needs a pointer to the variable that contains the size... 
status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(blockNumber, arrayAddress, &buffersize);//&buffersize is a pointer to the buffersize variable; MIFARE_Read requires a pointer instead of just a number
  if (status != MFRC522::STATUS_OK) {
          Serial.print("MIFARE_read() failed: ");
          Serial.println(mfrc522.GetStatusCodeName(status));
          return 4;//return "4" as error message
  }
  Serial.println("block was read");
}

void setup() {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  Serial.begin(115200);		// Initialize serial communications with the PC
  while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
  
  SPI.begin();			// Init SPI bus
  mfrc522.PCD_Init();		// Init MFRC522
  delay(100);				// Optional delay. Some board do need more time after init to be ready, see Readme
  mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
  Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
  
  servo1.attach(servoPin);
  
  ble_init();
  servo_close();
}

void loop() {
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  current_millis = millis();
  if (current_millis - previousMillis >= interval){
    previousMillis = current_millis; 
    
    byte readbackblock[18];
       
    String rfid = read_rfid();
    mfrc522.PCD_StopCrypto1();
    
    if(rfid.length())
    {
      Serial.println("rfid_tag: " + rfid);
      readBlock(block, readbackblock);
      Serial.print("read block: ");
      String key_access = "";
      for (int j=0 ; j<16 ; j++)
      {
        key_access += (char)readbackblock[j];
      }
      Serial.println(key_access);
      Serial.println("");
      mfrc522.PICC_HaltA();
      delay(1000);
      if(key_access == MASTER_ACCESS){
        Serial.println("Access granted!");
        servo_open();
        delay(5000);
        servo_close();
      }
      else{
        Serial.println("Access rejected!");
      }
    }
  }
  
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done!");
  pBLEScan->clearResults();
  current_millis = millis();
  check_beacon();
  
  if(!client.loop()){
    reconnect();
  }
}
