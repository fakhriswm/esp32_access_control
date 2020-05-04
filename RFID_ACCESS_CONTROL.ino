#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <EasyButton.h>
#include <Servo.h>
#include <PubSubClient.h>

#define GATE_ID "/gate1"
#define MASTER_ACCESS "eyrodigitallabs"

#define POS_UP 90
#define POS_DOWN 0


// Arduino pin number where the button is connected.
#define BUTTON_PIN 4
#define RST_PIN    15          // Configurable, see typical pin layout above
#define SS_PIN     21         // Configurable, see typical pin layout above
#define LED0       LED_BUILTIN
#define LED1       5
#define BUZZER     22   
#define MASTER_CARD     "EB693E5B"

#define LED_ON(LED)    digitalWrite(LED, HIGH)
#define LED_OFF(LED)   digitalWrite(LED, LOW)

#define  BUZZER_ON()   digitalWrite(BUZZER, HIGH)
#define  BUZZER_OFF()  digitalWrite(BUZZER, LOW)

const char* ssid = "hayo kere ya";
const char* password = "debbycantikbanget";
const char* mqtt_server = "test.mosquitto.org";
const char* topicOut = "barrier_status/gate1";
const char* subscribeTopic = "barrier_command/#";

WiFiClient espClient;
PubSubClient client(espClient);

static const int servoPin = 14;
uint16_t pos;

Servo servo1;
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
MFRC522::MIFARE_Key key;

int ledState = LOW;             // ledState used to set the LED

int duration = 2000;

byte readbackblock[18];
int block=2;

EasyButton button(BUTTON_PIN);

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;


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
  servo1.write(POS_UP);
  delay(100);
}

void servo_close(){
  servo1.write(POS_DOWN);
  delay(100);
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

// Callback.
void onPressedForDuration() {
    Serial.println("Button has been pressed for the given duration!");
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
      client.subscribe("esp32/output");
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

  if(subtopic == GATE_ID){
    if(payload_s == "open"){
      Serial.println("gate open");
      digitalWrite(LED_BUILTIN,HIGH);
      servo1.write(90);
      delay(2000);
      client.publish(topicOut,"1");
    }
   if(payload_s=="close"){
      Serial.println("gate close");
      digitalWrite(LED_BUILTIN,LOW);
      servo1.write(0);
      delay(2000);
      client.publish(topicOut,"0");
    }
  }
}

int readBlock(int blockNumber, byte arrayAddress[]){
  int largestModulo4Number=blockNumber/4*4;
  int trailerBlock=largestModulo4Number+3;//determine trailer block for the sector

  //authentication of the desired block for access
  byte status = (MFRC522::StatusCode)mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));

  if (status != MFRC522::STATUS_OK) {
         Serial.print("PCD_Authenticate() failed (read): ");
         //Serial.println(mfrc522.GetStatusCodeName(status));
         return 3;//return "3" as error message
  }

//reading a block
byte buffersize = 18;//we need to define a variable with the read buffer size, since the MIFARE_Read method below needs a pointer to the variable that contains the size... 
status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(blockNumber, arrayAddress, &buffersize);//&buffersize is a pointer to the buffersize variable; MIFARE_Read requires a pointer instead of just a number
  if (status != MFRC522::STATUS_OK) {
          Serial.print("MIFARE_read() failed: ");
          //Serial.println(mfrc522.GetStatusCodeName(status));
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
  delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
  mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
  Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));

  // Prepare the security key for the read and write functions.
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;  //keyByte is defined in the "MIFARE_Key" 'struct' definition in the .h file of the library
  }
  
  servo1.attach(servoPin);
  button.begin();
  
  // Attach callback.
  button.onPressedFor(duration, onPressedForDuration);
  
  servo_close();
}

void loop() {
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;    
    String rfid = read_rfid();
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
      mfrc522.PCD_StopCrypto1();
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
  button.read();

  if(!client.loop()){
    reconnect();
  }
}
