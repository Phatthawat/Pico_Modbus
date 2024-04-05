// Last EDIT : 14/01/2024
#include <Wire.h>
#include <SPI.h>
#include <ModbusEthernet.h>
#include <ModbusSerial.h>
#include <Adafruit_MCP23X17.h>
#include <SoftwareSerial.h>
#include <ModbusRTUSlave.h>

// Ethernet Configuration
byte mac_address[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  
byte ip_address[] = { 192, 168, 3, 120};  
byte subnet[] = { 255, 255, 255, 0};

#define RXPIN 1
#define TXPIN 0
#define btnRST 28

const byte SlaveId = 1;
const unsigned long Baudrate = 9600;

const int ledPin = LED_BUILTIN; 

// GPIO Configuration
const byte X_PIN[] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15,21,20};
const byte Y_PIN[] = {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0};
const int eIO_numOfPin = sizeof(X_PIN)/sizeof(X_PIN[0]);


bool coils[eIO_numOfPin];
bool discreteInputs[eIO_numOfPin];

byte eDISCRETE_INPUTS_Address = 1;
byte eDISCRETE_INPUTS_Array[] = {};

byte eOUTPUT_COILS_Address = 1;
byte eOUTPUT_COILS_Array[] = {};



ModbusEthernet eth_mb;
SoftwareSerial mySerial(RXPIN, TXPIN);
ModbusRTUSlave modbus(mySerial);
Adafruit_MCP23X17 mcp;

void setup() {

  Serial.begin(9600);

  Wire1.setSCL(27);
  Wire1.setSDA(26);
  Wire1.begin();

  if (!mcp.begin_I2C(32,&Wire1)) {
    Serial.println("Error.");
    while (1);
  }

  Ethernet.init(17); // Raspberry Pi Pico with w5500

  IPAddress ip(ip_address);
  IPAddress sn(subnet);
  Ethernet.begin(mac_address, ip, sn);

  Serial.print("Local IP : ");
  Serial.println(Ethernet.localIP());
  Serial.print("Subnet Mask : ");
  Serial.println(Ethernet.subnetMask());

  
  delay(1000);

  for(byte i=0; i<=eIO_numOfPin; i++){
    pinMode(X_PIN[i], INPUT);
    mcp.pinMode(Y_PIN[i], OUTPUT);
    delay(100);
  }

  for(int i=0; i<=eIO_numOfPin; i++){
    eDISCRETE_INPUTS_Array[i] = eDISCRETE_INPUTS_Address++;
    eOUTPUT_COILS_Array[i] = eOUTPUT_COILS_Address++;
    eth_mb.addIsts(eDISCRETE_INPUTS_Array[i]);
    eth_mb.addCoil(eOUTPUT_COILS_Array[i]);
    delay(100);
  }

  pinMode(btnRST, INPUT);
  pinMode(ledPin, OUTPUT);

  modbus.configureCoils(coils, eIO_numOfPin);      
  modbus.configureDiscreteInputs(discreteInputs, eIO_numOfPin);
  modbus.begin(SlaveId, Baudrate); 
  
  // Config Modbus TCP 
  eth_mb.config(mac_address, ip_address);

  digitalWrite(ledPin, 1);

}

void loop() {
  
  eth_mb.task();
  modbus.poll();
  Serial.println("Baudrate");
  for(byte i=0; i<eIO_numOfPin; i++){
    eth_mb.Ists(eDISCRETE_INPUTS_Array[i], !digitalRead(X_PIN[i]));
    mcp.digitalWrite(Y_PIN[i], eth_mb.Coil(eOUTPUT_COILS_Array[i]) | coils[i]);
    discreteInputs[i] = !digitalRead(X_PIN[i]);
  }
}
