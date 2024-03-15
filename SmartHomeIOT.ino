
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <ESP32Servo.h>
#include <SimpleTimer.h>
#include <LiquidCrystal_I2C.h>
#include "IRremote.hpp"


const char *service_name = "PROV_1234RS8";
const char *pop = "12345RS8";


char nodeName[] = "Smart Home";


#define ADC_VREF_mV 3300.0  // in millivolt
#define ADC_RESOLUTION 4096.0



const String password = "7890";  // change your password here
String input_password;

uint16_t iRData[24] = {
  0x3, 0x2, 0x1,
  0x0, 0x7, 0x6,
  0x5, 0x4, 0xB,
  0xA, 0x9, 0x8,
  0xF, 0xE, 0xD,
  0xC, 0x13, 0x12,
  0x11, 0x10, 0x17,
  0x16, 0x15, 0x14
};



// static uint8_t IR_RECV_PIN = 14;  // D35 (IR receiver pin)



// IRrecv irrecv(IR_RECV_PIN);
// decode_results results;


// define the Device Names
char deviceName_1[] = "Room1";
char deviceName_2[] = "Room2";
char deviceName_3[] = "Room3";
char deviceName_4[] = "Room4";
char deviceName_5[] = "Door";
char deviceName_7[] = "Out light";
char deviceName_8[] = "Pool";
// define the GPIO connected with Relays and switches
static uint8_t RelayPin1 = 18;  //D23
static uint8_t RelayPin2 = 17;  //D22
static uint8_t RelayPin3 = 16;  //D21
static uint8_t RelayPin4 = 32;
static uint8_t OutLight = 4;  //D19
static uint8_t RightDoor = 27;
static uint8_t LeftDoor = 26;
static uint8_t Pool = 2;
byte traffic[] = { 19, 23, 33 };
Servo rightDoorMotor;
Servo leftDoorMotor;



static uint8_t LDR_PIN = 34;
static uint8_t TEMP_PIN = 35;
static uint8_t RAIN_PIN = 36;
static uint8_t FLAME_PIN = 13;


bool toggleState_1 = LOW;  //Define integer to remember the toggle state for relay 1
bool toggleState_2 = LOW;  //Define integer to remember the toggle state for relay 2
bool toggleState_3 = LOW;  //Define integer to remember the toggle state for relay 3
bool toggleState_4 = LOW;
bool OutLightState = LOW;
bool doorState = LOW;
bool rainState = LOW;
bool ldrState = LOW;
bool flameState = LOW;
bool poolState = LOW;
uint16_t lcdState = 0x8;

SimpleTimer Timer;

float tempC = 0;
LiquidCrystal_I2C lcd_i2c(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 rows



static Switch my_switch1(deviceName_1, &RelayPin1);
static Switch my_switch2(deviceName_2, &RelayPin2);
static Switch my_switch3(deviceName_3, &RelayPin3);
static Switch my_switch4(deviceName_4, &RelayPin4);
static Switch my_switch5(deviceName_7, &OutLight);
static Switch my_switch8(deviceName_8, &Pool);
static Switch my_door(deviceName_5, &RightDoor);
static TemperatureSensor temperature("Temperature");






void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
#else
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("\nConnected to Wi-Fi!\n");
      break;
  }
}



void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if (strcmp(device_name, deviceName_1) == 0) {

    Serial.printf("Lightbulb = %s\n", val.val.b ? "true" : "false");

    if (strcmp(param_name, "Power") == 0) {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      toggleState_1 = val.val.b;
      (toggleState_1 == false) ? digitalWrite(RelayPin1, HIGH) : digitalWrite(RelayPin1, LOW);
      param->updateAndReport(val);
    }

  } else if (strcmp(device_name, deviceName_2) == 0) {

    Serial.printf("Switch value = %s\n", val.val.b ? "true" : "false");

    if (strcmp(param_name, "Power") == 0) {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      toggleState_2 = val.val.b;
      (toggleState_2 == false) ? digitalWrite(RelayPin2, HIGH) : digitalWrite(RelayPin2, LOW);
      param->updateAndReport(val);
    }

  } else if (strcmp(device_name, deviceName_3) == 0) {

    Serial.printf("Switch value = %s\n", val.val.b ? "true" : "false");

    if (strcmp(param_name, "Power") == 0) {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      toggleState_3 = val.val.b;
      (toggleState_3 == false) ? digitalWrite(RelayPin3, HIGH) : digitalWrite(RelayPin3, LOW);
      param->updateAndReport(val);
    }

  } else if (strcmp(device_name, deviceName_4) == 0) {

    Serial.printf("Switch value = %s\n", val.val.b ? "true" : "false");

    if (strcmp(param_name, "Power") == 0) {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      toggleState_4 = val.val.b;
      (toggleState_4 == false) ? digitalWrite(RelayPin4, HIGH) : digitalWrite(RelayPin4, LOW);
      param->updateAndReport(val);
    }

  } else if (strcmp(device_name, deviceName_7) == 0) {

    Serial.printf("Switch value = %s\n", val.val.b ? "true" : "false");

    if (strcmp(param_name, "Power") == 0) {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      OutLightState = val.val.b;
      (OutLightState == false) ? digitalWrite(OutLight, HIGH) : digitalWrite(OutLight, LOW);
      param->updateAndReport(val);
    }

  } else if (strcmp(device_name, deviceName_5) == 0) {

    Serial.printf("Switch value = %s\n", val.val.b ? "true" : "false");

    if (strcmp(param_name, "Power") == 0) {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      doorState = val.val.b;
      door();
      param->updateAndReport(val);
    }

  } else if (strcmp(device_name, deviceName_8) == 0) {

    Serial.printf("Switch value = %s\n", val.val.b ? "true" : "false");

    if (strcmp(param_name, "Power") == 0) {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      poolState = val.val.b;
      (poolState == false) ? digitalWrite(Pool, HIGH) : digitalWrite(Pool, LOW);
      param->updateAndReport(val);
    }
  }
}


void door() {
  if (!doorState) {
    digitalWrite(traffic[0], doorState);
    digitalWrite(traffic[1], !doorState);
    digitalWrite(traffic[2], doorState);
    int p = 95;
    for (int pos = 2; pos <= 95; pos += 1) {
      // in steps of 1 degree
      rightDoorMotor.write(pos);
      leftDoorMotor.write(p);
      p--;
      delay(25);  // waits 15ms to reach the position
    }
    doorState = true;
    my_door.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
    digitalWrite(traffic[0], doorState);
    digitalWrite(traffic[1], doorState);
    digitalWrite(traffic[2], !doorState);
  } else {
    digitalWrite(traffic[0], !doorState);
    digitalWrite(traffic[1], doorState);
    digitalWrite(traffic[2], !doorState);
    // rotates from 180 degrees to 0 degrees
    int p = 2;
    for (int pos = 95; pos >= 2; pos -= 1) {
      rightDoorMotor.write(pos);
      leftDoorMotor.write(p);
      p++;
      delay(25);  // waits 15ms to reach the position
    }
    doorState = false;
    my_door.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
    digitalWrite(traffic[0], doorState);
    digitalWrite(traffic[1], !doorState);
    digitalWrite(traffic[2], !doorState);
    
    
  }
}

bool rainCheaker() {
  if (analogRead(RAIN_PIN) && !isnan(analogRead(RAIN_PIN)))
    return true;
  return false;
}
bool flameCheaker() {
  if (analogRead(RAIN_PIN)&& !isnan(analogRead(RAIN_PIN)))
    return true;
  return false;
}
bool ldrCheaker() {
  if (analogRead(LDR_PIN) > 500 && !isnan(analogRead(LDR_PIN)))
    return true;
  return false;
}

void allOff() {
  toggleState_1 = LOW; digitalWrite(RelayPin1, HIGH);   my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_1);//Define integer to remember the toggle state for relay 1
  toggleState_2 = LOW;  //Define integer to remember the toggle state for relay 2
  toggleState_3 = LOW;  //Define integer to remember the toggle state for relay 3
  toggleState_4 = LOW;
  OutLightState = LOW;
  poolState = LOW;
  
  digitalWrite(RelayPin2, LOW);
  digitalWrite(RelayPin3, LOW);
  digitalWrite(RelayPin4, LOW);
  digitalWrite(OutLight, LOW);
  digitalWrite(Pool, LOW);

  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch8.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
}

void allOn() {
  toggleState_1 = HIGH;  digitalWrite(RelayPin1, LOW); my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_1);//Define integer to remember the toggle state for relay 1
  toggleState_2 = HIGH;  //Define integer to remember the toggle state for relay 2
  toggleState_3 = HIGH;  //Define integer to remember the toggle state for relay 3
  toggleState_4 = HIGH;
  OutLightState = HIGH;
  poolState = HIGH;
 
  digitalWrite(RelayPin2, HIGH);
  digitalWrite(RelayPin3, HIGH);
  digitalWrite(RelayPin4, HIGH);
  digitalWrite(OutLight, HIGH);
  digitalWrite(Pool, HIGH);
  
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
  my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
  my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
  my_switch8.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
}


  static uint32_t lastActionTime = 0;
  const uint32_t debounceDelay = 500; 


void lcdViewer() {

  uint16_t hex_string; 
  if (IrReceiver.decode()) {
    hex_string = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
    if (millis() - lastActionTime > debounceDelay && hex_string && hex_string != 0x40) {
      if (hex_string == iRData[10] || hex_string == iRData[11]) {
        lcdState = hex_string;  // Copy hex string to lcdState
        input_password = "";    // Reset input_password
      }
      if (hex_string == iRData[0])
        allOn();
      else if (hex_string == iRData[1])
        allOff();
      else if (hex_string == iRData[4]) {
          digitalWrite(RelayPin1, toggleState_1);
          toggleState_1 = !toggleState_1;
          my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_1);
      } else if (hex_string == iRData[5]) {
        if (!toggleState_2) {
          toggleState_2 = HIGH;
          digitalWrite(RelayPin2, HIGH);
          my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
        } else {
          toggleState_2 = LOW;
          digitalWrite(RelayPin2, LOW);
          my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
        }
      } else if (hex_string == iRData[6]) {
        if (!toggleState_3) {
          toggleState_3 = HIGH;
          digitalWrite(RelayPin3, HIGH);
          my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
        } else {
          toggleState_3 = LOW;
          digitalWrite(RelayPin3, LOW);
          my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
        }
      } else if (hex_string == iRData[7]) {
        if (!toggleState_4) {
          toggleState_4 = HIGH;
          digitalWrite(RelayPin4, HIGH);
          my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
        } else {
          toggleState_4 = LOW;
          digitalWrite(RelayPin4, LOW);
          my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
        }
      } else if (hex_string == iRData[8]) {
        if (!OutLightState) {
          OutLightState = HIGH;
          digitalWrite(OutLight, HIGH);
          my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
        } else {
          OutLightState = LOW;
          digitalWrite(OutLight, LOW);
          my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
        }
      } else if (hex_string == iRData[9]) {
        if (!poolState) {
          poolState = HIGH;
          digitalWrite(Pool, HIGH);
          my_switch8.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, true);
        } else {
          poolState = LOW;
          digitalWrite(Pool, LOW);
          my_switch8.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
        }
      }

      if (lcdState == iRData[11]) {
        lcd_i2c.clear();          // clear display
        lcd_i2c.setCursor(0, 0);  // move cursor to   (0, 0)
        lcd_i2c.print("Password : ");
        if (hex_string == iRData[12]) {
          input_password = "";  // clear input password
        } else if (hex_string == iRData[16]) {
          if (password == input_password) {
            Serial.println("The password is correct, ACCESS GRANTED!");
            lcd_i2c.setCursor(0, 1);  // move cursor to   (0, 0)
            lcd_i2c.print("ACCESS GRANTED!");
            door();
          } else {
            Serial.println("The password is incorrect, ACCESS DENIED!");
            lcd_i2c.setCursor(0, 1);  // move cursor to   (0, 0)
            lcd_i2c.print("ACCESS DENIED!");
          }

          input_password = "";  // clear input password
        } else {
          if (hex_string == iRData[15])
            input_password += "1";  // append new character to input password string
          else if (hex_string == iRData[14])
            input_password += "2";
          else if (hex_string == iRData[13])
            input_password += "3";
          else if (hex_string == iRData[19])
            input_password += "4";
          else if (hex_string == iRData[18])
            input_password += "5";
          else if (hex_string == iRData[17])
            input_password += "6";
          else if (hex_string == iRData[23])
            input_password += "7";
          else if (hex_string == iRData[22])
            input_password += "8";
          else if (hex_string == iRData[21])
            input_password += "9";
          else if (hex_string == iRData[20])
            input_password += "0";
        }

        lcd_i2c.setCursor(11, 0);
        lcd_i2c.print(input_password);
      }
      lastActionTime = millis();
    }
  }
  if (lcdState == iRData[10] && Timer.isReady()) {
      sendSensor();
      Timer.reset();  // Reset a second timer
  }
}

void readSensor() {
  flameState = flameCheaker();
  rainState = rainCheaker();
  ldrState = ldrCheaker();
  int adcVal = analogRead(TEMP_PIN);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  tempC = milliVolt / 10;
}

void sendSensor() {
  readSensor();
  temperature.updateAndReportParam("Temperature", tempC);
  lcd_i2c.clear();          // clear display
  lcd_i2c.setCursor(0, 0);  // move cursor to   (0, 0)
  lcd_i2c.print("TEMP ");
  lcd_i2c.print(tempC);
  if (rainState) {
    esp_rmaker_raise_alert("Rain");
    lcd_i2c.setCursor(9, 0);  // move cursor to   (0, 0)
    lcd_i2c.print("RAIN ☁");
  } else {
    lcd_i2c.setCursor(9, 0);  // move cursor to   (0, 0)
    lcd_i2c.print("RAIN ☀");
  }
  if (ldrState) {
    esp_rmaker_raise_alert("LDR");
    lcd_i2c.setCursor(0, 1);  // move cursor to   (0, 0)
    lcd_i2c.print("LDR ☀");
    my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
    digitalWrite(OutLight, LOW);
  } else {
    lcd_i2c.setCursor(0, 1);  // move cursor to   (0, 0)
    lcd_i2c.print(" LDR ☽");
    my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
    digitalWrite(OutLight, HIGH);
  }
  if (flameState) {
    esp_rmaker_raise_alert("Flame");
    lcd_i2c.setCursor(8, 1);  // move cursor to   (0, 0)
    lcd_i2c.print("Flame 🔥");
  } else {
    lcd_i2c.setCursor(8, 1);  // move cursor to   (0, 0)
    lcd_i2c.print("Flame ✗");
  }
}

void setup() {

  Serial.begin(115200);
  lcd_i2c.init();
  lcd_i2c.backlight();

  IrReceiver.begin(14);
  // Set the Relays GPIOs as output mode
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin4, OUTPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(OutLight, OUTPUT);
  pinMode(Pool, OUTPUT);
  pinMode(traffic[0], OUTPUT);
  pinMode(traffic[1], OUTPUT);
  pinMode(traffic[2], OUTPUT);
  rightDoorMotor.attach(RightDoor);
  leftDoorMotor.attach(LeftDoor);


  // Write to the GPIOs the default state on booting
  digitalWrite(RelayPin1, !toggleState_1);
  digitalWrite(RelayPin2, toggleState_2);
  digitalWrite(RelayPin3, toggleState_3);
  digitalWrite(RelayPin4, toggleState_4);
  digitalWrite(OutLight, OutLightState);
  digitalWrite(Pool, poolState);
  digitalWrite(traffic[0], !doorState);
  digitalWrite(traffic[1], doorState);
  digitalWrite(traffic[2], doorState);

  Node my_node;
  my_node = RMaker.initNode(nodeName);

  //Standard switch device
  my_switch1.addCb(write_callback);
  my_switch2.addCb(write_callback);
  my_switch3.addCb(write_callback);
  my_switch4.addCb(write_callback);
  my_switch5.addCb(write_callback);
  my_switch8.addCb(write_callback);
  my_door.addCb(write_callback);

  //Add switch device to the node
  my_node.addDevice(my_switch1);
  my_node.addDevice(my_switch2);
  my_node.addDevice(my_switch3);
  my_node.addDevice(my_switch4);
  my_node.addDevice(my_switch5);
  my_node.addDevice(my_switch8);
  my_node.addDevice(temperature);
  my_node.addDevice(my_door);


  Timer.setInterval(2000);
  //This is optional
  RMaker.enableOTA(OTA_USING_PARAMS);
  //If you want to enable scheduling, set time zone for your region using setTimeZone().
  //The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html
  // RMaker.setTimeZone("Asia/Shanghai");
  // Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
  RMaker.enableTZService();
  RMaker.enableSchedule();

  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch5.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_switch8.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  my_door.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
}


void loop() {

  lcdViewer();
}
