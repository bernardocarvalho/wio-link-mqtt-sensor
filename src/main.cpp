/*
 * vim: syntax=cpp ts=4 sw=4 sts=4 sr et
 * Use: SyntasticToggleMode
 *
 *  * @file
 *******************************************************************************
 * Copyright (c) 2021 by M5Stack
 *                  Equipped with M5StickC sample source code
 *
 * Describe:  NTP TIME.
 * Date: 2021/8/3
 * https://docs.m5stack.com/en/core/m5stickc
 * https://docs.m5stack.com/en/unit/watering
 * https://github.com/m5stack/M5Stack/blob/master/examples/Unit/WATERING/WATERING.ino
 * https://console.hivemq.cloud/clients/arduino-esp8266?uuid=e6c3a2b784ad4434b0238f17f98eac34
 * https://github.com/m5stack/M5StickC/blob/master/examples/Advanced/MQTT/MQTT.ino;
  https://github.com/arduino-libraries/ArduinoMqttClient
 ******************************************************************************
 */

#include <Arduino.h>

#include <EEPROM.h>
//;#include <WiFi.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

//#include <NTPClient.h>
//#include <TZ.h>
//#include <FS.h>

#include "time.h"

#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
//Auto water/ day
int addr = 0;  // EEPROM Start number of an ADDRESS.  EEPROM
#define SIZE 32  // define the size of EEPROM(Byte).
//
// Set the name and password of the wifi to be connected.
#ifndef SECRET_SSID
#include "arduino_secrets.h"
#endif

const int WATER_TIME = 15U; // in sec. 
const int RELAY_MTIME = 1000U; // in msec. 
const long AUTO_WATER = 12UL * 3600UL * 1000UL; // Water cycle in ms

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

const char* mqtt_broker  = "test.mosquitto.org";
const int mqtt_port = 1883;
const int msgPeriod = 30 * 1000U;
const int wifiPeriod = 360 * 1000U;

/**  Seeduino Wio specific **/
#define LED_YELLOW 12   // e.g. Yelow LED
//#define LED_OUT 12   // e.g. Yelow LED
//#define SWITCH_YELLOW 13 // PD5
#define SWITCH_IN 13 // PD5 YELLOW
/**  Seeduino Wio specific **/
                      //
#define INPUT_PIN 33 //  SCL
#define PUMP_PIN  14 //  SDA
                     //
#define RELAY_PIN 26
//#define RELAY2_PIN 0

#define NH2O_E_ADD    0x0
#define SUMH2O_E_ADD  0x4
#define CNTH2O_E_ADD  0x8
#define REBOOTS_E_ADD 0xC

#define WIFI_LINE    00
#define MQTT_LINE    10
#define MSG_LINE     20
#define TIME_LINE    30
//
WiFiClient espClient;
MqttClient mqttClient(espClient);

const char willTopic[] = "ipfn/m5stick/will";
const char inTopic[]   = "ipfn/m5stick/in";
const char outTopic[]  = "ipfn/m5stick/out";

const char* ntpServer = "ntp1.tecnico.ulisboa.pt";  // Set the connect NTP server.
const long gmtOffset_sec     = 0;
const int daylightOffset_sec = 0; // 3600;

#define MSG_BUFFER_SIZE 50
char msg[MSG_BUFFER_SIZE];

#define WIFI_RETRY 30
bool wifiOK = false;
bool ntpOK = false;

bool led_blink = false;
bool led_state = false;

int rawADC;
unsigned int sumWater;

unsigned long stopPump = 0, stopRelay = 0;
unsigned long nextMsg = 0;
unsigned long nextWifiCheck, nextWater;

int setupMqtt();
unsigned long read_long_eeprom(unsigned int addr);
void write_long_eeprom(unsigned int addr, unsigned long val);
/*
void printLocalTime() {  // Output current time.
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {  // Return 1 when the time is successfully
//Auto water/ day
        // obtained.
        return;
    }
    Serial.println(&timeinfo,
            "%A, %B %d %Y %H:%M:%S");  //prints date and time.
    //strftime(msg, MSG_BUFFER_SIZE, "%A %B %d %Y %H:%M:%S", &timeinfo);
    // ISO 8601 UTC
    strftime(msg, MSG_BUFFER_SIZE, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);


}
*/

void reconnectWifi() {
    if (WiFi.status() == WL_CONNECTED)
        if (mqttClient.connected() == 1)
            return;
        else {
            mqttClient.unsubscribe(inTopic);
            Serial.print("No mqttClient, ");
        }
    {
        Serial.print(" No link. Wifi "); Serial.print(WiFi.status());
    }
    Serial.print(" MQTT "); Serial.println(mqttClient.connected());
    mqttClient.unsubscribe(inTopic);
    mqttClient.flush();
    mqttClient.stop();
    delay(3000); 

    read_long_eeprom(SUMH2O_E_ADD);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    wifiOK = false;

    WiFi.begin(ssid, password);
    led_state = false;
    led_blink = false;

    for (int i = 0; i < WIFI_RETRY; i++){
        if (WiFi.status() != WL_CONNECTED) {
            Serial.print(".");
            led_state = not led_state;
            digitalWrite(LED_BUILTIN, led_state);
            delay(1000);
            // continue;
        }
        else {
            led_blink = true;

            Serial.println("");
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
            if(setupMqtt() == 0) {
                wifiOK = true;
                //wifiRetries = 0;
                break;
            }
            else {
                //WiFi.disconnect();
                delay(2000);
            }
                //
        }
    }
    //randomSeed(micros());

}

void write_long_eeprom(unsigned int addr, unsigned long val) {
    unsigned int uval;
    for (int i = 0; i < 4; i++) {
        uval = (unsigned int) ( 0xFF & val);
        //EEPROM.write(addr + i, uval);
        val /= 256;
    }
}
unsigned long read_long_eeprom(unsigned int addr) {
    unsigned int uval;
    unsigned long val = 0;
    for (int i = 3; i >= 0; i--) {
        val = val << 4;
//        uval = EEPROM.read(addr + i);
        Serial.println(uval,HEX);
        val |= (uval & 0xFF);
    }
    return val;
}
void onMqttMessage(int messageSize) {
    StaticJsonDocument<256> doc;
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', duplicate = ");
    Serial.print(mqttClient.messageDup() ? "true" : "false");
    Serial.print(", QoS = ");
    Serial.print(mqttClient.messageQoS());
    Serial.print(", retained = ");
    Serial.print(mqttClient.messageRetain() ? "true" : "false");
    Serial.print("', length ");
    Serial.print(messageSize);
//    nextWater = read_long_eeprom(NH2O_E_ADD);
    Serial.println(" bytes:");

    /* use the Stream interface to print the contents
    //
    while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
    }
    */
    deserializeJson(doc, mqttClient);

    Serial.print(", Pump: ");
    int pump = doc["pump"];
    unsigned long now = millis();
    if (pump == 1){
        nextWater = now + 3000UL;
    }
    Serial.print(pump);
    Serial.println();

    int save = doc["save"];

    if (save == 1) {
        read_long_eeprom(SUMH2O_E_ADD);
        read_long_eeprom(NH2O_E_ADD);
        write_long_eeprom(SUMH2O_E_ADD, sumWater);
        write_long_eeprom(NH2O_E_ADD, nextWater);
        Serial.print("\"save\":");
        Serial.println(save);
    }

    int clear = doc["clear"];
    if (clear == 1) {
        Serial.println("Clear");
        read_long_eeprom(SUMH2O_E_ADD);
        write_long_eeprom(SUMH2O_E_ADD, 11);
        sumWater = read_long_eeprom(SUMH2O_E_ADD);
        Serial.print("\"clear\":");
        Serial.println(sumWater);
    }

    nextMsg = now + 1000UL;
}
int setupMqtt() {
    String willPayload = "oh no!";
    bool willRetain = true;
    int willQos = 1;

    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();
    if (!mqttClient.connect(mqtt_broker, mqtt_port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        return -1;
        //while (1);
    }

    Serial.println("You're connected to the MQTT broker!");

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(inTopic);
    Serial.println();

    // subscribe to a topic
    // the second parameter sets the QoS of the subscription,
    // the library supports subscribing at QoS 0, 1, or 2
    // QoS 1  guarantees that the message will be transferred successfully to the broker.
    int subscribeQos = 1;

    int rc=0;
    mqttClient.unsubscribe(inTopic);
    mqttClient.subscribe(inTopic, subscribeQos);
    Serial.print(inTopic);
    Serial.println(" ...Subscribed");
    /*
    for (int i = 0; i < WIFI_RETRY; i++){
        rc = mqttClient.subscribe(inTopic, subscribeQos);
        if(rc==0) {
            M5.Lcd.printf("Subscribed");
            Serial.print(inTopic);
            Serial.println(" ...Subscribed");
            break;
        }
        else {
            mqttClient.unsubscribe(inTopic);
            M5.Lcd.printf("Not Subscribed");

            Serial.println("...Not Subscribed");
            delay(1000);
        }
    }
*/
    return rc;


    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(inTopic);

}
void setup() {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
//    pinMode(RELAY2_PIN, OUTPUT);
//    digitalWrite(RELAY2_PIN, HIGH);
    pinMode(INPUT_PIN, INPUT);
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    //delay(2000);
    //while(!Serial);
    Serial.println("\nHello!");
    /*
    if (!EEPROM.begin(SIZE)) {  // Request storage of SIZE size(success return
        delay(1000000);
        Serial.println("\nFailed to initialise EEPROM!");
        //     delay(1000000);
    }
    */
    delay(5000);
    Serial.println("\nHello!");
    /*
    //Serial.println("\n\nPress BtnA to Write EEPROM");
    //write_long_eeprom(NH2O_E_ADD, 0UL);
    EEPROM.write(NH2O_E_ADD, 0);
    EEPROM.write(NH2O_E_ADD + 1, 0);
    EEPROM.write(NH2O_E_ADD + 2, 0);
    EEPROM.write(NH2O_E_ADD + 3, 0);
    */
    delay(5000);
    sumWater = read_long_eeprom(SUMH2O_E_ADD);
    //nextWater = read_long_eeprom(NH2O_E_ADD);
    //sumWater = 0;
    //reboo = read_long_eeprom(REBOOTS_E_ADD);

    reconnectWifi();

    nextWifiCheck = millis() + 3600000UL;

    configTime(gmtOffset_sec, daylightOffset_sec,
            ntpServer, "pool.ntp.org", "time.nist.gov");  // init and get the time .
    ntpOK = true;
    struct tm ntpTime;
    //M5.Rtc.GetTime(&RTC_TimeStruct);  // Gets the time in the real-time clock.
    Serial.println(msg);

    // printLocalTime();
    //WiFi.mode(WIFI_OFF);  // Set the wifi mode to off.
    unsigned long now = millis();
    nextWater =  now + AUTO_WATER;
    //24UL * 3600UL * 1000UL; // start next day
    delay(200);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
}
void loop() {
    static  unsigned long lastLed = 0;
    const int ledPeriod = 2 * 1000U;

    //digitalWrite(RELAY2_PIN, HIGH);
    StaticJsonDocument<256> doc;
/*
    if (wifiOK && (WiFi.status() != WL_CONNECTED)){
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("Lost Conn to %s. 123456", ssid);
        wifiOK = false;
        led_blink = false;
        write_long_eeprom(NH2O_E_ADD, nextWater);
        write_long_eeprom(SUMH2O_E_ADD, sumWater);
    }
    if (M5.BtnB.wasReleasefor( 700)) {  // The button B is pressed for 700ms. 
        //M5.Lcd.fillScreen( BLACK);  // Set BLACK to the background color.  
        M5.Lcd.setCursor(0, MSG_LINE);
        Serial.println("Reseting eeprom");
        sumWater = 11;
        //write_long_eeprom(SUMH2O_E_ADD, sumWater);
        write_long_eeprom(NH2O_E_ADD, 0);
        EEPROM.write(SUMH2O_E_ADD, 11);
        EEPROM.write(SUMH2O_E_ADD + 1, 0);
        EEPROM.write(SUMH2O_E_ADD + 2, 0);
        EEPROM.write(SUMH2O_E_ADD + 3, 0);
    }

*/
    unsigned long now = millis();
    if (now  > nextWifiCheck) {
        nextWifiCheck = now + wifiPeriod;
        reconnectWifi();
    }

    if (now > nextWater){
        nextWater =  now + AUTO_WATER;
        //nextWater =  now + 24UL * 3600UL * 1000UL; // repeat next day
        stopPump = now + WATER_TIME * 1000UL;
        stopRelay = now + RELAY_MTIME;
        sumWater += WATER_TIME;
        led_state = true;
        // write_long_eeprom(SUMH2O_E_ADD, sumWater);
        // write_long_eeprom(NH2O_E_ADD, nextWater);
        digitalWrite(LED_BUILTIN, led_state);
        digitalWrite(PUMP_PIN, HIGH);
        digitalWrite(RELAY_PIN, HIGH);
    }
    if (now > stopPump){
        digitalWrite(PUMP_PIN, LOW);
        led_state = false;
        digitalWrite(LED_BUILTIN, led_state);
    }
    if (now > stopRelay){
        digitalWrite(RELAY_PIN, LOW);
    }
    //
    mqttClient.poll();
    now = millis();

    if(led_blink)
        if (now - lastLed > ledPeriod) {
            lastLed = now;
            led_state = not led_state;
            digitalWrite(LED_BUILTIN, led_state);
        }

    if (now > nextMsg ) {
        nextMsg = now + msgPeriod;
        //M5.Lcd.setCursor(0, 40);

        Serial.print(" Wifi: ");
        rawADC = analogRead(INPUT_PIN);
        //Serial.print(now/1000);
        snprintf(msg, MSG_BUFFER_SIZE, "%u, Humid ADC: %u, SumW: %lu", now/1000, rawADC, sumWater);
        Serial.print(msg);
        Serial.print(" Wifi: ");
        Serial.println(wifiOK);
        //M5.Lcd.setCursor(0, 25);  // Set cursor to (0,25).

        //printLocalTime(); // Time string will be on msg
        doc["time"]  = msg;
        doc["Humid"] = rawADC;
        doc["sumWater"] = sumWater;
        bool retained = false;

        int publishQos = 1;
        bool dup = false;

        mqttClient.beginMessage(outTopic,  (unsigned long) measureJson(doc), retained, publishQos, dup);
        serializeJson(doc, mqttClient);
        mqttClient.endMessage();
    }
}

// vim: syntax=cpp ts=4 sw=4 sts=4 sr et

/*
#define SERIAL_PRINTF_MAX_BUFF      256
void serialPrintf(const char *fmt, ...);
void serialPrintf(const char *fmt, ...) {
// Buffer for storing the formatted data
char buff[SERIAL_PRINTF_MAX_BUFF];
// pointer to the variable arguments list
va_list pargs;
// Initialise pargs to point to the first optional argument
va_start(pargs, fmt);
// create the formatted data and store in buff
vsnprintf(buff, SERIAL_PRINTF_MAX_BUFF, fmt, pargs);
va_end(pargs);
Serial.print(buff);
}
*/

/*
 *
 display.printf("Bat:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetBatVoltage(),
 M5.Axp.GetBatCurrent());
 display.printf("USB:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetVBusVoltage(),
 M5.Axp.GetVBusCurrent());
 display.printf("5V-In:\r\n  V: %.3fv  I: %.3fma\r\n",
 M5.Axp.GetVinVoltage(), M5.Axp.GetVinCurrent());
 display.printf("Bat power %.3fmw", M5.Axp.GetBatPower());
 */
