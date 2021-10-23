// AZ Touch ESP32 MQTT IO Thermostat

// github(@)meewis(.)net

// If you like my work, feel free to use it but it would be nice to donate me some bitcoins for a coffee ;) And if you have done that I won't hasitate to reply the email you have send me at github(@)meewis(.)net
// Coffee donation @ BTC 1HnNLkWEhWw9d9wUohUzfTR68J7FGuGkUj

// License: MIT

// Board:
// ESP32 Dev Module

// ESP:
// NodeMCU ESP32-WROOM-32D

// AZ-Touch MOD with 2.8 inch TFT with touch, version 1.2.0

// Sensor: BMP280

// Include
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "system.h"
#include "button.h"

// Wifi parameters
const char* wifi_ssid = "ssid";
const char* wifi_password = "password";

// MQTT parameters
const char* mqtt_server = "10.10.10.10";
const char* mqtt_id = "AZ_Thermo";
const char* mqtt_login = "";
const char* mqtt_passw = "";
const char* mqtt_topic_ip = "IP";
const char* mqtt_topic_id = "ID";
const char* mqtt_topic_livecheck = "LiveCheck";
const char* mqtt_topic_msgcheck = "MessageCheck";
const char* mqtt_topic_ledstatus = "LedStatus";
const char* mqtt_topic_relaisstatus = "RelaisStatus";
const char* mqtt_topic_data_in = "DataIn";
const char* mqtt_topic_data_out = "DataOut";
const char* mqtt_topic_temp_indoor = "TempIndoor";
const char* mqtt_topic_temp_outdoor = "TempOutdoor";
const char* mqtt_topic_temp_set_in = "TempSetIn";
const char* mqtt_topic_temp_set_out = "TempSetOut";
const char* mqtt_topic_temp_mode = "TempMode";

#if MQTT_MAX_PACKET_SIZE < 1024  // If the max message size is too small, throw an error at compile time.
#error "MQTT_MAX_PACKET_SIZE is too small in libraries/PubSubClient/src/PubSubClient.h
#endif

const int mqtt_topic_len = 40;
const int mqtt_msg_len = MQTT_MAX_PACKET_SIZE - 40;

// Serial paramters
const int serial_msg_len = MQTT_MAX_PACKET_SIZE - 40;

// GPIO
#define IO_BUTTON 0
#define IO_BOARD 0
#define IO_LED 0
#define IO_RELAIS 0
#define IO_BEEPER 21
#define IO_LCD_CS 5
#define IO_LCD_DC 4
#define IO_LCD_CLK 18
#define IO_LCD_RST 22
#define IO_LCD_MISO 19
#define IO_LCD_MOSI 23
#define IO_LCD_LED 15  
#define IO_TOUCH_CS 14
#define IO_TOUCH_IRQ 27 
#define IO_SENSOR_SCK 33
#define IO_SENSOR_MISO 32
#define IO_SENSOR_MOSI 26
#define IO_SENSOR_CS 25
#define DISPLAY_TOUCH_PRESSURE_MIN 10
#define DISPLAY_TOUCH_PRESSURE_MAX 2000
#define DISPLAY_TOUCH_MAXX 3700
#define DISPLAY_TOUCH_MAXY 3600
#define DISPLAY_TOUCH_MINX 370
#define DISPLAY_TOUCH_MINY 470
#define TEMPERATURE_MAX 30.0
#define TEMPERATURE_MIN 5.0
#define TEMPERATURE_DELTA 0.3
enum { PM_PAGE1, PM_PAGE2, PM_PAGE3};  // Program modes
enum { TM_BOOT, TM_HEATING, TM_COOLING, TM_OK};  // Thermostat modes

// Serial debug print
#define DEBUG_PRINT true
#define DEBUG_PRINT_SERIAL \
  if (DEBUG_PRINT) Serial
  
#define DEBUG_WARNING true
#define DEBUG_WARNING_SERIAL \
  if (DEBUG_WARNING) Serial
  
#define DEBUG_ERROR true
#define DEBUG_ERROR_SERIAL \
  if (DEBUG_ERROR) Serial

// Instance
SystemData System_Data;
ButtonData Button_Data;
WiFiClient WIFI_Client;
PubSubClient MQTT_Client(WIFI_Client);
Adafruit_ILI9341 display_lcd = Adafruit_ILI9341(IO_LCD_CS, IO_LCD_DC, IO_LCD_RST);
XPT2046_Touchscreen display_touch(IO_TOUCH_CS, IO_TOUCH_IRQ);
TS_Point display_point;
Adafruit_BMP280 sensor(IO_SENSOR_CS, IO_SENSOR_MOSI, IO_SENSOR_MISO, IO_SENSOR_SCK); // software SPI

// Variabels
char serial_msg_in[serial_msg_len];
int serial_msg_in_count = 0;
bool serial_msg_in_ready = false;
bool serial_msg_in_begin = false;
bool serial_msg_in_end = false;
char serial_msg_out[serial_msg_len];

char wifi_mac[20];
char wifi_ip[20];
int wifi_counter = 0;

char mqtt_topic[mqtt_topic_len];
char mqtt_msg[mqtt_msg_len];
char mqtt_msg_in[mqtt_msg_len];
char mqtt_msg_in_data[mqtt_msg_len];
char mqtt_msg_in_temp_outdoor[mqtt_msg_len];
char mqtt_msg_in_temp_set[mqtt_msg_len];
char mqtt_msg_out[mqtt_msg_len];
bool mqtt_msg_in_flag = false;
bool mqtt_msg_out_flag = false;
int mqtt_counter = 0;

int livecheck = 0;

bool data_led = false;
bool data_led_mem = false;
bool data_relais = false;
bool data_relais_mem = false;
int data_in = 0;
int data_out = 0;
float temp_indoor = 0.0;
float temp_indoor_mem = 0.0;
float temp_outdoor = 0.0;
float temp_outdoor_mem = 0.0;
int temp_outdoor_timer = 100;
int temp_outdoor_timer_mem = 0;
float temp_set = 0.0;
float temp_set_mem = 0.0;
int temp_status = 0;
int temp_mode = 0;
int temp_mode_mem = 0;

int display_mode = 1;
int display_mode_mem = 0;
bool display_button_press = false;
bool display_button_press_mem = false;
bool display_button_puls = false;
int display_button_code = 0;
long display_button_millis;

float sensor_temperature;
float sensor_pressure;
float sensor_altitude;
float sensor_humidity;

// Init
void setup() 
{
  Serial.begin(115200);

  //pinMode(IO_BUTTON, INPUT);
  //pinMode(IO_BOARD, OUTPUT);
  //pinMode(IO_LED, OUTPUT);
  //pinMode(IO_RELAIS, OUTPUT);
  pinMode(IO_LCD_LED, OUTPUT);

  //Button_Data.ButtonInit(IO_BUTTON);

  wifi_init();
  wifi_connect();

  mqtt_init();
  if (WiFi.status() == WL_CONNECTED)
  {
    mqtt_connect();
  }

  display_init();

  beeper_init();

  sensor_init();
}

//  Routines
void wifi_init()
{
  WiFi.disconnect(true); // delete old config
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
}

void wifi_connect() 
{
  DEBUG_PRINT_SERIAL.println("");
  DEBUG_PRINT_SERIAL.print("WiFi connecting to ");
  DEBUG_PRINT_SERIAL.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);  
  delay(1000);

  if (WiFi.status() == WL_CONNECTED) 
  {
    DEBUG_PRINT_SERIAL.println("");
    DEBUG_PRINT_SERIAL.println("WiFi connected");
  } 
  else 
  {
    DEBUG_WARNING_SERIAL.println("");
    DEBUG_WARNING_SERIAL.println("WiFi connection failed");
  }
}

void wifi_reconnect()
{
  DEBUG_PRINT_SERIAL.println("");
  DEBUG_PRINT_SERIAL.print("WiFi reconnecting to ");
  DEBUG_PRINT_SERIAL.println(wifi_ssid);

  WiFi.disconnect();
  //WiFi.reconnect();
  WiFi.begin(wifi_ssid, wifi_password);  
  delay(1000);

  if (WiFi.status() == WL_CONNECTED) 
  {
    DEBUG_PRINT_SERIAL.println("");
    DEBUG_PRINT_SERIAL.println("WiFi reconnected");
  } 
  else 
  {
    DEBUG_WARNING_SERIAL.println("");
    DEBUG_WARNING_SERIAL.println("WiFi reconnection failed");
  }
}

void wifi_info()
{
  sprintf(wifi_mac, "%2X:%2X:%2X:%2X:%2X:%2X", WiFi.macAddress()[5], WiFi.macAddress()[4], WiFi.macAddress()[3], WiFi.macAddress()[2], WiFi.macAddress()[1], WiFi.macAddress()[0]);
  sprintf(wifi_ip, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
}

bool wifi_check()
{
  bool wifi_status = false;

  if (WiFi.status() == WL_CONNECTED)
    wifi_status = true;
  if ((WiFi.localIP()[0] == 0) && (WiFi.localIP()[1] == 0) && (WiFi.localIP()[2] == 0) && (WiFi.localIP()[3] == 0) )
    wifi_status = false;
    
  return(wifi_status);
}

void mqtt_init()
{
  MQTT_Client.setServer(mqtt_server, 1883);
  MQTT_Client.setCallback(mqtt_callback);  
}

void mqtt_connect() 
{
  DEBUG_PRINT_SERIAL.println();
  DEBUG_PRINT_SERIAL.print("MQTT connecting to ");
  DEBUG_PRINT_SERIAL.println(mqtt_server);    

  if (MQTT_Client.connect(mqtt_id, mqtt_login, mqtt_passw)) 
  {
    DEBUG_PRINT_SERIAL.println("");
    DEBUG_PRINT_SERIAL.println("MQTT connected");
    DEBUG_PRINT_SERIAL.print("ID: ");
    DEBUG_PRINT_SERIAL.println(mqtt_id);

    delay(1000);
    
    strcpy(mqtt_topic, mqtt_id);
    strcat(mqtt_topic, "/");
    strcat(mqtt_topic, mqtt_topic_data_in);
    MQTT_Client.subscribe(mqtt_topic);
    MQTT_Client.loop();
    strcpy(mqtt_topic, mqtt_id);
    strcat(mqtt_topic, "/");
    strcat(mqtt_topic, mqtt_topic_temp_outdoor);
    MQTT_Client.subscribe(mqtt_topic);
    MQTT_Client.loop();
    strcpy(mqtt_topic, mqtt_id);
    strcat(mqtt_topic, "/");
    strcat(mqtt_topic, mqtt_topic_temp_set_in);
    MQTT_Client.subscribe(mqtt_topic);
    MQTT_Client.loop();

    strcpy(mqtt_msg_out, mqtt_id);
    MQTT_Client.publish(mqtt_topic_id, mqtt_msg_out);
  } 
  else 
  {
    DEBUG_WARNING_SERIAL.println("");
    DEBUG_WARNING_SERIAL.print("MQTT connection failed, rc=");
    DEBUG_WARNING_SERIAL.println(MQTT_Client.state());
  }
}

void mqtt_reconnect() 
{
  //DEBUG_PRINT_SERIAL.println();
  //DEBUG_PRINT_SERIAL.print("MQTT connecting to ");
  //DEBUG_PRINT_SERIAL.println(mqtt_server);    

  if (MQTT_Client.connect(mqtt_id, mqtt_login, mqtt_passw)) 
  {
    DEBUG_PRINT_SERIAL.println("");
    DEBUG_PRINT_SERIAL.println("MQTT reconnected");
    //DEBUG_PRINT_SERIAL.print("ID: ");
    //DEBUG_PRINT_SERIAL.println(mqtt_id);
    
    strcpy(mqtt_topic, mqtt_id);
    strcat(mqtt_topic, "/");
    strcat(mqtt_topic, mqtt_topic_data_in);
    MQTT_Client.subscribe(mqtt_topic);
    MQTT_Client.loop();
    strcpy(mqtt_topic, mqtt_id);
    strcat(mqtt_topic, "/");
    strcat(mqtt_topic, mqtt_topic_temp_outdoor);
    MQTT_Client.subscribe(mqtt_topic);
    MQTT_Client.loop();
    strcpy(mqtt_topic, mqtt_id);
    strcat(mqtt_topic, "/");
    strcat(mqtt_topic, mqtt_topic_temp_set_in);
    MQTT_Client.subscribe(mqtt_topic);
    MQTT_Client.loop();
    
    //strcpy(mqtt_msg_out, mqtt_id);
    //MQTT_Client.publish(mqtt_topic_id, mqtt_msg_out);
  } 
  else 
  {
    DEBUG_WARNING_SERIAL.println("");
    DEBUG_WARNING_SERIAL.print("MQTT reconnection failed, rc=");
    DEBUG_WARNING_SERIAL.println(MQTT_Client.state());
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
  mqtt_msg_in_flag = true;

  payload[length] = '\0';
  strcpy(mqtt_msg_in, (char *)payload);

  DEBUG_PRINT_SERIAL.println("");
  DEBUG_PRINT_SERIAL.print("MQTT msessage arrived [");
  DEBUG_PRINT_SERIAL.print(topic);
  DEBUG_PRINT_SERIAL.print("][");
  DEBUG_PRINT_SERIAL.print(mqtt_msg_in);
  DEBUG_PRINT_SERIAL.println("]");

  strcpy(mqtt_topic, mqtt_id);
  strcat(mqtt_topic, "/");
  strcat(mqtt_topic, mqtt_topic_data_in);
  if (strcmp(topic, mqtt_topic)==0)
  {
    payload[length] = '\0';
    strcpy(mqtt_msg_in_data, (char *)payload);
  }
  strcpy(mqtt_topic, mqtt_id);
  strcat(mqtt_topic, "/");
  strcat(mqtt_topic, mqtt_topic_temp_outdoor);
  if (strcmp(topic, mqtt_topic)==0)
  {
    payload[length] = '\0';
    strcpy(mqtt_msg_in_temp_outdoor, (char *)payload);
  }
  strcpy(mqtt_topic, mqtt_id);
  strcat(mqtt_topic, "/");
  strcat(mqtt_topic, mqtt_topic_temp_set_in);
  if (strcmp(topic, mqtt_topic)==0)
  {
    payload[length] = '\0';
    strcpy(mqtt_msg_in_temp_set, (char *)payload);
  }
}


void mqtt_publish(const char* topic, char* payload)
{
  mqtt_msg_out_flag = true;
  
  strcpy(mqtt_topic, mqtt_id);
  strcat(mqtt_topic, "/");
  strcat(mqtt_topic, topic);

  strcpy(mqtt_msg_out, (char *)payload);

  MQTT_Client.publish(mqtt_topic, mqtt_msg_out);

  DEBUG_PRINT_SERIAL.println("");
  DEBUG_PRINT_SERIAL.print("MQTT msessage send [");
  DEBUG_PRINT_SERIAL.print(topic);
  DEBUG_PRINT_SERIAL.print("][");
  DEBUG_PRINT_SERIAL.print(mqtt_msg_out);
  DEBUG_PRINT_SERIAL.println("]");
}

void mqtt_info()
{

}

bool mqtt_check()
{
  bool mqtt_status = false;

  if (MQTT_Client.connected())
    mqtt_status = true;
    
  return(mqtt_status);
}

void display_init()
{
  display_lcd.begin();
  display_lcd.setRotation(1);
  DEBUG_PRINT_SERIAL.println("");
  DEBUG_PRINT_SERIAL.print("Display width: ");
  DEBUG_PRINT_SERIAL.println(display_lcd.width());
  DEBUG_PRINT_SERIAL.print("Display height: ");
  DEBUG_PRINT_SERIAL.println(display_lcd.height());
  display_lcd_on();
  display_touch.begin();  
  display_button_millis = millis();
}

void display_update()
{
  //if (System_Data.system_100ms_puls)
    display_page();
    display_button();
}

void display_lcd_on()
{
  digitalWrite(IO_LCD_LED, LOW); // LOW to turn backlight on - pcb version 01-02-00  
}

void display_lcd_off()
{
  digitalWrite(IO_LCD_LED, HIGH); // HIGH to turn backlight off - pcb version 01-02-00  
}

void display_page()
{
  switch (display_mode) 
  {
    case 1:
      display_page1();
      break;
    case 2:
      display_page2();
      break;
    case 3:
      display_page3();
      break;
    default:
      display_mode = 1;
      break;
  }
  display_mode_mem = display_mode; //!!
}

void display_page1()
{
  unsigned char i;

  if ((display_mode != display_mode_mem) || (!(temp_outdoor_timer) && (temp_outdoor_timer_mem)) || ((temp_outdoor_timer) && !(temp_outdoor_timer_mem)))
    display_lcd.fillScreen(ILI9341_BLACK);
  
  if ((display_mode != display_mode_mem) || (temp_mode != temp_mode_mem) || (!(temp_outdoor_timer) && (temp_outdoor_timer_mem)) || ((temp_outdoor_timer) && !(temp_outdoor_timer_mem)))
  {
    //draw big circle 
    if (temp_mode == TM_BOOT) 
      for(i=0; i < 10; i++) display_lcd.drawCircle(120, 110, 85 + i, ILI9341_WHITE);
    else if (temp_mode == TM_HEATING) 
      for(i=0; i < 10; i++) display_lcd.drawCircle(120, 110, 85 + i, ILI9341_RED);
    else if (temp_mode == TM_COOLING) 
      for(i=0; i < 10; i++) display_lcd.drawCircle(120, 110, 85 + i, ILI9341_BLUE);
    else if (temp_mode == TM_OK) 
      for(i=0; i < 10; i++) display_lcd.drawCircle(120, 110, 85 + i, ILI9341_GREEN);
    else
      for(i=0; i < 10; i++) display_lcd.drawCircle(120, 110, 85 + i, ILI9341_BLACK);

    //draw small circle
    display_lcd.fillCircle(65, 180, 45, ILI9341_LIGHTGREY );
    if (temp_outdoor_timer)
      display_lcd.fillCircle(175, 60, 45, ILI9341_DARKGREY );
  
    //draw °C in big circle
    display_lcd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    display_lcd.setFont(&FreeSansBold9pt7b);
    display_lcd.setCursor(140, 100);
    display_lcd.print("o");
    display_lcd.setFont(&FreeSansBold24pt7b);
    display_lcd.setCursor(150, 130);
    display_lcd.print("C");
  
    // draw °C and indoor in small circle
    display_lcd.setTextColor(ILI9341_WHITE, ILI9341_LIGHTGREY );
    display_lcd.setFont(&FreeSansBold12pt7b);
    display_lcd.setCursor(85, 200);
    display_lcd.print("C");
    display_lcd.drawCircle(80,185, 2, ILI9341_WHITE);
    display_lcd.drawCircle(80,185, 3, ILI9341_WHITE);
    display_lcd.setFont(&FreeSansBold9pt7b);
    display_lcd.setCursor(35, 170);
    display_lcd.print("Binnen");

    if (temp_outdoor_timer)
    {
      // draw °C and outdoor in small circle
      display_lcd.setTextColor(ILI9341_WHITE, ILI9341_LIGHTGREY );
      display_lcd.setFont(&FreeSansBold12pt7b);
      display_lcd.setCursor(195, 80);
      display_lcd.print("C");
      display_lcd.drawCircle(190,65, 2, ILI9341_WHITE);
      display_lcd.drawCircle(190,65, 3, ILI9341_WHITE);
      display_lcd.setFont(&FreeSansBold9pt7b);
      display_lcd.setCursor(145, 50);
      display_lcd.print("Buiten");
    }
  }

  if ((display_mode != display_mode_mem) || (temp_indoor != temp_indoor_mem) || (!(temp_outdoor_timer) && (temp_outdoor_timer_mem)) || ((temp_outdoor_timer) && !(temp_outdoor_timer_mem)))
  {
    // draw indoor temperature
    int16_t x1, y1;
    uint16_t w1, h1;
    String cur_value1 = String(temp_indoor, 1);
    int str_len1 =  cur_value1.length() + 1; 
    char char_array1[str_len1];
    cur_value1.toCharArray(char_array1, str_len1);
    display_lcd.fillRect(25, 180, 50, 22, ILI9341_LIGHTGREY );
    display_lcd.setTextColor(ILI9341_WHITE, ILI9341_LIGHTGREY );
    display_lcd.setFont(&FreeSansBold12pt7b);
    display_lcd.getTextBounds(char_array1, 35, 200, &x1, &y1, &w1, &h1);
    display_lcd.setCursor(70 - w1, 200);
    display_lcd.print(char_array1);
  }

  if (((display_mode != display_mode_mem) || (temp_mode != temp_mode_mem) || (temp_outdoor != temp_outdoor_mem)) && (temp_outdoor_timer))
  {
    // draw outdoor temperature
    int16_t x2, y2;
    uint16_t w2, h2;
    String cur_value2 = String(temp_outdoor, 1);
    int str_len2 =  cur_value2.length() + 1; 
    char char_array2[str_len2];
    cur_value2.toCharArray(char_array2, str_len2);
    display_lcd.fillRect(135, 60, 50, 22, ILI9341_DARKGREY);
    display_lcd.setTextColor(ILI9341_WHITE, ILI9341_DARKGREY);
    display_lcd.setFont(&FreeSansBold12pt7b);
    display_lcd.getTextBounds(char_array2, 145, 80, &x2, &y2, &w2, &h2);
    display_lcd.setCursor(180 - w2, 80);
    display_lcd.print(char_array2);
  }

  if ((display_mode != display_mode_mem) || (temp_mode != temp_mode_mem) || (temp_set != temp_set_mem) || (!(temp_outdoor_timer) && (temp_outdoor_timer_mem)) || ((temp_outdoor_timer) && !(temp_outdoor_timer_mem)))
  {
    // draw set temperature
    int16_t x3, y3;
    uint16_t w3, h3;
    String cur_value3 = String(temp_set, 1);
    int str_len3 =  cur_value3.length() + 1; 
    char char_array3[str_len3];
    cur_value3.toCharArray(char_array3, str_len3);
    display_lcd.fillRect(40, 85, 100, 50, ILI9341_BLACK);
    display_lcd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    display_lcd.setFont(&FreeSansBold24pt7b);
    display_lcd.getTextBounds(char_array3, 80, 125, &x3, &y3, &w3, &h3);
    display_lcd.setCursor(130 - w3, 125);
    display_lcd.print(char_array3);
  }

  //button up
  if ((display_mode != display_mode_mem) || (!(temp_outdoor_timer) && (temp_outdoor_timer_mem)) || ((temp_outdoor_timer) && !(temp_outdoor_timer_mem)))
  {
    for(i=0; i < 5; i++) display_lcd.drawCircle(275, 45, 25 + i, ILI9341_WHITE);
    display_lcd.fillTriangle(275,30,290,60,260,60, ILI9341_WHITE);
  }
  
  //button right
  if ((display_mode != display_mode_mem) || (!(temp_outdoor_timer) && (temp_outdoor_timer_mem)) || ((temp_outdoor_timer) && !(temp_outdoor_timer_mem)))
  {
    for(i=0; i < 5; i++) display_lcd.drawCircle(275, 120, 25 + i, ILI9341_LIGHTGREY );
    display_lcd.fillTriangle(260,105,290,120,260,135, ILI9341_LIGHTGREY );
  }
   
  //button down
  if ((display_mode != display_mode_mem) || (!(temp_outdoor_timer) && (temp_outdoor_timer_mem)) || ((temp_outdoor_timer) && !(temp_outdoor_timer_mem)))
  {
    for(i=0; i < 5; i++) display_lcd.drawCircle(275, 195, 25 + i, ILI9341_WHITE);
    display_lcd.fillTriangle(275,210,290,180,260,180, ILI9341_WHITE);
  }
}

void display_page2()
{
  if (display_mode != display_mode_mem)
    display_lcd.fillScreen(ILI9341_BLACK);
}

void display_page3()
{
  if (display_mode != display_mode_mem)
    display_lcd.fillScreen(ILI9341_BLACK);
}

void display_button()
{
  // touch
  display_point = display_touch.getPoint(); 
  delay(1);
  display_point.x = map(display_point.x, DISPLAY_TOUCH_MINX, DISPLAY_TOUCH_MAXX, 320, 0); 
  display_point.y = map(display_point.y, DISPLAY_TOUCH_MINY, DISPLAY_TOUCH_MAXY, 240, 0);
  display_button_press = (display_point.z > DISPLAY_TOUCH_PRESSURE_MIN);
  display_button_puls = (display_button_press && !display_button_press_mem);
  if (display_button_press)
    display_button_press_mem = true;
  else if ((!display_button_press) && (millis() - display_button_millis > 300))
     display_button_press_mem = false;   
  if (display_button_puls)
  {
    display_button_millis = millis();
    
    DEBUG_PRINT_SERIAL.println("");
    DEBUG_PRINT_SERIAL.print("Display touch x:");
    DEBUG_PRINT_SERIAL.print(display_point.x);
    DEBUG_PRINT_SERIAL.print(", y:");
    DEBUG_PRINT_SERIAL.print(display_point.y);
    DEBUG_PRINT_SERIAL.println(".");    
  }  

  if (display_button_press)
  {
    display_button_code = 0;
    switch (display_mode) 
    {
      case 1:
        if ((display_point.x > 245) && (display_point.x < 305) && (display_point.y > 15) && (display_point.y < 75))
          display_button_code = 11; // button up
        if ((display_point.x > 245) && (display_point.x < 305) && (display_point.y > 90) && (display_point.y < 150))
          display_button_code = 12; // button right
        if ((display_point.x > 245) && (display_point.x < 305) && (display_point.y > 165) && (display_point.y < 225))
          display_button_code = 13; // button down
        break;
      case 2:
        ;
        break;
      case 3:
        ;
        break;
    }
  }
}

void beeper_init()
{
  ledcSetup(0, 1E5, 12);  
  ledcAttachPin(IO_BEEPER, 0);
}

void beeper_on()
{
  ledcWriteTone(0, 4000);
}

void beeper_off()
{
  ledcWriteTone(0, 0);
}

void sensor_init()
{
  sensor.begin();
}

void sensor_update()
{
  sensor_temperature = sensor.readTemperature();
  sensor_pressure = sensor.readPressure() / 100.0;
  sensor_altitude = sensor.readAltitude(1013.25); // SEALEVELPRESSURE_HPA
  //sensor_humidity = sensor.readHumidity();
}

// Main
void loop() 
{
  // system update
  System_Data.SystemUpdate();
  
  // system start
  if (System_Data.system_start_puls)
  {
    ;
  }
  
  // wifi
  if (wifi_check())
  {
    wifi_counter = 0;
  }
  else
  {
    if (System_Data.system_1s_puls)
    {
      wifi_counter++;
    }
    if (wifi_counter >= 30)
    {
      wifi_counter = 0;
      wifi_reconnect();
    }
  }
  
  // mqtt
  if (wifi_check())
  {
    if (mqtt_check())
    {
      mqtt_counter = 0;
    }
    else
    {
      if (System_Data.system_1s_puls)
      {
        mqtt_counter++;
      }
      if (mqtt_counter >= 3)
      {
        mqtt_counter = 0;
        mqtt_reconnect();
      }
    }
        
    if (mqtt_check())
    {
      MQTT_Client.loop();
    }
  
    if (mqtt_msg_in_flag)
    {
      mqtt_msg_in_flag = false;
      
      data_in = atoi(mqtt_msg_in);
      if (data_in)
      {
        data_relais = true;    
      }
      else
      {
        data_relais = false;    
      }

      //data_in = atoi(mqtt_msg_in_data);
      temp_outdoor = ((float)atoi(mqtt_msg_in_temp_outdoor)) / 10;
      if (temp_outdoor != 0.0)
        temp_outdoor_timer = 6666;
      temp_set = ((float)atoi(mqtt_msg_in_temp_set)) / 10;
  
      strcpy(mqtt_msg, mqtt_msg_in);
      mqtt_publish(mqtt_topic_msgcheck, mqtt_msg);
    }
  
    if (mqtt_msg_out_flag)
    {
      mqtt_msg_out_flag = false;
    }
  }
 
  if (System_Data.system_1s_puls)
  {
    DEBUG_PRINT_SERIAL.print("*");

    if (wifi_check() && mqtt_check())   
    {
      itoa(data_out, mqtt_msg, 10);
      mqtt_publish(mqtt_topic_data_out, mqtt_msg);

      itoa((int)(temp_indoor * 10), mqtt_msg, 10);
      mqtt_publish(mqtt_topic_temp_indoor, mqtt_msg);
      itoa((int)(temp_set * 10), mqtt_msg, 10);
      mqtt_publish(mqtt_topic_temp_set_out, mqtt_msg);
    }
  }

  if (System_Data.system_10s_puls)
  {
    DEBUG_PRINT_SERIAL.print("#");

    if (wifi_check() && mqtt_check())
    {
    itoa(livecheck++, mqtt_msg, 10);
    mqtt_publish(mqtt_topic_livecheck, mqtt_msg);
    }
  }

  if (System_Data.system_30s_puls)
  {
    wifi_info();
    
    DEBUG_PRINT_SERIAL.println("");
    DEBUG_PRINT_SERIAL.print("WiFi MAC address: ");
    DEBUG_PRINT_SERIAL.println(wifi_mac);
    DEBUG_PRINT_SERIAL.print("WiFi IP address: ");
    DEBUG_PRINT_SERIAL.println(wifi_ip);
    DEBUG_PRINT_SERIAL.print("WiFi counter: ");
    DEBUG_PRINT_SERIAL.println(wifi_counter);

    if (wifi_check() && mqtt_check())
    {
      strcpy(mqtt_msg, wifi_ip);
      mqtt_publish(mqtt_topic_ip, mqtt_msg);
    }
    
    DEBUG_PRINT_SERIAL.print("MQTT counter: ");
    DEBUG_PRINT_SERIAL.println(mqtt_counter);

    DEBUG_PRINT_SERIAL.print("IO relais: ");
    DEBUG_PRINT_SERIAL.println(data_relais ? "on": "off");

    if (wifi_check() && mqtt_check())
    {
      strcpy(mqtt_msg, data_relais ? "on" : "0ff");
      mqtt_publish(mqtt_topic_relaisstatus, mqtt_msg);
    }

    DEBUG_PRINT_SERIAL.println("");
    DEBUG_PRINT_SERIAL.print("Sensor temperature: ");
    DEBUG_PRINT_SERIAL.print(sensor_temperature);
    DEBUG_PRINT_SERIAL.println(" *C");
    DEBUG_PRINT_SERIAL.print("Sensor pressure: ");
    DEBUG_PRINT_SERIAL.print(sensor_pressure);
    DEBUG_PRINT_SERIAL.println(" hPa");
    DEBUG_PRINT_SERIAL.print("Sensor altitude: ");
    DEBUG_PRINT_SERIAL.print(sensor_altitude);
    DEBUG_PRINT_SERIAL.println(" m");
    //DEBUG_PRINT_SERIAL.print("Sensor humidity: ");
    //DEBUG_PRINT_SERIAL.print(sensor_humidity);
    //DEBUG_PRINT_SERIAL.println(" %");
  }

  // Data
  if (System_Data.system_100ms_puls)
    data_out++;

  // IO
  Button_Data.ButtonUpdate();

  if (Button_Data.button_puls)
  {
    data_relais = ! data_relais;
  }

  if (data_relais && !((wifi_check() && mqtt_check())))
  {
    data_led = System_Data.system_500ms_block;
  }
  else if (!data_relais && !((wifi_check() && mqtt_check())))
  {
    data_led = System_Data.system_1s_block;
  }
  else if (data_relais && ((wifi_check() && mqtt_check())))
  {
    data_led = true;
  }
  else
  {
    data_led = false;
  }
  digitalWrite(IO_RELAIS, data_relais);

  /*
  if (data_led != data_led_mem)
  {
    //mqtt_publish(mqtt_topic_ledstatus, data_led ? "on" : "off");
    if (data_led)
    {
        mqtt_publish(mqtt_topic_ledstatus, "on");
    }
    else
    {
      mqtt_publish(mqtt_topic_ledstatus, "off");    
    }
  }
  data_led_mem = data_led;
  */
  
  if (data_relais != data_relais_mem)
  {
    //mqtt_publish(mqtt_topic_relaisstatus, data_relais ? "on" : "0ff");
    if (data_relais)
    {
    mqtt_publish(mqtt_topic_relaisstatus, "on");
    }
    else
    {
      mqtt_publish(mqtt_topic_relaisstatus, "off");    
    }
  } 
  data_relais_mem = data_relais;

  //Display
  display_update();
  switch (display_mode) 
  {
    case 1:
      if ((display_button_puls) && (display_button_code == 12))
        display_mode = 2;
      break;
    case 2:
      if (display_button_puls)
        display_mode = 1;
      break;
    case 3:
      if (display_button_puls)
        display_mode = 1;
      break;
  }
  if (millis() - display_button_millis < 30000)
    display_lcd_on();
  else
    display_lcd_off();
  if ((millis() - display_button_millis > 20) && (millis() - display_button_millis < 200))
    beeper_on();
  else
    beeper_off();  

  //Temperature
  temp_indoor_mem = temp_indoor; //!!
  temp_indoor = sensor_temperature;

  temp_outdoor_mem = temp_outdoor; //!!

  temp_outdoor_timer_mem = temp_outdoor_timer; //!!
  if ((temp_outdoor_timer) && System_Data.system_100ms_puls)
    temp_outdoor_timer--;

  temp_set_mem = temp_set; //!!
  //temp_set = temp_set + 0.1;
  if (display_button_code == 11)
  {
    if (display_button_puls)
      temp_set = temp_set + 0.1;
    if ((display_button_press) && (millis() - display_button_millis > 3000) && System_Data.system_1s_puls)
      temp_set = temp_set + 1.0;
    else if ((display_button_press) && (millis() - display_button_millis > 1000) && System_Data.system_500ms_puls)
      temp_set = temp_set + 0.5;
  }
  if (display_button_code == 13)
  {
    if (display_button_puls)
      temp_set = temp_set - 0.1;
    if ((display_button_press) && (millis() - display_button_millis > 3000) && System_Data.system_1s_puls)
      temp_set = temp_set - 1.0;
    else if ((display_button_press) && (millis() - display_button_millis > 1000) && System_Data.system_500ms_puls)
      temp_set = temp_set - 0.5;
  }
  if (temp_set < TEMPERATURE_MIN)
    temp_set = TEMPERATURE_MIN;
  if (temp_set > TEMPERATURE_MAX)
    temp_set = TEMPERATURE_MAX;

  temp_mode_mem = temp_mode; //!!
  if ((temp_indoor == 0) || (temp_set == 0))
    temp_mode = TM_BOOT;
  else if (temp_indoor < temp_set - TEMPERATURE_DELTA)
    temp_mode = TM_HEATING;  
  else if (temp_indoor > temp_set + TEMPERATURE_DELTA)
    temp_mode = TM_COOLING;
  else
    temp_mode = TM_OK;

  //Sensor
  if (System_Data.system_1s_puls)
    sensor_update();
}
