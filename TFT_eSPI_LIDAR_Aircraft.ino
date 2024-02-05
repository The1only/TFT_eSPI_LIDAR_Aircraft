/**********************************************************************************
* Laser Altitude Indicator v1.0
* By Terje Nilsen, 2024
*
* This project uses a lo-cost SEEED Rp2040 board (7,-USd), with a SEEED sircualr coulor display (20,-USd), and a Garmin v3 LIDAR (120,-USd)
* (Total cost 150USd total + 3D printed box or something).
* You get RTC, LIDAR, SD-Card socket for logging, Touch screen, Colour Graphics, USB-Serial, IO pins for control or sensing etc. 
*
* It measures the actual AGL when close to ground, and can be used for lo-pass training etc. 
* In addition to the display output, it can be set to send the output over USB-Serial port to 
* an external device, like Android phone etc.for logging or cool apps.
* Many cool additions can be written, and as the touch-sceen works (even througth glass) one can have different screens by push.
* Today two different screens are implemented just to show how. 
* 
* This project and all its code is fri to use, but send pull request with updates.
* However; if it is used comersially, my name shall be listed.
**********************************************************************************/
#define Use_TFT   // Some might only whant to use an Androiud app etc. Then the Sircular display can be removed.
#undef USE_LIGHT  // If an relay for warning light etc, is mounted...
#define USE_TOUCH
#define USE_SD_CARD
#undef DEBUG

#define LOOP_COUNT 30

#define DISABLE_ALL_LIBRARY_WARNINGS  // Hmm well for now...
#define USE_TFT_ESPI_LIBRARY          // Yep we do...

#ifdef Use_TFT
#include <TFT_eSPI.h>
#include <SPI.h>
#include "lv_xiao_round_screen.h"
//#include "lv_hardware_test.h"       // Not used but many hints can be found in it...
#endif

#include <Arduino.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <SD.h>
#include "I2C_BM8563.h"  // RTC driver...
#include "NotoSansBold15.h"

// YES we may also suport ESP32 and add w-lan and bluetooth... (Not tested)
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#include "esp_wifi.h"
#include "WiFi.h"
const char *ntpServer = "time.cloudflare.com";
const char *ssid = "myssid";
const char *password = "mypassword";
#endif

#define OFFSET 1.7   // The sensor is mounted 1.7' abow ground when the rear wheels touches ground... (Must be set individually)
#define MAX_DIST 16  // 16 feet = 5 meters...

#define NUM_ADC_SAMPLE 20
#define RP2040_VREF 3300  // The actual voltage on 3V3 pin. (unit: mV)

#define CLOCK_X_POS 10  // X, Y position of screen preasure, can be used to identify location...
#define CLOCK_Y_POS 10

/***************************************************************************************
**                         Section 6: Colour enumeration
***************************************************************************************/
// Default color definitions
#define TFT_BLACK 0x0000                     /*   0,   0,   0 */
#define TFT_NAVY 0x000F                      /*   0,   0, 128 */
#define TFT_DARKGREEN 0x03E0                 /*   0, 128,   0 */
#define TFT_DARKCYAN 0x03EF                  /*   0, 128, 128 */
#define TFT_MAROON 0x7800                    /* 128,   0,   0 */
#define TFT_PURPLE 0x780F                    /* 128,   0, 128 */
#define TFT_OLIVE 0x7BE0                     /* 128, 128,   0 */
#define TFT_LIGHTGREY 0xD69A                 /* 211, 211, 211 */
#define TFT_DARKGREY 0x7BEF                  /* 128, 128, 128 */
#define TFT_BLUE 0x001F                      /*   0,   0, 255 */
#define TFT_GREEN 0x07E0                     /*   0, 255,   0 */
#define TFT_CYAN 0x07FF                      /*   0, 255, 255 */
#define TFT_RED 0xF800                       /* 255,   0,   0 */
#define TFT_MAGENTA 0xF81F                   /* 255,   0, 255 */
#define TFT_YELLOW 0xFFE0                    /* 255, 255,   0 */
#define TFT_WHITE 0xFFFF                     /* 255, 255, 255 */
#define TFT_ORANGE 0xFDA0                    /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0               /* 180, 255,   0 */
#define TFT_PINK 0xFE19 /* 255, 192, 203 */  //Lighter pink, was 0xFC9F
#define TFT_BROWN 0x9A60                     /* 150,  75,   0 */
#define TFT_GOLD 0xFEA0                      /* 255, 215,   0 */
#define TFT_SILVER 0xC618                    /* 192, 192, 192 */
#define TFT_SKYBLUE 0x867D                   /* 135, 206, 235 */
#define TFT_VIOLET 0x915C                    /* 180,  46, 226 */

#define CLOCK_FG TFT_SKYBLUE
#define OK_FG TFT_GREEN
#define CLOCK_BG TFT_NAVY
#define SECCOND_FG TFT_RED
#define LABEL_FG TFT_GOLD

#define CLOCK_R 230.0f / 2.0f  // Clock face radius (float type)
#define H_HAND_LENGTH CLOCK_R / 2.0f
#define M_HAND_LENGTH CLOCK_R / 1.4f
#define S_HAND_LENGTH CLOCK_R / 1.3f

#define FACE_W CLOCK_R * 2 + 1
#define FACE_H CLOCK_R * 2 + 1

// Calculate 1 second increment angles. Hours and minute hand angles
// change every second so we see smooth sub-pixel movement
#define SECOND_ANGLE 360.0 / 60.0
#define MINUTE_ANGLE SECOND_ANGLE / 60.0
#define HOUR_ANGLE MINUTE_ANGLE / 12.0

// Sprite width and height
#define FACE_W CLOCK_R * 2 + 1
#define FACE_H CLOCK_R * 2 + 1

LIDARLite myLidarLite;
const int ledPin = LED_BUILTIN;  // the number of the LED/Relay pin... (If used)

I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);
I2C_BM8563_TimeTypeDef flightStruct;
I2C_BM8563_TimeTypeDef timeStruct;
I2C_BM8563_DateTypeDef dateStruct;

// Log takeoff when going from red for 5sec. ...
// Log landed when 30sec. consecutive in read zone...
// Write to SD when landed...
I2C_BM8563_TimeTypeDef Takeoff_timeStruct;
I2C_BM8563_DateTypeDef Takeoff_dateStruct;
I2C_BM8563_TimeTypeDef Land_timeStruct;
I2C_BM8563_DateTypeDef Land_dateStruct;

#ifdef Use_TFT
//TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
TFT_eSprite face = TFT_eSprite(&tft);
lv_coord_t touchX, touchY;
lv_coord_t last_touchX, last_touchY;
#endif

int flash = 0;
int mode = 0;         // We support Mode 0 and Mode 1 (just screen layouts...)
int modepressed = 0;  // It is a bit tricky if someone press the screen for longer time, so we fix that...
int cal_cnt = 0;
static lv_obj_t *battery_bar, *battery_label;
bool use_lidar = true;
int flight_mode = 0;
int flight_time = 0;
int total_flight_time = 0;

// Time h:m:s
uint8_t h = 0, m = 0, s = 0;

// Time for next tick
uint32_t targetTime = 0;
const int Val[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

// Log every 1/10 sec. when in green zone... and when going from green to read.
// Stop loggin in black zone for 30sec., and when in red zoone for 1 minute.
// Stop logging = Write to SD card...
static uint16_t log_data[6500];  // 10min = 600sec = 6000samples  feet * 10 ...   10 = 1.0  255 = 25.5 feet...

// =========================================================================
void setLocalTime(void) {
  timeStruct.hours = 00;
  timeStruct.minutes = 10;
  timeStruct.seconds = 0;
  rtc.setTime(&timeStruct);

  dateStruct.weekDay = 7;
  dateStruct.month = 2;
  dateStruct.date = 4;
  dateStruct.year = 2024;
  rtc.setDate(&dateStruct);

  Serial.printf("->%04d/%02d/%02d %02d:%02d:%02d\n",
                dateStruct.year,
                dateStruct.month,
                dateStruct.date,
                timeStruct.hours,
                timeStruct.minutes,
                timeStruct.seconds);
}

// =========================================================================
// Setup
// =========================================================================
void setup() {
  Serial.begin(115200);

#ifdef DEBUG
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }
#endif

  Serial.println("Altitude Indicator v1.0");
  Serial.println("Booting...");

  // Initialise the screen
#ifdef Use_TFT
  lv_init();
  //lv_xiao_disp_init();
  lv_xiao_touch_init();
  tft.init();
  pinMode(TOUCH_INT, INPUT_PULLUP);

  // Ideally set orientation for good viewing angle range because
  // the anti-aliasing effectiveness varies with screen viewing angle
  // Usually this is when screen ribbon connector is at the bottom
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // Create the clock face sprite
  //face.setColorDepth(8); // 8 bit will work, but reduces effectiveness of anti-aliasing
  face.createSprite(FACE_W, FACE_H);

  // Only 1 font used in the sprite, so can remain loaded
  face.loadFont("NotoSansBold15");
  face.setTextDatum(MC_DATUM);
  face.setTextColor(CLOCK_FG, TFT_BLACK);
  face.setTextSize(2);
  face.drawString("Altitude Indicator", CLOCK_R, CLOCK_R * 0.7);
  face.drawString("v1.0", CLOCK_R, CLOCK_R);
  face.setTextSize(1);
  face.drawString("by Terje Nilsen", CLOCK_R, CLOCK_R * 1.3);
  face.pushSprite(5, 5, TFT_TRANSPARENT);
  delay(5000);

  // Draw the whole clock - NTP time not available yet
  renderFace(0);
#endif

  // RTC...
  struct tm timeInfo;
  Wire.begin();
  rtc.begin();

  // Set local time... when no battery mounted...
  // setLocalTime();

  // We do not use time at the moment, but can be used for automatic recoding of flight time etc...
  // Get RTC
  rtc.getDate(&dateStruct);
  rtc.getTime(&timeStruct);

  // Print RTC
  Serial.printf("%04d/%02d/%02d %02d:%02d:%02d\n",
                dateStruct.year,
                dateStruct.month,
                dateStruct.date,
                timeStruct.hours,
                timeStruct.minutes,
                timeStruct.seconds);

  Serial.print("Battery level: ");
  Serial.println(battery_level_percent());

  myLidarLite.begin(0, true);  // Set configuration to default and I2C to 400 kHz
  /*
    configure(int configuration, char lidarliteAddress)

    Selects one of several preset configurations.

    Parameters
    ----------------------------------------------------------------------------
    configuration:  Default 0.
      0: Default mode, balanced performance.
      1: Short range, high speed. Uses 0x1d maximum acquisition count.
      2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
      3: Maximum range. Uses 0xff maximum acquisition count.
      4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
      5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  //  myLidarLite.configure(0); // Change this number to try out alternate configurations
  // Read status register to check busy flag
  Wire.beginTransmission((int)LIDARLITE_ADDR_DEFAULT);
  Wire.write(0x01);  // Set the status register to be read
  // A nack means the device is not responding, report the error over serial
  int nackCatcher = Wire.endTransmission();
  if (nackCatcher != 0) {
    Serial.println("LIDAR NOT found.");
    use_lidar = false;
  }

#ifdef USE_LIGHT
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  configTime(8 * 3600, 0, ntpServer);
#endif

#ifdef USE_SD_CARD
  pinMode(D2, OUTPUT);
  if (!SD.begin(D2)) {
    Serial.print("SD Card mount failed!");
  } else {
    Serial.println("SD Card mount done!");
  }
  //  SD.end();

  // Read odometer value...
  char buffer[60];
  int32_t f_size = 0;
  if (SD.exists("odometer.txt")) {
    File myFile = SD.open("odometer.txt", FILE_READ);
    f_size = myFile.read((uint8_t*)buffer,49);
//    Serial.println(f_size);
//    Serial.println(buffer);
    myFile.close();
    if (f_size > 0) {
      sscanf(buffer,"%d",&total_flight_time);
    }
//    Serial.println("Found...");
  }
  else{
    File myFile = SD.open("odometer.txt", FILE_WRITE);
    myFile.write("1",2);    
    myFile.close();
//    Serial.println("NOT Found...");
  }
  Serial.print("odometer time in seconds = ");
  Serial.println(total_flight_time);

  if (SD.exists("log.txt")) {
    f_size=0;
    int offset = 0;
    File myFile = SD.open("log.txt", FILE_READ);
    do{
      myFile.seek(offset);
      f_size = myFile.read((uint8_t*)buffer,58);
      offset+=f_size+1;
      Serial.print(buffer);
    }while(f_size > 0);
    myFile.close();
  }

#endif
}

// =========================================================================
// Loop
// =========================================================================
void loop() {
  double dist = 0.0;
  static int m_speed = 0;
  if (++m_speed > 800) m_speed = 50;

#ifdef USE_TOUCH
  if (modepressed == 0) {
    if (chsc6x_is_pressed()) {
      chsc6x_get_xy(&touchX, &touchY);
      modepressed = 5;
      if (++mode >= 3) mode = 0;
      last_touchX = touchX;
      last_touchY = touchY;
      Serial.print("mode=");
      Serial.println(mode);
    }
  } else {
    if (!chsc6x_is_pressed()) {
      if (modepressed > 0) {
        modepressed -= 1;
      }
    }
  }
#endif

  // At the beginning of every 10 loop,
  // take a measurement with receiver bias correction
  if (++cal_cnt > 100) {
    cal_cnt = 0;
    if (use_lidar) {
      myLidarLite.distance();  // With bias correction
    }
  }

  // Take 99 measurements without receiver bias correction and print to serial terminal
  for (int i = 0; i < LOOP_COUNT; i++) {
    if (use_lidar) {
      dist += myLidarLite.distance(false);  // Get distance in cm...
    } else {
      dist += m_speed;
      delay(5);
    }
  }
  dist = dist / (LOOP_COUNT * 30.479999024640033);  //914.399970739201;  // (3000.0/3.28084);  // Make avradge and make feet from cm...
  dist = dist - OFFSET;
  if (dist < 0) dist = 0;  // Limmit distance to max feet...

//  Serial.print("dist=");
//  Serial.println(dist);

  // Update time periodically
  if (targetTime < millis()) {
    targetTime = millis() + 1000;
    rtc.getTime(&timeStruct);

    if (flight_mode == 0 && dist > 2) {
      flight_mode = 1;
      flight_time = 0;
      rtc.getDate(&Takeoff_dateStruct);
      rtc.getTime(&Takeoff_timeStruct);
    }
    if (flight_mode == 1 && dist <= 0.4) {
      flight_mode = 0;
      rtc.getDate(&Land_dateStruct);
      rtc.getTime(&Land_timeStruct);

      total_flight_time+= flight_time;

      // Read odometer value...
      char buffer[50];
      sprintf(buffer, "%d", total_flight_time); 
      File myFile = SD.open("odometer.txt", FILE_WRITE);
      myFile.seek(0);
      myFile.write(buffer,strlen(buffer)+1);    
      myFile.close();
      Serial.print("new flight time = ");
      Serial.println(total_flight_time);   

      myFile = SD.open("log.txt", FILE_WRITE);
      myFile.printf("Takeoff: %04d/%02d/%02d %02d:%02d:%02d  ",
                Takeoff_dateStruct.year,
                Takeoff_dateStruct.month,
                Takeoff_dateStruct.date,
                Takeoff_timeStruct.hours,
                Takeoff_timeStruct.minutes,
                Takeoff_timeStruct.seconds);

      myFile.printf("Landed: %04d/%02d/%02d %02d:%02d:%02d  ",
                Land_dateStruct.year,
                Land_dateStruct.month,
                Land_dateStruct.date,
                Land_timeStruct.hours,
                Land_timeStruct.minutes,
                Land_timeStruct.seconds);

       myFile.printf("Flight time (minutes): %d\n",flight_time/60);

      myFile.close();
  
    }
    if (flight_mode == 1) {
      flight_time++;
      flightStruct.hours = flight_time / 3600;
      flightStruct.minutes = (flight_time / 60) - (flightStruct.hours * 3600);
      flightStruct.seconds = flight_time - ((flightStruct.minutes * 60) + (flightStruct.hours * 3600));
    }
  }

#ifdef USE_LIGHT
  // Drive the alarm LED...
  if (dist < 2.0) {
    digitalWrite(ledPin, LOW);
  } else if (dist < 6.0) {
    if (flash < 5) {
      digitalWrite(ledPin, LOW);
    } else if (flash < 10) {
      digitalWrite(ledPin, HIGH);
    } else {
      flash = 0;
    }
    flash += 1;
  } else if (dist < 10.0) {
    if (flash < 15) {
      digitalWrite(ledPin, LOW);
    } else if (flash < 18) {
      digitalWrite(ledPin, HIGH);
    } else {
      flash = 0;
    }
    flash += 1;
  } else {
    digitalWrite(ledPin, HIGH);
  }
#endif

#ifdef Use_TFT
  if (mode == 0) {
    renderFace(dist);
  } else if (mode == 1) {
    renderFace2(dist);
  } else if (mode == 2) {
    renderFace3(dist);
  }
#endif
}

// =========================================================================
// Draw the clock face in the sprite
// =========================================================================
#ifdef Use_TFT
static void renderFace(float t) {
  if (t > MAX_DIST) t = MAX_DIST;  // Limmit distance to max feet...

  float s_angle = t * (360 / 16);  //SECOND_ANGLE;
  int Back_c;

  float Alt = t;  //float(int(t * 10) % 600) / 37.5;

  if (Alt < 2) Back_c = TFT_BROWN;
  else if (Alt < 6) Back_c = TFT_DARKGREEN;
  else Back_c = TFT_BLACK;

  // The face is completely redrawn - this can be done quickly
  face.fillSprite(TFT_BLACK);
  face.fillSmoothCircle(CLOCK_R, CLOCK_R, CLOCK_R, Back_c);
  face.setTextDatum(MC_DATUM);
  face.setTextColor(CLOCK_FG, CLOCK_BG);
  constexpr uint32_t dialOffset = CLOCK_R - 10;

  float xp = 0.0, yp = 0.0;  // Use float pixel position for smooth AA motion

  face.setTextSize(2);

  for (uint32_t h = 0; h <= 16; h++) {
    if (h < 2) face.setTextColor(TFT_RED, Back_c);
    else if (h < 7) face.setTextColor(TFT_GREEN, Back_c);
    else face.setTextColor(TFT_SKYBLUE, Back_c);

    if (h != 16) {
      getCoord(CLOCK_R, CLOCK_R, &xp, &yp, dialOffset, h * 360.0 / 16);
      face.drawNumber(Val[h], xp, 2 + yp);
    }
  }

  face.drawLine(CLOCK_R, CLOCK_R, 114, 25, TFT_DARKGREY);
  face.drawLine(CLOCK_R, CLOCK_R, 174, 50, TFT_DARKGREY);
  face.drawLine(CLOCK_R, CLOCK_R, 178, 178, TFT_DARKGREY);

  face.setCursor(CLOCK_R - 75, CLOCK_R * 0.9, 1);
  face.setTextColor(LABEL_FG, Back_c);
  face.setTextSize(1);
  char pTime[10];
  sprintf(pTime, "%02d:%02d:%02d", timeStruct.hours, timeStruct.minutes, timeStruct.seconds);
  face.println(pTime);
  face.setCursor(CLOCK_R + 30, CLOCK_R * 0.9, 1);
  sprintf(pTime, "%02d:%02d:%02d", flightStruct.hours, flightStruct.minutes, flightStruct.seconds);
  face.println(pTime);

  char strx[10];
  sprintf(strx, "%.1f", t);  //float(int(t * 10) % 600) / 37.5);

  face.setTextColor(LABEL_FG, Back_c);
  face.setTextSize(4);
  face.drawString(strx, CLOCK_R, CLOCK_R * 1.35);
  face.setTextSize(1);
  face.drawString("Laser", CLOCK_R, CLOCK_R * 0.4);
  face.setTextSize(2);
  face.drawString("Altitude", CLOCK_R, CLOCK_R * 0.60);
  face.setTextColor(CLOCK_FG, Back_c);
  face.setTextSize(2);
  face.drawString("feet", CLOCK_R, CLOCK_R * 1.6);

  // Draw the central pivot circle
  face.fillSmoothCircle(CLOCK_R, CLOCK_R, 9, CLOCK_FG);

  // Draw cecond hand
  getCoord(CLOCK_R, CLOCK_R, &xp, &yp, S_HAND_LENGTH, s_angle);
  face.drawWedgeLine(CLOCK_R, CLOCK_R, xp, yp, 7.0, 3.0, TFT_LIGHTGREY);

  face.pushSprite(5, 5, TFT_TRANSPARENT);
}

// =========================================================================
// Draw the clock face in the sprite
// =========================================================================
static void renderFace2(float t) {
  float s_angle = t * (360 / 16);  //SECOND_ANGLE;
  int Back_c;
  float Alt = t;  //float(int(t * 10) % 600) / 37.5;

  if (Alt < 2) Back_c = TFT_BROWN;
  else if (Alt < 6) Back_c = TFT_DARKGREEN;
  else Back_c = TFT_BLACK;

  // The face is completely redrawn - this can be done quickly
  face.fillSprite(TFT_BLACK);
  face.fillSmoothCircle(CLOCK_R, CLOCK_R, CLOCK_R, Back_c);
  face.setTextDatum(MC_DATUM);
  face.setTextColor(CLOCK_FG, CLOCK_BG);
  constexpr uint32_t dialOffset = CLOCK_R - 10;

  face.drawLine(CLOCK_R, CLOCK_R, 114, 25, TFT_DARKGREY);
  face.drawLine(CLOCK_R, CLOCK_R, 174, 50, TFT_DARKGREY);
  face.drawLine(CLOCK_R, CLOCK_R, 178, 178, TFT_DARKGREY);

  face.setCursor(CLOCK_R - 72, CLOCK_R * 0.55, 1);
  face.setTextColor(LABEL_FG, Back_c);
  face.setTextSize(3);
  char pTime[10];
  sprintf(pTime, "%02d:%02d:%02d", timeStruct.hours, timeStruct.minutes, timeStruct.seconds);
  face.println(pTime);
  face.setCursor(CLOCK_R - 72, CLOCK_R * 0.93, 1);
  sprintf(pTime, "%02d:%02d:%02d", flightStruct.hours, flightStruct.minutes, flightStruct.seconds);
  face.println(pTime);
  face.setTextSize(1);
  face.setTextColor(CLOCK_FG, Back_c);
  face.drawString("UTC time", CLOCK_R, CLOCK_R * 0.82);
  face.drawString("Flight time", CLOCK_R, CLOCK_R * 1.2);

  sprintf(pTime, "%.1f", t);  //float(int(t * 10) % 600) / 37.5);
  face.setTextColor(TFT_WHITE, Back_c);
  face.setTextSize(5);
  face.drawString(pTime, CLOCK_R, CLOCK_R * 1.55);
  face.setTextColor(LABEL_FG, Back_c);
  face.setTextSize(1);
  face.drawString("Laser", CLOCK_R, CLOCK_R * 0.1);
  face.setTextSize(2);
  face.drawString("Altitude", CLOCK_R, CLOCK_R * 0.30);
  face.setTextColor(CLOCK_FG, Back_c);
  face.setTextSize(2);
  face.drawString("feet", CLOCK_R, CLOCK_R * 1.8);

  face.pushSprite(5, 5, TFT_TRANSPARENT);
}


// =========================================================================
// Draw the clock face in the sprite
// =========================================================================
static void renderFace3(float t) {
  float s_angle = t * (360 / 16);  //SECOND_ANGLE;
  int Back_c;
  float Alt = t;  //float(int(t * 10) % 600) / 37.5;

  if (Alt < 2) Back_c = TFT_BROWN;
  else if (Alt < 6) Back_c = TFT_DARKGREEN;
  else Back_c = TFT_BLACK;

  // The face is completely redrawn - this can be done quickly
  face.fillSprite(TFT_BLACK);
  face.fillSmoothCircle(CLOCK_R, CLOCK_R, CLOCK_R, Back_c);
  face.setTextDatum(MC_DATUM);
  face.setTextColor(CLOCK_FG, CLOCK_BG);
  constexpr uint32_t dialOffset = CLOCK_R - 10;

  face.drawLine(CLOCK_R, CLOCK_R, 114, 25, TFT_DARKGREY);
  face.drawLine(CLOCK_R, CLOCK_R, 174, 50, TFT_DARKGREY);
  face.drawLine(CLOCK_R, CLOCK_R, 178, 178, TFT_DARKGREY);

  face.setCursor(CLOCK_R - 72, CLOCK_R * 0.55, 1);
  face.setTextColor(LABEL_FG, Back_c);
  face.setTextSize(3);
  char pTime[10];
  sprintf(pTime, "%02d:%02d:%02d", timeStruct.hours, timeStruct.minutes, timeStruct.seconds);
  face.println(pTime);
  face.setCursor(CLOCK_R - 72, CLOCK_R * 0.93, 1);
  sprintf(pTime, "%02d:%02d:%02d", flightStruct.hours, flightStruct.minutes, flightStruct.seconds);
  face.println(pTime);
  face.setTextSize(1);
  face.setTextColor(CLOCK_FG, Back_c);
  face.drawString("UTC time", CLOCK_R, CLOCK_R * 0.82);
  face.drawString("Flight time", CLOCK_R, CLOCK_R * 1.2);

  sprintf(pTime, "%.1f", t);  //float(int(t * 10) % 600) / 37.5);
  face.setTextColor(TFT_WHITE, Back_c);
  face.setTextSize(5);
  face.drawString(pTime, CLOCK_R, CLOCK_R * 1.55);
  face.setTextColor(LABEL_FG, Back_c);
  face.setTextSize(1);
  face.drawString("Laser", CLOCK_R, CLOCK_R * 0.1);
  face.setTextSize(2);
  face.drawString("Altitude", CLOCK_R, CLOCK_R * 0.30);
  face.setTextColor(CLOCK_FG, Back_c);
  face.setTextSize(2);
  face.drawString("feet", CLOCK_R, CLOCK_R * 1.8);

  face.pushSprite(5, 5, TFT_TRANSPARENT);
}
#endif

// =========================================================================

int32_t battery_level_percent(void) {
  int32_t mvolts = 0;
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
  for (int8_t i = 0; i < NUM_ADC_SAMPLE; i++) {
    mvolts += analogReadMilliVolts(D0);
  }
  mvolts /= NUM_ADC_SAMPLE;
#elif defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE) || defined(ARDUINO_SEEED_XIAO_NRF52840)
  int32_t adc_raw = 0;
  for (int8_t i = 0; i < NUM_ADC_SAMPLE; i++) {
    adc_raw += analogRead(D0);
  }
  adc_raw /= NUM_ADC_SAMPLE;
  mvolts = 2400 * adc_raw / (1 << 12);
#elif defined(ARDUINO_SEEED_XIAO_RP2040)
  int32_t adc_raw = 0;
  for (int8_t i = 0; i < NUM_ADC_SAMPLE; i++) {
    adc_raw += analogRead(D0);
  }
  adc_raw /= NUM_ADC_SAMPLE;
  mvolts = RP2040_VREF * adc_raw / (1 << 12);
#endif
  int32_t level = (mvolts - 1850) * 100 / 250;  // 1850 ~ 2100
  level = (level < 0) ? 0 : ((level > 100) ? 100 : level);
  return level;
}

// =========================================================================
// Get coordinates of end of a line, pivot at x,y, length r, angle a
// =========================================================================
// Coordinates are returned to caller via the xp and yp pointers
#define DEG2RAD 0.0174532925
void getCoord(int16_t x, int16_t y, float *xp, float *yp, int16_t r, float a) {
  float sx1 = cos((a - 90) * DEG2RAD);
  float sy1 = sin((a - 90) * DEG2RAD);
  *xp = sx1 * r + x;
  *yp = sy1 * r + y;
}

