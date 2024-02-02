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
#define Use_TFT       // Some might only whant to use an Androiud app etc. Then the Sircular display can be removed. 
#undef USE_LIGHT      // If an relay for warning light etc, is mounted...

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
#include "I2C_BM8563.h"               // RTC driver...
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

LIDARLite myLidarLite;
const int ledPin = LED_BUILTIN;  // the number of the LED/Relay pin... (If used)

I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);
I2C_BM8563_TimeTypeDef timeStruct;
I2C_BM8563_DateTypeDef dateStruct;

#ifdef Use_TFT
//TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
TFT_eSprite face = TFT_eSprite(&tft);
lv_coord_t touchX, touchY;
lv_coord_t last_touchX, last_touchY;
#endif

int flash = 0;
int mode = 0;               // We support Mode 0 and Mode 1 (just screen layouts...)
int modepressed = 0;        // It is a bit tricky if someone press the screen for longer time, so we fix that... 
int cal_cnt = 0;

#define CLOCK_X_POS 10      // X, Y position of screen preasure, can be used to identify location...
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

// Time h:m:s
uint8_t h = 0, m = 0, s = 0;

float time_secs = h * 3600 + m * 60 + s;

// Time for next tick
uint32_t targetTime = 0;

const int Val[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

// =========================================================================
// Setup
// =========================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  // Initialise the screen
#ifdef Use_TFT
  lv_init();
  //lv_xiao_disp_init();
  lv_xiao_touch_init();
  tft.init();
  pinMode(TOUCH_INT, INPUT_PULLUP);
  Wire.begin();

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

  // Draw the whole clock - NTP time not available yet
  renderFace(0);
#endif

  myLidarLite.begin(0, true);  // Set configuration to default and I2C to 400 kHz
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  configTime(8 * 3600, 0, ntpServer);
#endif

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

//  Wire.begin();
//  rtc.begin();
//  syncTime();
}

// =========================================================================
// Loop
// =========================================================================
void loop() {

#ifdef Use_TFT
  if (modepressed == 0) {
    if (chsc6x_is_pressed()) {
      chsc6x_get_xy(&touchX, &touchY);
      modepressed = 5;
      if (++mode >= 2) mode = 0;
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
  if (++cal_cnt == 1) {
    myLidarLite.distance();  // With bias correction
  }
  cal_cnt = cal_cnt % 100;

  double dist = 0.0;

  // Take 99 measurements without receiver bias correction and print to serial terminal
  for (int i = 0; i < 30; i++) {
    dist += myLidarLite.distance(false);  // Get distance in cm...
  }

  dist = dist / 914.399970739201;  // (3000.0/3.28084);  // Make avradge and make feet from cm...
  dist = dist - OFFSET;
  if (dist < 0) dist = 0;                // Limmit distance to max feet...

  Serial.print("dist=");
  Serial.println(dist);

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

  /*
  // Update time periodically
  if (targetTime < millis()) {
    targetTime = millis() + 100;
    time_secs += 0.100;
    if (time_secs >= (60 * 60 * 24)) time_secs = 0;
*/

#ifdef Use_TFT
  if (mode == 0) {
    renderFace(dist);
  } else if (mode == 1) {
    renderFace2(dist);
  }
#endif
  // syncTime();
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

  char strx[10];
  sprintf(strx, "%.1f", t);  //float(int(t * 10) % 600) / 37.5);

  face.setTextColor(LABEL_FG, Back_c);
  face.setTextSize(4);
  face.drawString(strx, CLOCK_R, CLOCK_R * 1.35);
  face.setTextSize(1);
  face.drawString("Laser", CLOCK_R, CLOCK_R * 0.55);
  face.setTextSize(2);
  face.drawString("Altitude", CLOCK_R, CLOCK_R * 0.70);
  face.setTextColor(CLOCK_FG, Back_c);
  face.setTextSize(2);
  face.drawString("feet", CLOCK_R, CLOCK_R * 1.6);

  // Draw the central pivot circle
  face.fillSmoothCircle(CLOCK_R, CLOCK_R, 9, CLOCK_FG);

  // Draw cecond hand
  getCoord(CLOCK_R, CLOCK_R, &xp, &yp, S_HAND_LENGTH, s_angle);
  face.drawWedgeLine(CLOCK_R, CLOCK_R, xp, yp, 7.0, 3.0, TFT_LIGHTGREY);

  /*

  face.setCursor( CLOCK_R-30, CLOCK_R * 1.5, 1);
  face.setTextColor(LABEL_FG,CLOCK_BG);  
  face.setTextSize(1);
  face.println("Hello World!");
  */
  face.pushSprite(5, 5, TFT_TRANSPARENT);
}

// =========================================================================
// Draw the clock face in the sprite
// =========================================================================
static void renderFace2(float t) {
  float s_angle = t * (360 / 16);  //SECOND_ANGLE;
  int Back_c;
  float Alt = t;  //float(int(t * 10) % 600) / 37.5;

  Back_c = TFT_BLACK;

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

  char strx[10];
  sprintf(strx, "%.1f", t);  //float(int(t * 10) % 600) / 37.5);

  face.setTextColor(LABEL_FG, Back_c);
  face.setTextSize(4);
  face.drawString(strx, CLOCK_R, CLOCK_R * 1.35);
  face.setTextSize(1);
  face.drawString("Laser", CLOCK_R, CLOCK_R * 0.55);
  face.setTextSize(2);
  face.drawString("Altitude", CLOCK_R, CLOCK_R * 0.70);
  face.setTextColor(CLOCK_FG, Back_c);
  face.setTextSize(2);
  face.drawString("feet", CLOCK_R, CLOCK_R * 1.6);

  face.pushSprite(5, 5, TFT_TRANSPARENT);
}
#endif
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

void syncTime(void) {
#ifdef Use_TFT
  targetTime = millis() + 100;
  rtc.getTime(&timeStruct);
  time_secs = timeStruct.hours * 3600 + timeStruct.minutes * 60 + timeStruct.seconds;
#endif
}
