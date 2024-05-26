#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

#define RightFrontFWD 13
#define RightFrontBWD 14
#define RightBackFWD 26
#define RightBackBWD 27

#define LeftFrontFWD 32
#define LeftFrontBWD 33
#define LeftBackFWD 19
#define LeftBackBWD 18

#define CONTROLLED 0
#define AUTONOMOUS 1

bool opmode = 0;

#define FACE_COUNT 7
volatile int face = 0;

hw_timer_t *My_timer = NULL;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void IRAM_ATTR onTimer() {
  ++face;
  if (face > FACE_COUNT)
    face = 0;
}

void setup() {
	pinMode(RightFrontFWD, OUTPUT);
	pinMode(RightFrontBWD, OUTPUT);
	pinMode(RightBackFWD, OUTPUT);
	pinMode(RightBackBWD, OUTPUT);
	pinMode(LeftFrontFWD, OUTPUT);
	pinMode(LeftFrontBWD, OUTPUT);
	pinMode(LeftBackFWD, OUTPUT);
	pinMode(LeftBackBWD, OUTPUT);

	Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.display();
  delay(250);

  display.clearDisplay();
 
	Dabble.begin("MyEsp32");

  // timer for face
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer);
}

void loop() {
  if (opmode < 2)
    detectKeyPress();
  drawFace();
}

bool keyPress() {
  return GamePad.isUpPressed() || GamePad.isDownPressed() || GamePad.isLeftPressed() ||
         GamePad.isRightPressed() || GamePad.isSquarePressed() || GamePad.isCirclePressed() ||
         GamePad.isCrossPressed() || GamePad.isTrianglePressed() || GamePad.isSelectPressed() ||
         GamePad.isStartPressed();
}

void detectKeyPress() {
	Dabble.processInput();//this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.
	if (GamePad.isUpPressed()) {
		Serial.println("Up");
		moveForward();
	} else if (GamePad.isDownPressed()) {
		Serial.println("Down");
		moveBackward();
	} else if (GamePad.isLeftPressed()) {
		Serial.println("Left");
		rotateLeft();
	} else if (GamePad.isRightPressed()) {
		Serial.println("Right");
		rotateRight();
	} else if (GamePad.isSquarePressed()) {
		Serial.println("Square");
		moveSidewaysRight();
	} else if (GamePad.isCirclePressed()) {
		Serial.println("Circle");
		moveSidewaysLeft();
	} else if (GamePad.isCrossPressed()) {
		Serial.println("Cross");
		moveRightForward();
	} else if (GamePad.isTrianglePressed()) {
		Serial.println("Triangle");
		moveLeftForward();
	} else {
		stopMoving();
	}

	if (GamePad.isStartPressed()) {
		Serial.println("Start");
	}

	if (GamePad.isSelectPressed()) {
		Serial.println("Select");
    opmode = 1 - opmode;
	}
  delay(100);
}

void drawFace() {
  if (opmode == AUTONOMOUS)
    face = 8;

   display.clearDisplay();
  switch (face) {
    case 0:
      // Angry Eyes
      display.fillTriangle(56, 36,  12, 36,  16, 16, SSD1306_WHITE);
      display.fillTriangle(72, 36, 112, 36, 108, 16, SSD1306_WHITE);
      display.display();
      break;

    case 1:
      // Center Eyes
      display.fillCircle(display.width() / 2 - 25, display.height() / 2, 15, SSD1306_WHITE);
      display.fillCircle(display.width() / 2 + 25, display.height() / 2, 15, SSD1306_WHITE);
      display.display();
      break;

    case 2:
      // Right Eyes
      display.fillCircle(display.width() / 2 - 15, display.height() / 2, 15, SSD1306_WHITE);
      display.fillCircle(display.width() / 2 + 35, display.height() / 2, 15, SSD1306_WHITE);
      display.display();
      break;

    case 3:
      // Center Eyes
      display.fillCircle(display.width() / 2 - 25, display.height() / 2, 15, SSD1306_WHITE);
      display.fillCircle(display.width() / 2 + 25, display.height() / 2, 15, SSD1306_WHITE);
      display.display();
      break;

    case 4:
      // Blink
      display.fillRoundRect(display.width() / 2 - 40, display.height() / 2, 30, 8, 8, SSD1306_WHITE);
      display.fillRoundRect(display.width() / 2 + 2, display.height() / 2, 30, 8, 8, SSD1306_WHITE);
      display.display();
      break;

    case 5:
      // Center Eyes
      display.fillCircle(display.width() / 2 - 25, display.height() / 2, 15, SSD1306_WHITE);
      display.fillCircle(display.width() / 2 + 25, display.height() / 2, 15, SSD1306_WHITE);
      display.display();
      break;
  
    case 6:
       // Left Eyes
      display.fillCircle(display.width() / 2 - 35, display.height() / 2, 15, SSD1306_WHITE);
      display.fillCircle(display.width() / 2 + 15, display.height() / 2, 15, SSD1306_WHITE);
      display.display();
      break;

    case 7:
      // Blink Left
      display.fillRoundRect(display.width() / 2 - 60, display.height() / 2, 30, 8, 8, SSD1306_WHITE);
      display.fillRoundRect(display.width() / 2 - 15, display.height() / 2, 30, 8, 8, SSD1306_WHITE);
      display.display();
      break;

    default:
      display.fillRoundRect(display.width() / 2 - 15, display.height() / 2, 30, 8, 8, SSD1306_WHITE);
      display.display();
      break;
  }
}

void moveForward() {
	digitalWrite(RightFrontFWD, HIGH);
	digitalWrite(RightFrontBWD, LOW);
	digitalWrite(RightBackFWD, HIGH);
	digitalWrite(RightBackBWD, LOW);

	digitalWrite(LeftFrontFWD, HIGH);
	digitalWrite(LeftFrontBWD, LOW);
	digitalWrite(LeftBackFWD, HIGH);
	digitalWrite(LeftBackBWD, LOW);
}

void moveBackward() {
	digitalWrite(RightFrontFWD, LOW);
	digitalWrite(RightFrontBWD, HIGH);
	digitalWrite(RightBackFWD, LOW);
	digitalWrite(RightBackBWD, HIGH);

	digitalWrite(LeftFrontFWD, LOW);
	digitalWrite(LeftFrontBWD, HIGH);
	digitalWrite(LeftBackFWD, LOW);
	digitalWrite(LeftBackBWD, HIGH);
}

void rotateRight() {
	digitalWrite(RightFrontFWD, LOW);
	digitalWrite(RightFrontBWD, HIGH);
	digitalWrite(RightBackFWD, LOW);
	digitalWrite(RightBackBWD, HIGH);

	digitalWrite(LeftFrontFWD, HIGH);
	digitalWrite(LeftFrontBWD, LOW);
	digitalWrite(LeftBackFWD, HIGH);
	digitalWrite(LeftBackBWD, LOW);
}

void rotateLeft() {
	digitalWrite(RightFrontFWD, HIGH);
	digitalWrite(RightFrontBWD, LOW);
	digitalWrite(RightBackFWD, HIGH);
	digitalWrite(RightBackBWD, LOW);

	digitalWrite(LeftFrontFWD, LOW);
	digitalWrite(LeftFrontBWD, HIGH);
	digitalWrite(LeftBackFWD, LOW);
	digitalWrite(LeftBackBWD, HIGH);
}

void moveSidewaysRight() {
	digitalWrite(RightFrontFWD, LOW);
	digitalWrite(RightFrontBWD, HIGH);
	digitalWrite(RightBackFWD, HIGH);
	digitalWrite(RightBackBWD, LOW);

	digitalWrite(LeftFrontFWD, HIGH);
	digitalWrite(LeftFrontBWD, LOW);
	digitalWrite(LeftBackFWD, LOW);
	digitalWrite(LeftBackBWD, HIGH);
}

void moveSidewaysLeft() {
	digitalWrite(RightFrontFWD, HIGH);
	digitalWrite(RightFrontBWD, LOW);
	digitalWrite(RightBackFWD, LOW);
	digitalWrite(RightBackBWD, HIGH);

	digitalWrite(LeftFrontFWD, LOW);
	digitalWrite(LeftFrontBWD, HIGH);
	digitalWrite(LeftBackFWD, HIGH);
	digitalWrite(LeftBackBWD, LOW);
}

void moveRightForward() {
	digitalWrite(RightFrontFWD, LOW);
	digitalWrite(RightFrontBWD, LOW);
	digitalWrite(RightBackFWD, HIGH);
	digitalWrite(RightBackBWD, LOW);

	digitalWrite(LeftFrontFWD, HIGH);
	digitalWrite(LeftFrontBWD, LOW);
	digitalWrite(LeftBackFWD, LOW);
	digitalWrite(LeftBackBWD, LOW);
}

void moveLeftForward() {
	digitalWrite(RightFrontFWD, HIGH);
	digitalWrite(RightFrontBWD, LOW);
	digitalWrite(RightBackFWD, LOW);
	digitalWrite(RightBackBWD, LOW);

	digitalWrite(LeftFrontFWD, LOW);
	digitalWrite(LeftFrontBWD, LOW);
	digitalWrite(LeftBackFWD, HIGH);
	digitalWrite(LeftBackBWD, LOW);
}

void stopMoving() {
	digitalWrite(RightFrontFWD, LOW);
	digitalWrite(RightFrontBWD, LOW);
	digitalWrite(RightBackFWD, LOW);
	digitalWrite(RightBackBWD, LOW);

	digitalWrite(LeftFrontFWD, LOW);
	digitalWrite(LeftFrontBWD, LOW);
	digitalWrite(LeftBackFWD, LOW);
	digitalWrite(LeftBackBWD, LOW);
}

void testdrawcircle(void) {
  for (int16_t i = 0; i < display.height(); i += 2) {
    display.drawCircle(display.width() / 2, display.height() / 2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
}

void drawRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color) {
  display.drawRoundRect(x0, y0, w, h, radius, color);
}

void fillRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color) {
  display.fillRoundRect(x0, y0, w, h, radius, color);
}

void drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
  display.drawTriangle(x0, y0, x1, y1, x2, y2, color);
}

void fillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
  display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
}
