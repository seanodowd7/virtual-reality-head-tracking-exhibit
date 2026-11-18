#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <stdlib.h>

const int LED_PIN = 6;
const int ZERO_BUTTON = 4;
const int BT_RX = 8;  // HM-10 TXD
const int BT_TX = 9;  // HM-10 RXD through voltage divider

const int NUM_LEDS = 66;
const int LIT_LEDS = 5;
const int BRIGHTNESS = 30;

const float YAW_RANGE = 45.0;  // Degrees from center to either end
const int YAW_DIRECTION = 1;  // Set to -1 to reverse movement

const unsigned long MOVE_INTERVAL_MS = 12;
const unsigned long DATA_TIMEOUT_MS = 2000;

SoftwareSerial HM10(BT_RX, BT_TX);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

char line[40];
int lineLength = 0;
bool lineTooLong = false;

int shownStart = -1;
int targetStart = -1;
unsigned long lastMove = 0;
unsigned long lastData = 0;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long buttonChangedAt = 0;

void showPosition(int start) {
  if (start == shownStart) return;

  strip.clear();

  if (start >= 0) {
    int middlePixel = start + LIT_LEDS / 2;
    uint32_t color;

    if (middlePixel < 22) {
      color = strip.Color(255, 0, 0);  // Red
    } else if (middlePixel < 44) {
      color = strip.Color(0, 0, 255);  // Blue
    } else {
      color = strip.Color(0, 255, 0);  // Green
    }

    for (int i = start; i < start + LIT_LEDS; i++) {
      strip.setPixelColor(i, color);
    }
  }

  strip.show();
  shownStart = start;
}

void handleYawLine() {
  if (line[0] != 'Y' || line[1] != ':') return;

  char *end;
  float yaw = strtod(line + 2, &end);

  if (end == line + 2 || (*end != '\0' && *end != ',') || !isfinite(yaw)) {
    return;
  }

  lastData = millis();
  Serial.print("Yaw: ");
  Serial.println(yaw, 1);

  float position = (yaw * YAW_DIRECTION + YAW_RANGE) / (2.0 * YAW_RANGE);
  position = constrain(position, 0.0, 1.0);
  targetStart = int(position * (NUM_LEDS - LIT_LEDS) + 0.5);
}

void readBluetooth() {
  while (HM10.available()) {
    char incoming = HM10.read();

    if (incoming == '\n') {
      if (!lineTooLong && lineLength >= 3) {
        line[lineLength] = '\0';
        handleYawLine();
      }

      lineLength = 0;
      lineTooLong = false;
    } else if (incoming != '\r' && !lineTooLong) {
      if (lineLength < sizeof(line) - 1) {
        line[lineLength++] = incoming;
      } else {
        lineTooLong = true;
      }
    }
  }
}

void checkZeroButton() {
  bool reading = digitalRead(ZERO_BUTTON);
  unsigned long now = millis();

  if (reading != lastButtonReading) {
    lastButtonReading = reading;
    buttonChangedAt = now;
  }

  if (reading != stableButtonState && now - buttonChangedAt >= 50) {
    stableButtonState = reading;

    if (stableButtonState == LOW) {
      HM10.print("ZERO\n");
      Serial.println("ZERO command sent");
      targetStart = -1;
      showPosition(-1);
    }
  }
}

void setup() {
  Serial.begin(9600);
  HM10.begin(9600);
  pinMode(ZERO_BUTTON, INPUT_PULLUP);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear();
  strip.show();

  Serial.println("Waiting for yaw from Pico W...");
}

void loop() {
  readBluetooth();
  checkZeroButton();

  unsigned long now = millis();

  if (targetStart >= 0 && now - lastData >= DATA_TIMEOUT_MS) {
    targetStart = -1;
    showPosition(-1);
  }

  if (targetStart >= 0 && shownStart != targetStart &&
      now - lastMove >= MOVE_INTERVAL_MS) {
    lastMove = now;

    if (shownStart < 0) {
      showPosition(targetStart);
    } else if (shownStart < targetStart) {
      showPosition(shownStart + 1);
    } else {
      showPosition(shownStart - 1);
    }
  }
}