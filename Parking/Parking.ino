// ESP32 기반 스마트 주차 시스템의 센서 측정, 상태 판정, 출력 제어를 담당하는 파일

#include <Arduino.h>

// 기능별 컴파일 옵션이다. 사용하지 않는 출력 장치는 0으로 바꾸면 관련 코드를 제외할 수 있다.
#define USE_OLED 1

#if USE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 64
  #define OLED_RESET -1

  Adafruit_SSD1306 display(
      SCREEN_WIDTH,
      SCREEN_HEIGHT,
      &Wire,
      OLED_RESET
  );
#endif

// 현재 시제품은 주차 슬롯 2개를 측정한다.
const int SLOT_COUNT = 2;

// 평균 거리가 이 값 이하이면 차량이 있는 것으로 판정한다.
// TODO: 실제 설치 환경에서 센서 각도와 차량 높이에 맞게 기준값 재검증 필요
const float THRESHOLD_CM = 40.0;

// Echo 신호가 돌아오지 않는 경우를 측정 실패로 처리하기 위한 제한 시간이다.
const unsigned long ECHO_TIMEOUT_US = 30000;

// 초음파 센서 값의 순간 오차를 줄이기 위해 여러 번 측정한 뒤 평균을 사용한다.
const int SAMPLE_COUNT = 5;

#define USE_BUZZER 1

// Heltec WiFi LoRa 32 V3 기준 핀 배치이다.
// 배열의 같은 인덱스는 하나의 주차 슬롯을 의미한다.
const int trigPins[SLOT_COUNT] = {26, 33};
const int echoPins[SLOT_COUNT] = {35, 36};
const int redLedPins[SLOT_COUNT]   = {1, 2};
const int greenLedPins[SLOT_COUNT] = {4, 5};
const int buzzerPin = 7;
const int OLED_SDA_PIN = 17;
const int OLED_SCL_PIN = 18;

// 현재 상태와 직전 상태를 비교해 상태 변경 알림을 한 번만 발생시킨다.
bool occupied[SLOT_COUNT] = {false, false};
bool prevOccupied[SLOT_COUNT] = {false, false};
bool prevFullParking = false;

/*
 * HC-SR04 센서에서 거리를 한 번 측정한다.
 * 정상 측정 시 cm 단위 거리, 실패 시 -1.0을 반환한다.
 */
float measureDistanceCm(int trigPin, int echoPin) {

  // Trigger 펄스가 안정적으로 시작되도록 먼저 LOW 상태를 짧게 유지한다.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration =
      pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    return -1.0;
  }

  // HC-SR04의 왕복 시간 기반 거리 환산식이다.
  return duration / 58.0;
}

/*
 * 같은 센서를 여러 번 측정하고 유효한 값만 평균에 포함한다.
 * 일부 측정이 실패해도 남은 값으로 판정할 수 있게 하기 위한 처리이다.
 */
float measureAverageDistanceCm(
    int trigPin,
    int echoPin) {

  float sum = 0;
  int validCount = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {

    float d =
        measureDistanceCm(trigPin, echoPin);

    if (d > 0) {
      sum += d;
      validCount++;
    }

    // 연속 측정 시 초음파 간섭이 생길 수 있어 짧게 대기한다.
    delay(50);
  }

  if (validCount == 0) {
    return -1.0;
  }

  return sum / validCount;
}

// 슬롯 점유 상태를 LED 색상으로 표시한다.
void updateLedStatus(
    int slotIndex,
    bool isOccupied) {

  if (isOccupied) {

    digitalWrite(
        redLedPins[slotIndex],
        HIGH);

    digitalWrite(
        greenLedPins[slotIndex],
        LOW);

  } else {

    digitalWrite(
        redLedPins[slotIndex],
        LOW);

    digitalWrite(
        greenLedPins[slotIndex],
        HIGH);
  }
}

// 슬롯 상태가 변경되었을 때 사용하는 짧은 알림음이다.
void beepShort() {

#if USE_BUZZER
  digitalWrite(buzzerPin, HIGH);
  delay(120);
  digitalWrite(buzzerPin, LOW);
#endif

}

// 전체 슬롯이 만차로 전환되는 순간을 구분하기 위해 두 번 울린다.
void beepFullParking() {

#if USE_BUZZER
  beepShort();
  delay(150);
  beepShort();
#endif

}

// 모든 슬롯이 점유 상태인지 확인한다.
bool isFullParking() {

  for (int i = 0; i < SLOT_COUNT; i++) {

    if (!occupied[i]) {
      return false;
    }
  }

  return true;
}

// OLED에 빈 슬롯 수, 슬롯별 상태, 측정 거리 또는 오류 상태를 표시한다.
void updateOled(float distances[]) {

#if USE_OLED

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int emptyCount = 0;

  for (int i = 0; i < SLOT_COUNT; i++) {
    if (!occupied[i]) {
      emptyCount++;
    }
  }

  display.setCursor(0, 0);
  display.println("Smart Parking");

  display.setCursor(0, 14);
  display.print("Empty: ");
  display.print(emptyCount);
  display.print(" / ");
  display.println(SLOT_COUNT);

  for (int i = 0; i < SLOT_COUNT; i++) {

    display.setCursor(
        0,
        32 + i * 14);

    display.print("S");
    display.print(i + 1);
    display.print(": ");

    if (occupied[i]) {
      display.print("FULL ");
    } else {
      display.print("EMPTY");
    }

    display.print(" ");

    if (distances[i] < 0) {
      display.print("ERR");
    } else {
      display.print(distances[i], 0);
      display.print("cm");
    }
  }

  display.display();

#endif
}

void setup() {

  Serial.begin(115200);

  // 센서 입력과 출력 장치를 초기 상태로 맞춘다.
  for (int i = 0; i < SLOT_COUNT; i++) {

    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);

    pinMode(redLedPins[i], OUTPUT);
    pinMode(greenLedPins[i], OUTPUT);

    digitalWrite(trigPins[i], LOW);
    digitalWrite(redLedPins[i], LOW);
    digitalWrite(greenLedPins[i], LOW);
  }

#if USE_BUZZER
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
#endif

#if USE_OLED

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

  if (!display.begin(
          SSD1306_SWITCHCAPVCC,
          0x3C)) {

    Serial.println(
        "OLED initialization failed.");

  } else {

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("Smart Parking");
    display.println("2 Slot Mode");
    display.println("System Start");
    display.display();

    delay(1000);
  }

#endif

  Serial.println(
      "Smart Parking System Started.");
}

void loop() {

  float distances[SLOT_COUNT];
  bool stateChanged = false;

  Serial.println(
      "========== Parking Status ==========");

  // 각 슬롯을 순서대로 측정해 센서 간 간섭을 줄인다.
  for (int i = 0; i < SLOT_COUNT; i++) {

    distances[i] =
        measureAverageDistanceCm(
            trigPins[i],
            echoPins[i]);

    prevOccupied[i] =
        occupied[i];

    if (distances[i] >= 0) {

      occupied[i] =
          (distances[i] <= THRESHOLD_CM);

      Serial.print("Slot ");
      Serial.print(i + 1);
      Serial.print(" : ");
      Serial.print(distances[i]);
      Serial.print(" cm -> ");

      Serial.println(
          occupied[i]
              ? "OCCUPIED"
              : "EMPTY");

    } else {

      // 측정 실패 시 잘못된 값으로 상태를 덮어쓰지 않고 기존 상태를 유지한다.
      Serial.print("Slot ");
      Serial.print(i + 1);
      Serial.println(
          " : Measurement Error");
    }

    updateLedStatus(
        i,
        occupied[i]);

    if (prevOccupied[i] != occupied[i]) {
      stateChanged = true;
    }

    delay(100);
  }

  bool currentFullParking = isFullParking();

  if (currentFullParking && !prevFullParking) {
    beepFullParking();
    Serial.println("Parking Full");

  } else if (stateChanged) {
    beepShort();
  }

  prevFullParking = currentFullParking;

  updateOled(distances);

  Serial.println(
      "====================================");

  Serial.println();

  delay(1000);
}
