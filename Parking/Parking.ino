/*
 * Smart Parking System
 *
 * Heltec WiFi LoRa 32 V3 보드와 HC-SR04 초음파 센서 2개를 이용하여
 * 주차 슬롯의 점유 여부를 판단하는 프로그램이다.
 *
 * 주요 기능
 * 1. 초음파 센서로 각 슬롯의 거리 측정
 * 2. 여러 번 측정한 평균값으로 오차 완화
 * 3. 설정 거리 이하이면 주차된 상태로 판정
 * 4. 적색/녹색 LED로 슬롯 상태 표시
 * 5. 슬롯 상태 변경 또는 만차 발생 시 부저 출력
 * 6. 보드 내장 OLED에 빈자리 수와 측정 결과 표시
 */

#include <Arduino.h>

// ============================================================
// 기능 사용 여부 설정
// 1: 기능 사용, 0: 기능 사용 안 함
// ============================================================

// OLED 기능을 컴파일할지 결정한다.
#define USE_OLED 1

#if USE_OLED
  // I2C 통신과 SSD1306 OLED 제어에 필요한 라이브러리
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

  // OLED 해상도 설정
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 64

  // 별도의 OLED Reset 핀을 사용하지 않으므로 -1로 설정
  #define OLED_RESET -1

  // SSD1306 OLED 객체 생성
  // Wire 객체를 사용하여 I2C 방식으로 OLED와 통신한다.
  Adafruit_SSD1306 display(
      SCREEN_WIDTH,
      SCREEN_HEIGHT,
      &Wire,
      OLED_RESET
  );
#endif

// ============================================================
// 기본 동작 설정
// ============================================================

// 감지할 주차 슬롯 개수
const int SLOT_COUNT = 2;

// 측정 거리가 이 값 이하이면 차량이 주차된 것으로 판단한다.
const float THRESHOLD_CM = 40.0;

// pulseIn()이 Echo 신호를 기다리는 최대 시간
// 30,000us 동안 신호가 없으면 측정 오류로 처리한다.
const unsigned long ECHO_TIMEOUT_US = 30000;

// 센서 하나당 반복 측정할 횟수
// 유효한 측정값만 평균에 사용한다.
const int SAMPLE_COUNT = 5;

// 부저 기능 사용 여부
#define USE_BUZZER 1

// ============================================================
// 핀 설정 - Heltec WiFi LoRa 32 V3 기준
// ============================================================

// HC-SR04 초음파 센서 핀
// 배열의 같은 인덱스가 하나의 주차 슬롯을 의미한다.
// 예: trigPins[0], echoPins[0]은 1번 슬롯 센서
const int trigPins[SLOT_COUNT] = {26, 33};
const int echoPins[SLOT_COUNT] = {35, 36};

// 슬롯 상태 표시용 LED 핀
// 차량이 있으면 적색 LED, 비어 있으면 녹색 LED가 켜진다.
const int redLedPins[SLOT_COUNT]   = {1, 2};
const int greenLedPins[SLOT_COUNT] = {4, 5};

// Active Buzzer 제어 핀
// Active Buzzer는 HIGH 신호만 인가해도 소리가 발생한다.
const int buzzerPin = 7;

// 보드 내장 OLED의 I2C 통신 핀
const int OLED_SDA_PIN = 17;
const int OLED_SCL_PIN = 18;

// ============================================================
// 주차 상태 저장 변수
// ============================================================

// 현재 각 슬롯의 점유 상태
// true: 차량 있음, false: 빈자리
bool occupied[SLOT_COUNT] = {false, false};

// 직전 반복에서의 슬롯 점유 상태
// 현재 상태와 비교하여 상태 변경 여부를 판단한다.
bool prevOccupied[SLOT_COUNT] = {false, false};

// 직전 반복에서 전체 주차장이 만차였는지 저장한다.
// 만차가 되는 순간에만 부저를 울리기 위해 사용한다.
bool prevFullParking = false;

// ============================================================
// 단일 거리 측정 함수
// ============================================================

/*
 * 지정한 HC-SR04 센서에서 거리를 한 번 측정한다.
 *
 * 매개변수
 * - trigPin: 초음파 송신 신호를 출력할 Trigger 핀
 * - echoPin: 반사파 수신 시간을 읽을 Echo 핀
 *
 * 반환값
 * - 정상 측정: 거리(cm)
 * - 측정 실패: -1.0
 */
float measureDistanceCm(int trigPin, int echoPin) {

  // 안정적인 Trigger 펄스를 만들기 위해 먼저 LOW 상태를 유지한다.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // HC-SR04에 10us 길이의 HIGH 펄스를 보내 측정을 시작한다.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Echo 핀이 HIGH로 유지된 시간을 마이크로초 단위로 측정한다.
  // 제한 시간 안에 신호가 없으면 pulseIn()은 0을 반환한다.
  unsigned long duration =
      pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  // Echo 신호를 받지 못한 경우 측정 오류를 나타내는 -1 반환
  if (duration == 0) {
    return -1.0;
  }

  // HC-SR04의 일반적인 거리 환산식
  // 음파의 왕복 시간을 거리(cm)로 변환한다.
  return duration / 58.0;
}

// ============================================================
// 평균 거리 측정 함수
// ============================================================

/*
 * 지정한 센서의 거리를 여러 번 측정하고 평균값을 반환한다.
 * 측정 오류(-1)는 평균 계산에서 제외한다.
 *
 * 반환값
 * - 유효한 측정값이 있는 경우: 평균 거리(cm)
 * - 모든 측정이 실패한 경우: -1.0
 */
float measureAverageDistanceCm(
    int trigPin,
    int echoPin) {

  // 유효한 거리 측정값의 합계
  float sum = 0;

  // 정상적으로 측정된 횟수
  int validCount = 0;

  // SAMPLE_COUNT만큼 반복 측정
  for (int i = 0; i < SAMPLE_COUNT; i++) {

    float d =
        measureDistanceCm(trigPin, echoPin);

    // 0보다 큰 정상 측정값만 평균 계산에 포함한다.
    if (d > 0) {
      sum += d;
      validCount++;
    }

    // 연속 측정 간 간섭을 줄이기 위한 대기 시간
    delay(50);
  }

  // 모든 측정이 실패한 경우 오류값 반환
  if (validCount == 0) {
    return -1.0;
  }

  // 유효한 측정값만 사용하여 평균 계산
  return sum / validCount;
}

// ============================================================
// LED 상태 표시 함수
// ============================================================

/*
 * 특정 슬롯의 주차 상태에 따라 LED를 제어한다.
 *
 * - 차량 있음: 적색 LED ON, 녹색 LED OFF
 * - 빈자리: 적색 LED OFF, 녹색 LED ON
 */
void updateLedStatus(
    int slotIndex,
    bool isOccupied) {

  if (isOccupied) {

    // 주차된 상태 표시
    digitalWrite(
        redLedPins[slotIndex],
        HIGH);

    digitalWrite(
        greenLedPins[slotIndex],
        LOW);

  } else {

    // 빈자리 상태 표시
    digitalWrite(
        redLedPins[slotIndex],
        LOW);

    digitalWrite(
        greenLedPins[slotIndex],
        HIGH);
  }
}

// ============================================================
// 부저 제어 함수
// ============================================================

/*
 * 부저를 짧게 한 번 울린다.
 * 슬롯 상태가 변경되었을 때 사용한다.
 */
void beepShort() {

#if USE_BUZZER
  digitalWrite(buzzerPin, HIGH);
  delay(120);
  digitalWrite(buzzerPin, LOW);
#endif

}

/*
 * 짧은 부저음을 두 번 발생시킨다.
 * 전체 슬롯이 만차가 되는 순간에 사용한다.
 */
void beepFullParking() {

#if USE_BUZZER
  beepShort();
  delay(150);
  beepShort();
#endif

}

// ============================================================
// 전체 만차 여부 확인 함수
// ============================================================

/*
 * 모든 슬롯이 점유 상태인지 확인한다.
 *
 * 반환값
 * - true: 모든 슬롯에 차량이 있음
 * - false: 하나 이상의 빈자리가 있음
 */
bool isFullParking() {

  for (int i = 0; i < SLOT_COUNT; i++) {

    // 빈 슬롯을 하나라도 발견하면 만차가 아니다.
    if (!occupied[i]) {
      return false;
    }
  }

  // 반복문이 끝날 때까지 빈 슬롯이 없으면 만차
  return true;
}

// ============================================================
// OLED 화면 출력 함수
// ============================================================

/*
 * OLED에 전체 빈자리 수와 각 슬롯의 상태를 출력한다.
 *
 * 표시 내용
 * - Smart Parking 제목
 * - 빈 슬롯 수 / 전체 슬롯 수
 * - 각 슬롯의 FULL 또는 EMPTY 상태
 * - 거리 측정값 또는 ERR
 */
void updateOled(float distances[]) {

#if USE_OLED

  // 이전 화면 내용을 지운다.
  display.clearDisplay();

  // 기본 글자 크기와 색상 설정
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // 현재 빈 슬롯 개수 계산
  int emptyCount = 0;

  for (int i = 0; i < SLOT_COUNT; i++) {
    if (!occupied[i]) {
      emptyCount++;
    }
  }

  // 화면 상단 제목 출력
  display.setCursor(0, 0);
  display.println("Smart Parking");

  // 빈자리 개수 출력
  display.setCursor(0, 14);
  display.print("Empty: ");
  display.print(emptyCount);
  display.print(" / ");
  display.println(SLOT_COUNT);

  // 각 슬롯의 상태와 거리 출력
  for (int i = 0; i < SLOT_COUNT; i++) {

    // 슬롯마다 출력 행의 Y 좌표를 다르게 설정한다.
    display.setCursor(
        0,
        32 + i * 14);

    // 슬롯 번호 출력
    display.print("S");
    display.print(i + 1);
    display.print(": ");

    // 주차 여부 출력
    if (occupied[i]) {
      display.print("FULL ");
    } else {
      display.print("EMPTY");
    }

    display.print(" ");

    // 거리 측정 실패 시 ERR, 정상 측정 시 정수 단위 거리 출력
    if (distances[i] < 0) {
      display.print("ERR");
    } else {
      display.print(distances[i], 0);
      display.print("cm");
    }
  }

  // 메모리에 작성한 화면 내용을 실제 OLED에 반영한다.
  display.display();

#endif
}

// ============================================================
// 초기 설정 함수
// ============================================================

void setup() {

  // 시리얼 모니터 통신 속도 설정
  Serial.begin(115200);

  // 각 슬롯에 연결된 센서와 LED 핀 초기화
  for (int i = 0; i < SLOT_COUNT; i++) {

    // Trigger는 출력, Echo는 입력으로 설정
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);

    // 적색 및 녹색 LED 핀을 출력으로 설정
    pinMode(redLedPins[i], OUTPUT);
    pinMode(greenLedPins[i], OUTPUT);

    // 시작 시 모든 출력 핀을 LOW로 초기화
    digitalWrite(trigPins[i], LOW);
    digitalWrite(redLedPins[i], LOW);
    digitalWrite(greenLedPins[i], LOW);
  }

#if USE_BUZZER
  // 부저 출력 핀 초기화
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
#endif

#if USE_OLED

  // 지정한 SDA, SCL 핀으로 I2C 통신 시작
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

  // I2C 주소 0x3C로 OLED 초기화 시도
  if (!display.begin(
          SSD1306_SWITCHCAPVCC,
          0x3C)) {

    // OLED 초기화 실패 메시지 출력
    Serial.println(
        "OLED initialization failed.");

  } else {

    // OLED 초기화 성공 시 시작 화면 표시
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("Smart Parking");
    display.println("2 Slot Mode");
    display.println("System Start");
    display.display();

    // 사용자가 시작 화면을 확인할 수 있도록 1초 대기
    delay(1000);
  }

#endif

  // 시스템 시작 메시지 출력
  Serial.println(
      "Smart Parking System Started.");
}

// ============================================================
// 메인 반복 함수
// ============================================================

void loop() {

  // 각 슬롯의 평균 거리 측정값을 저장할 배열
  float distances[SLOT_COUNT];

  // 이번 반복에서 하나 이상의 슬롯 상태가 변경되었는지 저장
  bool stateChanged = false;

  Serial.println(
      "========== Parking Status ==========");

  // 모든 주차 슬롯을 순서대로 측정하고 상태 갱신
  for (int i = 0; i < SLOT_COUNT; i++) {

    // 현재 슬롯의 평균 거리 측정
    distances[i] =
        measureAverageDistanceCm(
            trigPins[i],
            echoPins[i]);

    // 상태 갱신 전에 기존 상태를 저장
    prevOccupied[i] =
        occupied[i];

    // 거리 측정이 정상적으로 이루어진 경우
    if (distances[i] >= 0) {

      // 기준 거리 이하이면 차량이 있는 것으로 판정
      occupied[i] =
          (distances[i] <= THRESHOLD_CM);

      // 시리얼 모니터에 슬롯 번호, 거리, 판정 결과 출력
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

      // 측정 실패 시 기존 점유 상태를 유지하고 오류 메시지만 출력
      Serial.print("Slot ");
      Serial.print(i + 1);
      Serial.println(
          " : Measurement Error");
    }

    // 현재 점유 상태에 맞게 LED 갱신
    updateLedStatus(
        i,
        occupied[i]);

    // 이전 상태와 현재 상태가 다르면 상태 변경으로 기록
    if (prevOccupied[i] != occupied[i]) {
      stateChanged = true;
    }

    // 다른 초음파 센서와의 간섭을 줄이기 위한 대기
    delay(100);
  }

  // 모든 슬롯이 점유되었는지 확인
  bool currentFullParking = isFullParking();

  // 이전에는 만차가 아니었고 현재 처음 만차가 된 경우
  if (currentFullParking && !prevFullParking) {
    beepFullParking();
    Serial.println("Parking Full");

  // 만차 전환이 아니지만 슬롯 상태가 변경된 경우
  } else if (stateChanged) {
    beepShort();
  }

  // 다음 반복에서 비교할 수 있도록 현재 만차 상태 저장
  prevFullParking = currentFullParking;

  // OLED 화면에 최신 상태 출력
  updateOled(distances);

  Serial.println(
      "====================================");

  Serial.println();

  // 다음 측정 주기까지 1초 대기
  delay(1000);
}
