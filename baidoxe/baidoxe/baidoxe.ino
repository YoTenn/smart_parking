#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// ---------- PIN ----------
#define RFID_SS_PIN   5
#define RFID_RST_PIN  27
#define RFID_SCK_PIN  18
#define RFID_MISO_PIN 19
#define RFID_MOSI_PIN 23

#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define SERVO_PIN     13

#define IR_SLOT_1     32
#define IR_SLOT_2     33
#define IR_SLOT_3     34  // GPIO34 input-only; sensor must drive a defined HIGH/LOW level

// ---------- MODULE ----------
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo barrier;

// ---------- SETTINGS ----------
const int TOTAL_SLOTS = 3;
const int GATE_CLOSED_ANGLE = 10;
const int GATE_OPEN_ANGLE = 45;

const unsigned long GATE_OPEN_TIME = 3000;
const unsigned long PYTHON_TIMEOUT = 20000;
const unsigned long CARD_COOLDOWN = 1500;
const unsigned long SLOT_REPORT_INTERVAL = 1000; // Send status every second so PC can resync.

// ---------- STATE ----------
bool gateOpen = false;
bool waitingForPython = false;

unsigned long gateOpenedAt = 0;
unsigned long requestStartedAt = 0;
unsigned long lastCardTime = 0;
unsigned long lastSlotReportAt = 0;

String waitingUID = "";
String pendingUID = "";
String pendingAction = "";

// ---------- PARKING SENSORS ----------
// Default: LOW means occupied.
bool isSlotOccupied(int pin) {
  return digitalRead(pin) == LOW;
}

int getOccupiedSlots() {
  int count = 0;
  if (isSlotOccupied(IR_SLOT_1)) count++;
  if (isSlotOccupied(IR_SLOT_2)) count++;
  if (isSlotOccupied(IR_SLOT_3)) count++;
  return count;
}

int getFreeSlots() {
  return TOTAL_SLOTS - getOccupiedSlots();
}

void reportSlots() {
  int occupied = getOccupiedSlots();
  int freeSlots = TOTAL_SLOTS - occupied;
  Serial.print("SLOTS,");
  Serial.print(freeSlots);
  Serial.print(",");
  Serial.println(occupied);
  lastSlotReportAt = millis();
}

// ---------- LCD ----------
void showMessage(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void updateParkingLCD() {
  int occupied = getOccupiedSlots();
  int freeSlots = TOTAL_SLOTS - occupied;

  lcd.setCursor(0, 0);
  lcd.print("Trong:");
  lcd.print(freeSlots);
  lcd.print("/3 Co:");
  lcd.print(occupied);
  lcd.print("   ");

  lcd.setCursor(0, 1);
  if (waitingForPython) {
    lcd.print("Dang nhan dien ");
  } else if (gateOpen) {
    lcd.print("CONG DANG MO   ");
  } else if (freeSlots == 0) {
    lcd.print("BAI XE DA DAY  ");
  } else {
    lcd.print("Quet the RFID  ");
  }
}

// ---------- RFID UID ----------
String getCardUID() {
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

// ---------- GATE ----------
void openGate(const String& action) {
  if (gateOpen) return;

  pendingUID = waitingUID;
  pendingAction = action;
  barrier.write(GATE_OPEN_ANGLE);
  gateOpen = true;
  gateOpenedAt = millis();
  waitingForPython = false;

  Serial.println("GATE_OPENED");
  updateParkingLCD();
}

void closeGate() {
  barrier.write(GATE_CLOSED_ANGLE);
  gateOpen = false;
  Serial.println("Thanh chan: DONG");

  // Notify PC that this gate cycle finished; PC commits the transaction then.
  if (pendingUID.length() > 0 && pendingAction.length() > 0) {
    Serial.print("GATE_CLOSED,");
    Serial.print(pendingUID);
    Serial.print(",");
    Serial.println(pendingAction);
  }

  pendingUID = "";
  pendingAction = "";
  updateParkingLCD();
}

// ---------- SERIAL COMMANDS FROM PYTHON ----------
void handlePythonCommand() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  if (command.length() == 0) return;

  Serial.print("Python command: ");
  Serial.println(command);

  if (command == "OPEN,ENTRY" && waitingForPython) {
    openGate("ENTRY");
    showMessage("XE DUOC PHEP", "MO CONG VAO");
  }
  else if (command == "OPEN,EXIT" && waitingForPython) {
    openGate("EXIT");
    showMessage("XE DUOC PHEP", "MO CONG RA");
  }
  else if (command.startsWith("DENY,") && waitingForPython) {
    waitingForPython = false;

    if (command == "DENY,FULL") {
      showMessage("BAI XE DA DAY", "Khong the vao");
    }
    else if (command == "DENY,NO_PLATE") {
      showMessage("KHONG THAY BIEN", "Tu choi mo cong");
    }
    else if (command == "DENY,PLATE_MISMATCH") {
      showMessage("SAI BIEN SO", "Tu choi mo cong");
    }
    else if (command == "DENY,DUPLICATE_PLATE") {
      showMessage("BIEN SO DA CO", "Tu choi mo cong");
    }
    else {
      showMessage("KHONG HOP LE", "Tu choi mo cong");
    }

    delay(1200);
    updateParkingLCD();
  }
}

// ---------- RFID REQUEST ----------
void handleRFID() {
  if (gateOpen || waitingForPython) return;
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  if (millis() - lastCardTime < CARD_COOLDOWN) {
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }
  lastCardTime = millis();
  waitingUID = getCardUID();

  Serial.print("CARD,");
  Serial.print(waitingUID);
  Serial.print(",");
  Serial.println(getFreeSlots());

  waitingForPython = true;
  requestStartedAt = millis();
  showMessage("DANG XU LY", "Cho nhan dien...");
  updateParkingLCD();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(IR_SLOT_1, INPUT);
  pinMode(IR_SLOT_2, INPUT);
  pinMode(IR_SLOT_3, INPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();

  barrier.setPeriodHertz(50);
  barrier.attach(SERVO_PIN, 500, 2400);
  barrier.write(GATE_CLOSED_ANGLE);

  SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);
  rfid.PCD_Init();

  showMessage("SMART PARKING", "Khoi dong...");
  delay(800);
  Serial.println("SMART_PARKING_READY");
  updateParkingLCD();

  // Send initial state immediately at startup.
  reportSlots();
}

// ---------- LOOP ----------
void loop() {
  handlePythonCommand();

  if (gateOpen && millis() - gateOpenedAt >= GATE_OPEN_TIME) {
    closeGate();
  }

  if (waitingForPython && millis() - requestStartedAt >= PYTHON_TIMEOUT) {
    waitingForPython = false;
    Serial.println("PYTHON_TIMEOUT");
    showMessage("LOI NHAN DIEN", "Hay quet lai");
    delay(1000);
    updateParkingLCD();
  }

  // Periodic SLOTS packets keep the GUI synchronized even if its COM port opens
  // after the original startup packet was sent. Send immediately on count changes too.
  static int previousOccupied = -1;
  int occupied = getOccupiedSlots();
  if (occupied != previousOccupied || millis() - lastSlotReportAt >= SLOT_REPORT_INTERVAL) {
    previousOccupied = occupied;
    reportSlots();
    updateParkingLCD();
  }

  handleRFID();
  delay(10);
}
