// Haptik-Modul-Test: Vibration solange der Taster gedrückt ist
// Pogo/Jumper: 3V3 -> 3V3, GND -> GND, GPIO21 -> SDA, GPIO22 -> SCL
// Auf dem Modul: DRV2605L (0x5A), MCP23017 (0x27), Taster an GPA0 -> GND,
// Modul-ID: GPB5 und GPB6 an GND.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <Adafruit_MCP23X17.h>

// Motortyp: true = LRA (Linearaktor), false = ERM (Rundmotor mit Unwucht)
#define MOTOR_IST_LRA false

#define PIN_SDA 21
#define PIN_SCL 22

#define MCP_ADRESSE 0x27
#define MCP_TASTER 0     // GPA0
#define MCP_ID_B5 13     // GPB5
#define MCP_ID_B6 14     // GPB6

#define STAERKE 0x7F     // Vibrationsstärke im Echtzeitmodus (0x00 bis 0x7F)

Adafruit_DRV2605 drv;
Adafruit_MCP23X17 mcp;

void i2cScan() {
  Serial.println("I2C-Scan ...");
  uint8_t gefunden = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Gerät bei 0x%02X\n", a);
      gefunden++;
    }
  }
  if (gefunden == 0) {
    Serial.println("  Nichts gefunden. SDA/SCL, 3V3 und GND prüfen.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Haptik-Modul-Test ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  i2cScan();

  if (!mcp.begin_I2C(MCP_ADRESSE, &Wire)) {
    Serial.println("MCP23017 nicht gefunden (erwartet 0x27). Verkabelung prüfen!");
    while (true) delay(1000);
  }
  Serial.println("MCP23017 gefunden.");

  if (!drv.begin(&Wire)) {
    Serial.println("DRV2605L nicht gefunden (erwartet 0x5A). Verkabelung prüfen!");
    while (true) delay(1000);
  }
  Serial.println("DRV2605L gefunden.");

  mcp.pinMode(MCP_TASTER, INPUT_PULLUP);
  mcp.pinMode(MCP_ID_B5, INPUT_PULLUP);
  mcp.pinMode(MCP_ID_B6, INPUT_PULLUP);
  Serial.printf("Modul-ID: GPB5=%s  GPB6=%s (beide LOW = Haptik)\n",
                mcp.digitalRead(MCP_ID_B5) ? "HIGH" : "LOW",
                mcp.digitalRead(MCP_ID_B6) ? "HIGH" : "LOW");

  if (MOTOR_IST_LRA) {
    drv.useLRA();
  } else {
    drv.useERM();
  }
  drv.setMode(DRV2605_MODE_REALTIME);
  drv.setRealtimeValue(0);
  Serial.println("Bereit. Taster gedrückt halten = Vibration.");
}

void loop() {
  static bool warGedrueckt = false;
  bool gedrueckt = mcp.digitalRead(MCP_TASTER) == LOW;

  if (gedrueckt != warGedrueckt) {
    drv.setRealtimeValue(gedrueckt ? STAERKE : 0);
    Serial.println(gedrueckt ? "Taster gedrückt -> Vibration an" : "Taster losgelassen -> Vibration aus");
    warGedrueckt = gedrueckt;
  }

  delay(20);   // Entprellen
}
