// DAC-Test: ESP32 + MCP4725 auf dem Breadboard
// Verdrahtung: 3V3 -> VCC, GND -> GND, GPIO21 -> SDA, GPIO22 -> SCL
// Optional: OUT -> GPIO34, dann misst der ESP32 die Ausgangsspannung zur Kontrolle mit.
// Der MCP4725 erzeugt eine Spannung zwischen 0 V und VCC (hier 3,3 V) am Pin OUT.
// Der Test fährt Stufen ab; die Spannung an OUT mit dem Multimeter gegen GND nachmessen.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_ADC_KONTROLLE 34   // nur Eingang, ideal zum Mitmessen von OUT

#define VCC_MV 3300            // Versorgung des MCP4725 in mV (Referenz für die Ausgangsspannung)
#define STUFE_MS 4000          // so lange bleibt jede Spannung stehen

const uint16_t STUFEN_MV[] = {0, 500, 1000, 1500, 2000, 2500, 3000, 3300};
const uint8_t ANZAHL_STUFEN = sizeof(STUFEN_MV) / sizeof(STUFEN_MV[0]);

Adafruit_MCP4725 dac;
uint8_t dacAdresse = 0;

bool antwortet(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

void leitungenPruefen() {
  pinMode(PIN_SDA, INPUT_PULLDOWN);
  pinMode(PIN_SCL, INPUT_PULLDOWN);
  delay(5);
  Serial.printf("Bus-Leitungen: SDA=%s  SCL=%s (HIGH = Pull-ups vom DAC-Board vorhanden)\n",
                digitalRead(PIN_SDA) ? "HIGH" : "LOW",
                digitalRead(PIN_SCL) ? "HIGH" : "LOW");
}

bool dacSuchen() {
  dacAdresse = 0;
  Serial.print("I2C-Scan:");
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (antwortet(a)) {
      Serial.printf(" 0x%02X", a);
      n++;
      if (!dacAdresse && a >= 0x60 && a <= 0x67) dacAdresse = a;
    }
  }
  Serial.println(n ? "" : " nichts gefunden");

  if (!dacAdresse) {
    Serial.println("  [FEHLT]  MCP4725 (erwartet 0x60-0x67, meist 0x60 oder 0x61 je nach A0)");
    return false;
  }
  Serial.printf("  [OK]     MCP4725 bei 0x%02X\n", dacAdresse);
  return dac.begin(dacAdresse, &Wire);
}

// DAC-Register zurücklesen: Byte 1 = D11..D4, Byte 2 = D3..D0 (obere 4 Bit)
int dacZuruecklesen() {
  if (Wire.requestFrom(dacAdresse, (uint8_t)3) != 3) return -1;
  Wire.read();                        // Statusbyte
  uint8_t hoch = Wire.read();
  uint8_t tief = Wire.read();
  return (hoch << 4) | (tief >> 4);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== DAC-Test MCP4725 ===");

  leitungenPruefen();
  Wire.begin(PIN_SDA, PIN_SCL);

  while (!dacSuchen()) {
    Serial.println("Neuer Versuch in 2 s ...\n");
    delay(2000);
  }
  analogSetPinAttenuation(PIN_ADC_KONTROLLE, ADC_11db);   // Messbereich bis ca. 3,1 V
  Serial.println("Multimeter an OUT und GND des MCP4725 halten.\n");
}

void loop() {
  static uint8_t stufe = 0;

  uint16_t sollMv = STUFEN_MV[stufe];
  uint16_t wert = (uint32_t)sollMv * 4095 / VCC_MV;
  if (wert > 4095) wert = 4095;

  if (!antwortet(dacAdresse)) {
    Serial.println("MCP4725 antwortet nicht mehr -> neu suchen");
    while (!dacSuchen()) delay(2000);
  }
  dac.setVoltage(wert, false);        // false = nicht ins EEPROM schreiben (schont den Speicher)
  delay(50);

  int zurueck = dacZuruecklesen();
  uint32_t espMv = analogReadMilliVolts(PIN_ADC_KONTROLLE);

  Serial.printf("Stufe %u: Soll %4u mV  (DAC-Wert %4u, zurückgelesen %4d %s)  ESP misst an GPIO34: %4lu mV\n",
                stufe + 1, sollMv, wert, zurueck, zurueck == wert ? "OK" : "FEHLER", espMv);

  stufe = (stufe + 1) % ANZAHL_STUFEN;
  delay(STUFE_MS);
}
