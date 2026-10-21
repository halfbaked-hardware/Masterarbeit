// Multiplexer-Test: findet angeschlossene Multiplexer/Port-Expander und gibt die Adresse aus
// Pogo/Jumper: 3V3 -> 3V3, GND -> GND, GPIO21 -> SDA, GPIO22 -> SCL
// Erkannt werden:
//   - TCA9548A / PCA9548A (I2C-Multiplexer, 0x70 bis 0x77), inkl. Geräte hinter jedem Kanal
//   - MCP23017 (Port-Expander, 0x20 bis 0x27), inkl. Modul-ID an GPB5/GPB6
// Der Scan wiederholt sich alle 2 s; Ausgabe nur, wenn sich etwas geändert hat.

#include <Arduino.h>
#include <Wire.h>

#define PIN_SDA 21
#define PIN_SCL 22

#define MCP_IODIRB 0x01
#define MCP_GPPUB  0x0D
#define MCP_GPIOB  0x13

bool antwortet(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

// TCA9548A hat nur ein Steuerregister: geschriebener Wert muss zurückgelesen werden
bool istTca9548(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  Wire.write(0x05);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(adresse, (uint8_t)1) != 1) return false;
  bool ok = Wire.read() == 0x05;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);              // alle Kanäle wieder aus
  Wire.endTransmission();
  return ok;
}

bool mcpLesen(uint8_t adresse, uint8_t reg, uint8_t &wert) {
  Wire.beginTransmission(adresse);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(adresse, (uint8_t)1) != 1) return false;
  wert = Wire.read();
  return true;
}

void mcpSchreiben(uint8_t adresse, uint8_t reg, uint8_t wert) {
  Wire.beginTransmission(adresse);
  Wire.write(reg);
  Wire.write(wert);
  Wire.endTransmission();
}

void tcaKanaeleScannen(uint8_t tca) {
  for (uint8_t kanal = 0; kanal < 8; kanal++) {
    Wire.beginTransmission(tca);
    Wire.write(1 << kanal);
    Wire.endTransmission();

    Serial.printf("    Kanal %u:", kanal);
    uint8_t anzahl = 0;
    for (uint8_t a = 1; a < 127; a++) {
      if (a >= 0x70 && a <= 0x77) continue;   // Multiplexer selbst überspringen
      if (antwortet(a)) {
        Serial.printf(" 0x%02X", a);
        anzahl++;
      }
    }
    Serial.println(anzahl ? "" : " -");
  }
  Wire.beginTransmission(tca);
  Wire.write(0x00);
  Wire.endTransmission();
}

void mcpInfo(uint8_t mcp) {
  uint8_t iodirb = 0, gpiob = 0, gppub = 0;
  mcpLesen(mcp, MCP_IODIRB, iodirb);
  mcpLesen(mcp, MCP_GPPUB, gppub);
  // Pull-ups an GPB5/GPB6 kurz einschalten, damit die ID-Pins nicht floaten
  mcpSchreiben(mcp, MCP_GPPUB, gppub | 0x60);
  delay(2);
  if (mcpLesen(mcp, MCP_GPIOB, gpiob) && (iodirb & 0x60) == 0x60) {
    Serial.printf("    Modul-ID: GPB5=%s  GPB6=%s\n",
                  (gpiob & 0x20) ? "HIGH" : "LOW",
                  (gpiob & 0x40) ? "HIGH" : "LOW");
  }
  mcpSchreiben(mcp, MCP_GPPUB, gppub);
}

// Ergebnis als Bitmaske, damit Änderungen erkannt werden: Bit 0-7 = MCP 0x20-0x27, Bit 8-15 = TCA 0x70-0x77
uint16_t scannen(bool ausgeben) {
  uint16_t maske = 0;

  for (uint8_t a = 0x70; a <= 0x77; a++) {
    if (antwortet(a) && istTca9548(a)) {
      maske |= 1 << (8 + a - 0x70);
      if (ausgeben) {
        Serial.printf("  I2C-Multiplexer (TCA9548A) bei 0x%02X\n", a);
        tcaKanaeleScannen(a);
      }
    }
  }

  for (uint8_t a = 0x20; a <= 0x27; a++) {
    if (antwortet(a)) {
      maske |= 1 << (a - 0x20);
      if (ausgeben) {
        Serial.printf("  Port-Expander (MCP23017) bei 0x%02X  (A2..A0 = %u%u%u)\n",
                      a, (a >> 2) & 1, (a >> 1) & 1, a & 1);
        mcpInfo(a);
      }
    }
  }

  return maske;
}

// Ohne Pull-ups oder bei Kurzschluss nach GND liegen SDA/SCL auf LOW
void leitungenPruefen() {
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delay(5);
  Serial.printf("  Ruhepegel: SDA=%s  SCL=%s (beide müssen HIGH sein)\n",
                digitalRead(PIN_SDA) ? "HIGH" : "LOW",
                digitalRead(PIN_SCL) ? "HIGH" : "LOW");
}

void alleAdressenScannen() {
  Serial.print("  Alle I2C-Geräte:");
  uint8_t anzahl = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (antwortet(a)) {
      Serial.printf(" 0x%02X", a);
      anzahl++;
    }
  }
  Serial.println(anzahl ? "" : " keine");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Multiplexer-Test ===");
  leitungenPruefen();
  Wire.begin(PIN_SDA, PIN_SCL);
}

void loop() {
  static int32_t letzteMaske = -1;
  uint16_t maske = scannen(false);

  if (maske != letzteMaske) {
    Serial.println("\nScan ...");
    if (maske == 0) {
      Serial.println("  Kein Multiplexer gefunden. SDA/SCL, 3V3 und GND prüfen.");
      alleAdressenScannen();
    } else {
      scannen(true);
    }
    letzteMaske = maske;
  }

  delay(2000);
}
