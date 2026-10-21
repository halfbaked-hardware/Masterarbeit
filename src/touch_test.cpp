// Touch-Modul-Test: MPR121 + OLED + MCP23017
// ESP32: GPIO21 -> SDA, GPIO22 -> SCL. Das Modul darf direkt am ESP32 oder
// an einem Kanal des TCA9548A hängen; beides wird automatisch erkannt.
// Auf dem Modul: MPR121 (0x5A bis 0x5D), SSD1306 128x64 (0x3C/0x3D), MCP23017 (0x20 bis 0x27).
// Elektroden an den MPR121-Pins ELEKTRODEN (Standard 0 bis 3).
// Das OLED zeigt, welche Elektrode berührt wird; Kontrast ist gedrosselt.
// Alle 1 s wird geprüft, ob das Modul noch da ist; nach Umstecken wird alles neu eingerichtet.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PIN_SDA 21
#define PIN_SCL 22

#define ELEKTRODEN 4         // genutzte Elektroden 0 bis ELEKTRODEN-1
#define OLED_KONTRAST 0x10   // 0x00 (dunkel) bis 0xFF (hell); niedrig = wenig Strom
#define PRUEF_MS 1000

Adafruit_MPR121 cap;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

uint8_t tcaAdresse = 0;      // 0 = kein Multiplexer
int8_t kanalTouch = -1;      // -1 = direkt am Bus
uint8_t mprAdresse = 0, oledAdresse = 0, mcpAdresse = 0;
bool bereit = false;
uint16_t letzterStatus = 0xFFFF;

// ---------- I2C-Hilfen ----------

bool antwortet(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

void kanal(int8_t k) {
  if (!tcaAdresse) return;
  Wire.beginTransmission(tcaAdresse);
  Wire.write(k < 0 ? 0 : 1 << k);
  Wire.endTransmission();
}

uint8_t ersteAdresse(uint8_t von, uint8_t bis) {
  for (uint8_t a = von; a <= bis; a++) {
    if (antwortet(a)) return a;
  }
  return 0;
}

// ---------- Prüfungen ----------

void leitungenPruefen() {
  pinMode(PIN_SDA, INPUT_PULLDOWN);
  pinMode(PIN_SCL, INPUT_PULLDOWN);
  delay(5);
  Serial.printf("Bus-Leitungen: SDA=%s  SCL=%s (HIGH = Pull-ups vorhanden)\n",
                digitalRead(PIN_SDA) ? "HIGH" : "LOW",
                digitalRead(PIN_SCL) ? "HIGH" : "LOW");
}

void adressenAusgeben(const char *wo) {
  Serial.printf("  %s:", wo);
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (tcaAdresse && a >= 0x70 && a <= 0x77) continue;
    if (antwortet(a)) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " -");
}

void erwartetAusgeben(const char *name, uint8_t adresse, const char *bereich) {
  if (adresse) Serial.printf("  [OK]     %-9s 0x%02X\n", name, adresse);
  else         Serial.printf("  [FEHLT]  %-9s (erwartet %s)\n", name, bereich);
}

// ---------- Erkennung ----------

void modulSuchen() {
  kanalTouch = -1;
  mprAdresse = oledAdresse = mcpAdresse = 0;

  tcaAdresse = ersteAdresse(0x70, 0x77);
  Serial.println("\nGefundene I2C-Adressen:");
  if (tcaAdresse) {
    Serial.printf("  Multiplexer TCA9548A bei 0x%02X\n", tcaAdresse);
    for (int8_t k = 0; k < 8; k++) {
      kanal(k);
      char wo[12];
      snprintf(wo, sizeof(wo), "Kanal %d", k);
      adressenAusgeben(wo);
      if (kanalTouch < 0 && ersteAdresse(0x5A, 0x5D)) kanalTouch = k;
    }
    // Kein MPR121? Dann den Kanal mit OLED nehmen, um das Modul trotzdem zu prüfen
    for (int8_t k = 0; k < 8 && kanalTouch < 0; k++) {
      kanal(k);
      if (ersteAdresse(0x3C, 0x3D)) kanalTouch = k;
    }
    kanal(kanalTouch);
  } else {
    Serial.println("  Kein Multiplexer, Modul direkt am Bus");
    adressenAusgeben("Bus");
  }

  if (tcaAdresse && kanalTouch < 0) {
    Serial.println("Touch-Modul: weder MPR121 noch OLED an einem Kanal gefunden.");
    return;
  }

  mprAdresse = ersteAdresse(0x5A, 0x5D);
  oledAdresse = ersteAdresse(0x3C, 0x3D);
  mcpAdresse = ersteAdresse(0x20, 0x27);

  if (tcaAdresse) Serial.printf("Touch-Modul an Kanal %d:\n", kanalTouch);
  else            Serial.println("Touch-Modul:");
  erwartetAusgeben("MPR121", mprAdresse, "0x5A-0x5D");
  erwartetAusgeben("OLED", oledAdresse, "0x3C/0x3D");
  erwartetAusgeben("MCP23017", mcpAdresse, "0x20-0x27");
}

// ---------- Display ----------

void displayZeigen(uint16_t status) {
  if (!oledAdresse) return;
  kanal(kanalTouch);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Touch-Modul");
  if (tcaAdresse) display.printf("  K%d", kanalTouch);
  display.setCursor(0, 10);
  display.printf("MPR 0x%02X", mprAdresse);
  if (mcpAdresse) display.printf(" MCP 0x%02X", mcpAdresse);
  else            display.print(" MCP -");

  // Ein Kästchen pro Elektrode, gefüllt = berührt
  const uint8_t breite = 128 / ELEKTRODEN;
  for (uint8_t i = 0; i < ELEKTRODEN; i++) {
    uint8_t x = i * breite + 2;
    bool an = status & (1 << i);
    if (an) display.fillRect(x, 24, breite - 4, 24, SSD1306_WHITE);
    else    display.drawRect(x, 24, breite - 4, 24, SSD1306_WHITE);
    display.setTextColor(an ? SSD1306_BLACK : SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(x + (breite - 4) / 2 - 5, 29);
    display.print(i);
  }

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 54);
  if (status == 0) {
    display.print("keine Beruehrung");
  } else {
    display.print("Pin");
    for (uint8_t i = 0; i < ELEKTRODEN; i++) {
      if (status & (1 << i)) display.printf(" %u", i);
    }
  }
  display.display();
}

// ---------- Einrichten ----------

void modulEinrichten() {
  bereit = false;
  letzterStatus = 0xFFFF;
  modulSuchen();
  if (oledAdresse && display.begin(SSD1306_SWITCHCAPVCC, oledAdresse)) {
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(OLED_KONTRAST);
  } else if (oledAdresse) {
    oledAdresse = 0;
    Serial.println("OLED antwortet, lässt sich aber nicht starten.");
  }

  if (!mprAdresse) {
    Serial.println("MPR121 fehlt, neuer Versuch in 1 s ...");
    if (oledAdresse) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Touch-Modul");
      display.println();
      display.println("MPR121 fehlt!");
      display.printf("OLED 0x%02X  ", oledAdresse);
      if (mcpAdresse) display.printf("MCP 0x%02X", mcpAdresse);
      display.display();
    }
    return;
  }

  // Autoconfig: Ladestrom/-zeit passend zu den Elektroden (ohne liegen die Werte nahe 0)
  if (!cap.begin(mprAdresse, &Wire, MPR121_TOUCH_THRESHOLD_DEFAULT,
                 MPR121_RELEASE_THRESHOLD_DEFAULT, true)) {
    Serial.println("MPR121 antwortet, lässt sich aber nicht starten.");
    return;
  }
  // Nur die angeschlossenen Elektroden einschalten, freie Pins melden sonst Fehlberührungen
  cap.writeRegister(MPR121_ECR, 0x00);
  cap.writeRegister(MPR121_ECR, 0x80 | ELEKTRODEN);
  Serial.printf("MPR121 gestartet, Elektroden 0 bis %d aktiv. Beim Start nicht berühren!\n", ELEKTRODEN - 1);

  bereit = true;
}

bool modulNochDa() {
  if (tcaAdresse && !antwortet(tcaAdresse)) return false;
  kanal(kanalTouch);
  return antwortet(mprAdresse);
}

// ---------- Setup / Loop ----------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Touch-Modul-Test ===");

  leitungenPruefen();
  Wire.begin(PIN_SDA, PIN_SCL);
  modulEinrichten();
}

void loop() {
  static uint32_t letztePruefung = 0;
  if (millis() - letztePruefung >= PRUEF_MS) {
    letztePruefung = millis();
    if (!bereit || !modulNochDa()) {
      if (bereit) Serial.println("\nTouch-Modul weg -> neu suchen");
      delay(200);                     // Kontakte nach dem Umstecken zur Ruhe kommen lassen
      modulEinrichten();
    }
  }
  if (!bereit) return;

  kanal(kanalTouch);
  uint16_t status = cap.touched() & ((1 << ELEKTRODEN) - 1);

  if (status != letzterStatus) {
    for (uint8_t i = 0; i < ELEKTRODEN; i++) {
      uint16_t bit = 1 << i;
      if (letzterStatus == 0xFFFF) break;
      if ((status & bit) && !(letzterStatus & bit)) Serial.printf("Pin %u berührt\n", i);
      if (!(status & bit) && (letzterStatus & bit)) Serial.printf("Pin %u losgelassen\n", i);
    }
    displayZeigen(status);
    letzterStatus = status;
  }

  // Rohwerte alle 2 s: bei Berührung sinkt "Gefiltert" deutlich unter "Baseline"
  static uint32_t letzteAusgabe = 0;
  if (millis() - letzteAusgabe > 2000) {
    letzteAusgabe = millis();
    Serial.print("Gefiltert:");
    for (uint8_t i = 0; i < ELEKTRODEN; i++) Serial.printf(" %4u", cap.filteredData(i));
    Serial.print("   Baseline:");
    for (uint8_t i = 0; i < ELEKTRODEN; i++) Serial.printf(" %4u", cap.baselineData(i));
    Serial.println();
  }

  delay(20);
}
