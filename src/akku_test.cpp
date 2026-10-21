// Akkumodul-Test: MCP23017, Display und LED-Treiber prüfen
// Pogo/Jumper: 3V3 -> 3V3, GND -> GND, GPIO21 -> SDA, GPIO22 -> SCL
// Auf dem Modul: MCP23017 (0x20 bis 0x27), SSD1306 128x64 (0x3C/0x3D),
// Grove LED Bar v2.0 (MY9221, kein I2C!): DCKI an GPB4, DI an GPB5 des MCP23017.
// LED-Ablauf: alle an, alle aus, dann Lauflicht über die 10 LEDs.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PIN_SDA 21
#define PIN_SCL 22

#define MCP_ID_B6 14     // GPB6 (GPB5 ist hier DI der LED-Bar)
#define MCP_BAR_DCKI 12  // GPB4
#define MCP_BAR_DI 13    // GPB5
#define MCP_GPIOB 0x13
#define BAR_LEDS 10

#define LED_SCHRITT_MS 300

Adafruit_MCP23X17 mcp;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

uint8_t mcpAdresse = 0;
uint8_t displayAdresse = 0;
bool idB6 = true;

bool antwortet(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

void regSchreiben(uint8_t adresse, uint8_t reg, uint8_t wert) {
  Wire.beginTransmission(adresse);
  Wire.write(reg);
  Wire.write(wert);
  Wire.endTransmission();
}

int regLesen(uint8_t adresse, uint8_t reg) {
  Wire.beginTransmission(adresse);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(adresse, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// ---------- LED-Bar (MY9221 über MCP23017) ----------
// Pro Pegelwechsel wird GPIOB direkt geschrieben (ein I2C-Schreibzugriff);
// die übrigen GPB-Pins sind Eingänge und bleiben davon unberührt.

uint8_t barGpiob = 0;

void barPin(uint8_t bit, bool pegel) {
  if (pegel) barGpiob |= bit; else barGpiob &= ~bit;
  regSchreiben(mcpAdresse, MCP_GPIOB, barGpiob);
}

const uint8_t BIT_DCKI = 1 << (MCP_BAR_DCKI - 8);
const uint8_t BIT_DI = 1 << (MCP_BAR_DI - 8);

// 16 Bit senden, MSB zuerst; MY9221 übernimmt bei jeder Taktflanke
void barSenden(uint16_t daten) {
  for (uint8_t i = 0; i < 16; i++) {
    barPin(BIT_DI, daten & 0x8000);
    barPin(BIT_DCKI, !(barGpiob & BIT_DCKI));
    daten <<= 1;
  }
}

// maske: Bit n = LED n an
void ledsSetzen(uint16_t maske) {
  Wire.setClock(400000);
  barSenden(0x0000);                          // Kommandowort
  for (uint8_t i = 0; i < 12; i++) {          // 10 LEDs + 2 Füllwörter
    barSenden((i < BAR_LEDS && (maske & (1 << i))) ? 0x00FF : 0x0000);
  }
  // Latch: DCKI ruhig, DI > 220 µs LOW, dann 4 Pulse auf DI
  barPin(BIT_DI, false);
  delayMicroseconds(300);
  for (uint8_t i = 0; i < 4; i++) {
    barPin(BIT_DI, true);
    barPin(BIT_DI, false);
  }
  delayMicroseconds(300);
}

// ---------- Display ----------

void displayZeigen(const char *ledText) {
  if (!displayAdresse) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Akkumodul-Test");
  if (mcpAdresse) {
    display.printf("MCP 0x%02X B6=%c\n", mcpAdresse, idB6 ? 'H' : 'L');
  } else {
    display.println("MCP fehlt");
  }
  display.println("LED-Bar an GPB4/GPB5");
  display.print(ledText);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Akkumodul-Test ===");

  Wire.begin(PIN_SDA, PIN_SCL);

  // MCP23017
  for (uint8_t a = 0x20; a <= 0x27 && !mcpAdresse; a++) {
    if (antwortet(a)) mcpAdresse = a;
  }
  if (mcpAdresse && mcp.begin_I2C(mcpAdresse, &Wire)) {
    Serial.printf("MCP23017 gefunden bei 0x%02X (A2..A0 = %u%u%u)\n", mcpAdresse,
                  (mcpAdresse >> 2) & 1, (mcpAdresse >> 1) & 1, mcpAdresse & 1);
    for (uint8_t pin = 0; pin < 16; pin++) {
      mcp.pinMode(pin, INPUT_PULLUP);
    }
    idB6 = mcp.digitalRead(MCP_ID_B6);
    Serial.printf("Modul-ID: GPB6=%s\n", idB6 ? "HIGH" : "LOW");
    mcp.pinMode(MCP_BAR_DCKI, OUTPUT);
    mcp.pinMode(MCP_BAR_DI, OUTPUT);
    barPin(BIT_DCKI | BIT_DI, false);
  } else {
    mcpAdresse = 0;
    Serial.println("Kein MCP23017 gefunden (0x20 bis 0x27).");
  }

  // Display
  if (antwortet(0x3C)) displayAdresse = 0x3C;
  else if (antwortet(0x3D)) displayAdresse = 0x3D;
  if (displayAdresse && display.begin(SSD1306_SWITCHCAPVCC, displayAdresse)) {
    Serial.printf("SSD1306 gefunden bei 0x%02X\n", displayAdresse);
  } else {
    displayAdresse = 0;
    Serial.println("Kein SSD1306 gefunden (0x3C/0x3D).");
  }

  displayZeigen("");
}

void loop() {
  // LED-Ablauf: Schritt 0 = alle an, 1 = alle aus, danach Lauflicht
  static uint32_t letzterSchritt = 0;
  static uint8_t schritt = 0;
  if (mcpAdresse && millis() - letzterSchritt >= LED_SCHRITT_MS) {
    letzterSchritt = millis();
    char text[22];
    if (schritt == 0) {
      ledsSetzen(0xFFFF);
      snprintf(text, sizeof(text), "LED: alle an");
    } else if (schritt == 1) {
      ledsSetzen(0);
      snprintf(text, sizeof(text), "LED: alle aus");
    } else {
      uint8_t kanal = schritt - 2;
      ledsSetzen(1 << kanal);
      snprintf(text, sizeof(text), "LED: Nr. %u", kanal + 1);
    }
    Serial.println(text);
    displayZeigen(text);
    schritt = (schritt + 1) % (BAR_LEDS + 2);
  }

  // Pegeländerungen am MCP23017 ausgeben
  static int32_t letzte = -1;
  if (mcpAdresse) {
    uint16_t pins = mcp.readGPIOAB();   // Bit 0-7 = GPA0-7, Bit 8-15 = GPB0-7
    pins |= (uint16_t)(BIT_DCKI | BIT_DI) << 8;   // Ausgänge der LED-Bar ausblenden
    if (pins != letzte) {
      Serial.print("GPA7..0: ");
      for (int8_t i = 7; i >= 0; i--) Serial.print((pins >> i) & 1);
      Serial.print("  GPB7..0: ");
      for (int8_t i = 15; i >= 8; i--) Serial.print((pins >> i) & 1);
      Serial.println();
      letzte = pins;
    }
  }

  delay(20);
}
