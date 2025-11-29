/*
  Dual MOSFET Control Test (Arduino GIGA R1 WiFi)
  ------------------------------------------------
  D6 -> Charger MOSFET (Active HIGH)
  D7 -> ESC Power MOSFET (Active HIGH)

  Rules:
   - ESC stays ON normally.
   - When charger MOSFET (D6) turns ON, ESC MOSFET (D7) turns OFF automatically.
   - Type "on" or "off" in Serial Monitor to toggle charger.
*/

const uint8_t MOSFET_CHARGER_PIN = 24;    // D6 controls charger
const uint8_t MOSFET_ESC_PIN     = 51;    // D7 controls ESC power
const bool MOSFET_ACTIVE_HIGH    = true; // HIGH = ON

String input = "";

void setup() {
  Serial.begin(115200);
  pinMode(MOSFET_CHARGER_PIN, OUTPUT);
  pinMode(MOSFET_ESC_PIN, OUTPUT);

  // Initialize safe states
  digitalWrite(MOSFET_CHARGER_PIN, MOSFET_ACTIVE_HIGH ? LOW : HIGH); // Charger OFF
  digitalWrite(MOSFET_ESC_PIN,     MOSFET_ACTIVE_HIGH ? HIGH : LOW); // ESC ON (for flight)

  Serial.println(F("=== Dual MOSFET Control Test ==="));
  Serial.println(F("Commands: 'on' -> charger ON (ESC OFF), 'off' -> charger OFF (ESC ON)"));
  Serial.print(F("Charger Pin = D"));
  Serial.println(MOSFET_CHARGER_PIN);
  Serial.print(F("ESC Pin = D"));
  Serial.println(MOSFET_ESC_PIN);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        handleCommand(input);
        input = "";
      }
    } else {
      input += c;
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "on") {
    // Turn ON charger
    digitalWrite(MOSFET_CHARGER_PIN, MOSFET_ACTIVE_HIGH ? HIGH : LOW);
    // Turn OFF ESC
    digitalWrite(MOSFET_ESC_PIN, MOSFET_ACTIVE_HIGH ? LOW : HIGH);

    Serial.println(F("Charger: ON   |  ESC: OFF"));
  }
  else if (cmd == "off") {
    // Turn OFF charger
    digitalWrite(MOSFET_CHARGER_PIN, MOSFET_ACTIVE_HIGH ? LOW : HIGH);
    // Turn ON ESC
    digitalWrite(MOSFET_ESC_PIN, MOSFET_ACTIVE_HIGH ? HIGH : LOW);

    Serial.println(F("Charger: OFF  |  ESC: ON"));
  }
  else {
    Serial.println(F("Unknown command. Type 'on' or 'off'."));
  }
}
