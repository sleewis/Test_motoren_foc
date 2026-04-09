
// ═════════════════════════════════════════════════════════════════════════════
//  Test_motoren_foc — Sjoerd Leewis
//
//  Testsketch voor de twee GM4108H-120T BLDC-motoren met AS5600-encoders.
//  Doel: motoren en encoders verifiëren, en FOC-gains afstellen via serieel.
//
//  Twee modi:
//    OL  (open-loop)  — geef een spanning [V] per motor op.
//                       Gebruik dit eerst om te controleren of de motor
//                       draait en de encoder stijgt bij positieve spanning.
//    FOC (gesloten lus) — geef een koppelstroom [A] per motor op.
//                         De interne PI-regelaars regelen de stroom direct.
//
//  Seriële commando's (115200 baud):
//
//    m1 <waarde>   → motor 1 setpoint  [V] (ol) of [A] (foc)
//    m2 <waarde>   → motor 2 setpoint
//    ol            → schakel naar open-loop modus
//    foc           → schakel naar FOC modus
//    kp <waarde>   → FOC Kp aanpassen  [V/A]
//    ki <waarde>   → FOC Ki aanpassen  [V/(A·s)]
//    stop          → beide motoren naar 0
//    info          → toon huidige instellingen
//
//  Voorbeelden:
//    "m1 2.0"   → motor 1 op 2 V (open-loop) of 2 A (FOC)
//    "m2 -1.5"  → motor 2 achteruit
//    "foc"      → schakel naar FOC
//    "kp 0.8"   → FOC Kp naar 0.8
//    "stop"     → beide motoren stoppen
//
//  Hardware: MKS-ESP32FOC V2.0
//  Motoren:  2× GM4108H-120T (11 poolparen)
//  Encoders: 2× AS5600 (I²C)
// ═════════════════════════════════════════════════════════════════════════════

#include "Motor.h"

// ─── I2C-bussen ───────────────────────────────────────────────────────────────
// I2C_1 (Wire0, GPIO19/18): AS5600 motor 1 (0x36)
// I2C_2 (Wire1, GPIO23/5):  AS5600 motor 2 (0x36)
TwoWire I2C_1 = TwoWire(0);
//TwoWire I2C_2 = TwoWire(1);

// ─── Motoren ──────────────────────────────────────────────────────────────────
Motor motor1(33, 32, 25, 12, 39, 36, &I2C_1, 11);
//Motor motor2(26, 27, 14, 12, 34, 35, &I2C_2, 11);

// ─── Synchronisatie uitlijning → encoder-task ─────────────────────────────────
// De encoder-task wacht hierop zodat hij niet concurrent met alignFinish() loopt.
volatile bool motorAligned = false;

// ─── Gedeelde toestand (fast ↔ slow task) ─────────────────────────────────────
// De fast task schrijft telemetrie; de slow task schrijft commando's.
// Een mutex voorkomt dat beide tasks tegelijk schrijven.
SemaphoreHandle_t xMutex;

struct SharedState {
  // Commando's — slow task schrijft, fast task leest
  float cmd1    = 0.0f;   // setpoint motor 1  [V] of [A]
  float cmd2    = 0.0f;   // setpoint motor 2  [V] of [A]
  bool  useFOC  = false;  // false = open-loop, true = FOC

  // Telemetrie — fast task schrijft, slow task leest
  float angle1  = 0.0f;   // mechanische hoek motor 1 [rad]
  float vel1    = 0.0f;   // hoeksnelheid motor 1 [rad/s]
  float id1     = 0.0f;   // d-as stroom motor 1 [A]
  float iq1     = 0.0f;   // q-as stroom motor 1 [A]
  float cur1    = 0.0f;   // gefilterde totale stroom motor 1 [A]

  float angle2  = 0.0f;
  float vel2    = 0.0f;
  float id2     = 0.0f;
  float iq2     = 0.0f;
  float cur2    = 0.0f;

  bool  ok1     = false;
  bool  ok2     = false;
} gState;


// ═════════════════════════════════════════════════════════════════════════════
//  FAST TASK — Core 1  (500 Hz)
//
//  Voert elke 2 ms de motorregelaar uit:
//    1. Lees commando's en modus uit shared state
//    2. Roep loopOpenLoop() of loopFOC() aan per motor
//    3. Schrijf telemetrie terug naar shared state
// ═════════════════════════════════════════════════════════════════════════════
void fastTask(void *pvParameters) {
  motor1.begin();
  //motor2.begin();

  // Parallelle uitlijning — beide motoren tegelijk zodat de wachttijd
  // maar één keer valt (2 s in plaats van 2× 2 s)
  motor1.alignStart();
  //motor2.alignStart();
  delay(2000);
  motor1.alignFinish();
  //motor2.alignFinish();

  motorAligned = true;  // geef de encoder-task toestemming om te starten
  Serial.println("[Fast] Motoren klaar. Start regelaar.");

  // FreeRTOS ticks zijn 1 ms — te grof voor 2 kHz (500 µs).
  // Busy-wait met esp_timer_get_time() geeft µs-nauwkeurige timing.
  // Core 1 is exclusief voor deze task, dus busy-wait is hier prima.
  int64_t nextWakeUs = esp_timer_get_time();
  const int64_t periodUs = 500LL;  // 500 µs = 2 kHz

  while (true) {
    // ── Lees commando's van slow task ────────────────────────────────────────
    float cmd1;
    bool  useFOC;
    if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
      cmd1   = gState.cmd1;
      useFOC = gState.useFOC;
      xSemaphoreGive(xMutex);
    }

    // ── Motoren aansturen ────────────────────────────────────────────────────
    if (useFOC) {
      motor1.loopFOC( cmd1);
      //motor2.loopFOC(-cmd2);  // motor 2 gespiegeld gemonteerd → teken omdraaien
    } else {
      motor1.loopOpenLoop( cmd1);
      //motor2.loopOpenLoop(-cmd2);
    }

    // ── Telemetrie terugschrijven ─────────────────────────────────────────────
    if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
      gState.angle1 = motor1.getAngle();
      gState.vel1   = motor1.getVelocity();
      gState.id1    = motor1.getId();
      gState.iq1    = motor1.getIq();
      gState.cur1   = motor1.getAvgCurrent();
      gState.ok1    = motor1.isOk();

      //gState.angle2 = motor2.getAngle();
      //gState.vel2   = motor2.getVelocity();
      //gState.id2    = motor2.getId();
      //gState.iq2    = motor2.getIq();
      //gState.cur2   = motor2.getAvgCurrent();
      //gState.ok2    = motor2.isOk();
      xSemaphoreGive(xMutex);
    }

    // ── Wacht tot volgende periode ────────────────────────────────────────────
    nextWakeUs += periodUs;
    while (esp_timer_get_time() < nextWakeUs) { }
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  SLOW TASK — Core 0  (~10 Hz)
//
//  Leest seriële commando's en drukt telemetrie af.
//
//  Commando's worden als tekst ingegeven en geparsed:
//    "m1 2.0"   → cmd1 = 2.0
//    "m2 -1.5"  → cmd2 = -1.5
//    "foc"      → useFOC = true
//    "ol"       → useFOC = false
//    "kp 0.8"   → motor1 en motor2 focKp = 0.8
//    "ki 30.0"  → motor1 en motor2 focKi = 30.0
//    "stop"     → cmd1 = cmd2 = 0
//    "info"     → druk huidige instellingen af
// ═════════════════════════════════════════════════════════════════════════════
void slowTask(void *pvParameters) {
  // Lokale kopieën van de commando-state (voor de info-uitvoer)
  float cmd1 = 0.0f, cmd2 = 0.0f;
  bool  useFOC = false;
  float focKp = 0.5f, focKi = 50.0f;

  while (true) {
    // ── Lees telemetrie van fast task ─────────────────────────────────────────
    float a1, v1, id1, iq1, c1;
    bool  ok1;

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      a1 = gState.angle1; v1 = gState.vel1;
      id1 = gState.id1;   iq1 = gState.iq1; c1 = gState.cur1; ok1 = gState.ok1;
      //a2 = gState.angle2; v2 = gState.vel2;
      //id2 = gState.id2;   iq2 = gState.iq2; c2 = gState.cur2; ok2 = gState.ok2;
      xSemaphoreGive(xMutex);
    }

    // ── Telemetrie afdrukken ──────────────────────────────────────────────────
    // Formaat:
    //   M1: angle=3.14 vel= 12.5 Id= 0.01 Iq= 0.48 I=0.52A [OK ][FOC]
    //   M2: angle=1.57 vel=-12.5 Id= 0.00 Iq= 0.49 I=0.51A [OK ][FOC]
    const char* modus = useFOC ? "FOC" : "OL ";

    if (useFOC) {
      // In FOC-modus: toon Id en Iq (nuttig voor afstellen)
      Serial.printf(
        "M1: angle=%6.3f  vel=%7.2f rad/s  Id=%6.3fA  Iq=%6.3fA  I=%5.3fA  [%s][%s]\n",
        a1, v1, id1, iq1, c1, ok1 ? "OK  " : "FOUT", modus);
      //Serial.printf(
      //  "M2: angle=%6.3f  vel=%7.2f rad/s  Id=%6.3fA  Iq=%6.3fA  I=%5.3fA  [%s][%s]\n\n",
      //  a2, v2, id2, iq2, c2, ok2 ? "OK  " : "FOUT", modus);
    } else {
      // In open-loop: Id/Iq niet beschikbaar, toon alleen encoder en stroom
      Serial.printf(
        "M1: angle=%6.3f  vel=%7.2f rad/s  I=%5.3fA  [%s][%s]\n",
        a1, v1, c1, ok1 ? "OK  " : "FOUT", modus);
      //Serial.printf(
      //  "M2: angle=%6.3f  vel=%7.2f rad/s  I=%5.3fA  [%s][%s]\n\n",
      //  a2, v2, c2, ok2 ? "OK  " : "FOUT", modus);
    }

    // ── Seriële commando's verwerken ──────────────────────────────────────────
    while (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() == 0) continue;

      if (cmd == "stop") {
        // Beide motoren stoppen
        cmd1 = cmd2 = 0.0f;
        Serial.println("[CMD] Stop: beide motoren op 0.");

      } else if (cmd == "foc") {
        // Schakel naar FOC gesloten-lus modus
        useFOC = true;
        cmd1 = cmd2 = 0.0f;  // veiligheidshalve resetten bij moduswissel
        Serial.println("[CMD] Modus: FOC (koppelstroom [A])");

      } else if (cmd == "ol") {
        // Schakel naar open-loop spanning modus
        useFOC = false;
        cmd1 = cmd2 = 0.0f;
        Serial.println("[CMD] Modus: open-loop (spanning [V])");

      } else if (cmd == "info") {
        // Druk huidige instellingen af
        Serial.printf("[INFO] modus=%s  m1=%.3f  m2=%.3f  Kp=%.3f  Ki=%.3f\n",
                      useFOC ? "FOC" : "OL", cmd1, cmd2, focKp, focKi);

      } else if (cmd.startsWith("m1 ")) {
        // Motor 1 setpoint
        float val = cmd.substring(3).toFloat();
        val = constrain(val, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
        cmd1 = val;
        Serial.printf("[CMD] Motor 1 = %.3f %s\n", val, useFOC ? "A" : "V");

      } else if (cmd.startsWith("m2 ")) {
        // Motor 2 setpoint
        float val = cmd.substring(3).toFloat();
        val = constrain(val, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
        cmd2 = val;
        Serial.printf("[CMD] Motor 2 = %.3f %s\n", val, useFOC ? "A" : "V");

      } else if (cmd.startsWith("kp ")) {
        // FOC proportionele gain aanpassen
        focKp = cmd.substring(3).toFloat();
        motor1.setFocGains(focKp, focKi);
        //motor2.setFocGains(focKp, focKi);

      } else if (cmd.startsWith("ki ")) {
        // FOC integrerende gain aanpassen
        focKi = cmd.substring(3).toFloat();
        motor1.setFocGains(focKp, focKi);
        //motor2.setFocGains(focKp, focKi);

      } else {
        Serial.println("[?] Onbekend commando. Zie header van dit bestand voor de lijst.");
      }

      // Schrijf bijgewerkte commando's naar shared state
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        gState.cmd1   = cmd1;
        gState.cmd2   = cmd2;
        gState.useFOC = useFOC;
        xSemaphoreGive(xMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // ~10 Hz telemetrie-uitvoer
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  ENCODER TASK — Core 0
//
//  Leest de AS5600 via I2C zo snel mogelijk (~3–5 kHz bij 400 kHz I2C-bus).
//  Schrijft de hoek en snelheid naar volatile leden in Motor, zodat de
//  fast task die direct kan lezen zonder I2C-wachttijd.
//
//  Wacht met starten totdat de uitlijning klaar is, zodat alignFinish()
//  niet concurrent met pollEncoder() de encoder uitleest.
// ═════════════════════════════════════════════════════════════════════════════
void encoderTask(void *pvParameters) {
  while (!motorAligned) vTaskDelay(pdMS_TO_TICKS(1));

  while (true) {
    motor1.pollEncoder();
    // Geen delay — I2C-transacties (~150–200 µs elk) begrenzen automatisch
    // de leessnelheid. taskYIELD() geeft andere Core-0-taken een kans.
    taskYIELD();
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);  // even wachten zodat de seriële monitor opent

  Serial.println("══════════════════════════════════════════");
  Serial.println("  Test_motoren_foc — motor & encoder test");
  Serial.println("══════════════════════════════════════════");
  Serial.println("Commando's: m1 <V/A>  m2 <V/A>  ol  foc  kp <val>  ki <val>  stop  info");
  Serial.println("Wacht op uitlijning (~2 s)...");

  I2C_1.begin(19, 18, 400000);  // motor 1 encoder
  //I2C_2.begin(23,  5, 400000);  // motor 2 encoder

  xMutex = xSemaphoreCreateMutex();

  // Fast task op Core 1 met hogere prioriteit — mag nooit blokkeren
  xTaskCreatePinnedToCore(fastTask,    "FastTask",    4096, NULL, 2, NULL, 1);
  // Slow task op Core 0 met lagere prioriteit
  xTaskCreatePinnedToCore(slowTask,    "SlowTask",    4096, NULL, 1, NULL, 0);
  // Encoder task op Core 0 — leest I2C en schrijft hoek voor fast task
  xTaskCreatePinnedToCore(encoderTask, "EncoderTask", 2048, NULL, 1, NULL, 0);
}


// ─── Loop niet gebruikt — beide cores draaien via FreeRTOS ────────────────────
void loop() {
  vTaskDelete(NULL);
}
