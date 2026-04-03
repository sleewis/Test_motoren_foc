
#include "Motor.h"

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
Motor::Motor(int u, int v, int w, int en,
             int ia, int ib,
             TwoWire *wbus,
             int poles)
{
  U = u; V = v; W = w;
  EN = en;
  IA_PIN = ia;
  IB_PIN = ib;
  wire      = wbus;
  polePairs = poles;

  rawAngle = 0;
  lastMechAngle = prevMechAngle = offset = _velocity = 0.0f;
  avgIa = avgIb = avgIc = avgI = peakI = 0.0f;

  _id = _iq = _id_integral = _iq_integral = 0.0f;

  // FOC-gains — startwaarden, aanpasbaar via setFocGains()
  _focKp     = 0.5f;   // [V/A]
  _focKi     = 50.0f;  // [V/(A·s)]
  _focIntMax = 6.0f;   // [V]  anti-windup limiet

  // Shunt-versterker MKS-ESP32FOC V2.0
  shunt  = 0.01f;
  gain   = 50.0f;
  adcMid = 1845.0f;

  _encoderOk   = false;
  _overcurrent = false;
}


// ─────────────────────────────────────────────────────────────────────────────
// Initialisatie & uitlijning
// ─────────────────────────────────────────────────────────────────────────────
void Motor::begin() {
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  ledcAttach(U, PWM_FREQ, PWM_RES);
  ledcAttach(V, PWM_FREQ, PWM_RES);
  ledcAttach(W, PWM_FREQ, PWM_RES);
}

void Motor::alignStart() {
  Serial.println("[Motor] Uitlijning starten...");
  applyVoltageAlphaBeta(ALIGN_VOLTAGE, 0.0f);
}

void Motor::alignFinish() {
  bool ok = false;
  for (int i = 0; i < 10 && !ok; i++) { ok = readSensor(); if (!ok) delay(5); }

  if (!ok) {
    Serial.println("[Motor] FOUT: encoder niet bereikbaar!");
    _encoderOk = false; brake(); return;
  }

  float mech = rawAngle * (TWO_PI / 4096.0f);
  offset        = -mech * polePairs;
  lastMechAngle = prevMechAngle = mech;
  _velocity     = 0.0f;
  _encoderOk    = true;
  brake();
  Serial.printf("[Motor] Uitlijning klaar. Offset: %.4f rad\n", offset);
}


// ─────────────────────────────────────────────────────────────────────────────
// FOC gains aanpassen tijdens runtime
// ─────────────────────────────────────────────────────────────────────────────
void Motor::setFocGains(float kp, float ki) {
  _focKp = kp;
  _focKi = ki;
  // Integratoren resetten zodat de nieuwe gains direct schoon starten
  _id_integral = 0.0f;
  _iq_integral = 0.0f;
  Serial.printf("[FOC] Kp=%.3f  Ki=%.3f\n", kp, ki);
}


// ─────────────────────────────────────────────────────────────────────────────
// Gemeenschappelijke update: encoder + snelheid + stroom + beveiliging
//
// Wordt aangeroepen door zowel loopOpenLoop() als loopFOC() om duplicaat
// code te vermijden. Geeft false als de motor gestopt moet worden.
//
// Ia, Ib, Ic worden via referentie teruggegeven voor gebruik in de FOC-keten.
// ─────────────────────────────────────────────────────────────────────────────
bool Motor::commonUpdate(float &Ia, float &Ib, float &Ic) {
  // ── Encoder ──────────────────────────────────────────────────────────────
  if (!readSensor()) {
    _encoderOk = false; brake(); return false;
  }
  _encoderOk    = true;
  lastMechAngle = rawAngle * (TWO_PI / 4096.0f);

  // ── Snelheid (differentie + wrap-correctie + low-pass filter) ────────────
  float delta = lastMechAngle - prevMechAngle;
  if      (delta >  PI) delta -= TWO_PI;
  else if (delta < -PI) delta += TWO_PI;
  float rawVelocity = delta / MOTOR_DT;
  _velocity     = VELOCITY_ALPHA * _velocity + (1.0f - VELOCITY_ALPHA) * rawVelocity;
  prevMechAngle = lastMechAngle;

  // ── Fasestromen meten ─────────────────────────────────────────────────────
  Ia = readCurrent(IA_PIN);
  Ib = readCurrent(IB_PIN);
  Ic = -Ia - Ib;   // Kirchhoff: Ia + Ib + Ic = 0

  // EMA-filters voor telemetrie
  avgIa = 0.9f * avgIa + 0.1f * fabsf(Ia);
  avgIb = 0.9f * avgIb + 0.1f * fabsf(Ib);
  avgIc = 0.9f * avgIc + 0.1f * fabsf(Ic);
  avgI  = avgIa + avgIb + avgIc;
  peakI = fabsf(Ia) + fabsf(Ib) + fabsf(Ic);

  // ── Overstroom-beveiliging ────────────────────────────────────────────────
  if (peakI > MAX_CURRENT_A) {
    _overcurrent = true; brake();
    Serial.printf("[Motor] OVERSTROOM: %.2f A\n", peakI);
    return false;
  }
  _overcurrent = false;
  return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// Open-loop spanning
//
// Stuurt de motor aan met een vaste fasespanning, zonder te kijken naar de
// gemeten stroom. De encoder-hoek wordt WEL gebruikt voor sinusvormige
// commutatie (= veldgeoriënteerd, maar zonder stroomterugkoppeling).
//
// Gebruik dit om te controleren:
//   - Draait de motor in de juiste richting?
//   - Leest de encoder correct (stijgende hoek bij positieve spanning)?
//   - Is de uitlijnoffset correct?
//
// voltage: [V], bereik [-VOLTAGE_LIMIT, +VOLTAGE_LIMIT]
// ─────────────────────────────────────────────────────────────────────────────
void Motor::loopOpenLoop(float voltage) {
  float Ia, Ib, Ic;
  if (!commonUpdate(Ia, Ib, Ic)) return;

  // Elektrische hoek + 90° voor maximaal koppel (spanning loodrecht op rotor)
  float theta  = lastMechAngle * polePairs + offset + HALF_PI;

  // Sinusvormige spanning in het αβ-stelsel
  // Dit is equivalent aan de klassieke sinusvormige driefase-commutatie
  float Valpha = sinf(theta) * voltage;
  float Vbeta  = -cosf(theta) * voltage;

  applyVoltageAlphaBeta(Valpha, Vbeta);

  // Id en Iq op NaN zetten zodat duidelijk is dat de FOC-regelaar niet actief is
  _id = NAN;
  _iq = NAN;
}


// ─────────────────────────────────────────────────────────────────────────────
// FOC gesloten lus
//
// Voert de volledige FOC-keten uit:
//   Clarke → Park → PI(Id,Iq) → inverse Park → inverse Clarke → PWM
//
// iqTarget: gewenste koppelstroom [A]
// ─────────────────────────────────────────────────────────────────────────────
void Motor::loopFOC(float iqTarget) {
  float Ia, Ib, Ic;
  if (!commonUpdate(Ia, Ib, Ic)) return;

  // ── Clarke-transform: (Ia, Ib, Ic) → (Iα, Iβ) ───────────────────────────
  // Converteert driefasige stroom naar stilstaand tweeassig stelsel.
  float Ialpha = Ia;
  float Ibeta  = (Ia + 2.0f * Ib) * 0.5773503f;  // 1/√3

  // ── Park-transform: (Iα, Iβ) → (Id, Iq) ─────────────────────────────────
  // Converteert naar roterend stelsel dat met de rotor meedraait.
  // Geen HALF_PI hier — de PI-regelaars zorgen zelf voor de 90°-verschuiving.
  float theta = lastMechAngle * polePairs + offset;
  float cosT  = cosf(theta);
  float sinT  = sinf(theta);
  _id =  Ialpha * cosT + Ibeta * sinT;
  _iq = -Ialpha * sinT + Ibeta * cosT;

  // ── PI op Id (target = 0 A) ───────────────────────────────────────────────
  float id_error   = -_id;
  _id_integral    += id_error * MOTOR_DT;
  _id_integral     = constrain(_id_integral, -_focIntMax / _focKi, _focIntMax / _focKi);
  float Vd = constrain(_focKp * id_error + _focKi * _id_integral, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

  // ── PI op Iq (target = iqTarget) ─────────────────────────────────────────
  float iq_error   = iqTarget - _iq;
  _iq_integral    += iq_error * MOTOR_DT;
  _iq_integral     = constrain(_iq_integral, -_focIntMax / _focKi, _focIntMax / _focKi);
  float Vq = constrain(_focKp * iq_error + _focKi * _iq_integral, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

  // ── Inverse Park: (Vd, Vq) → (Vα, Vβ) ───────────────────────────────────
  float Valpha = Vd * cosT - Vq * sinT;
  float Vbeta  = Vd * sinT + Vq * cosT;

  applyVoltageAlphaBeta(Valpha, Vbeta);
}


// ─────────────────────────────────────────────────────────────────────────────
// Hulpfuncties
// ─────────────────────────────────────────────────────────────────────────────
bool Motor::readSensor() {
  wire->beginTransmission(AS5600_ADDR);
  wire->write(0x0C);
  if (wire->endTransmission(false) != 0) return false;
  if (wire->requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint16_t high = wire->read();
  uint16_t low  = wire->read();
  rawAngle = ((high << 8) | low) & 0x0FFF;
  return true;
}

float Motor::readCurrent(int pin) {
  float voltage = (analogRead(pin) - adcMid) * (3.3f / 4095.0f);
  return voltage / (shunt * gain);
}

// Inverse Clarke + PWM
// valpha, vbeta: statorspanning in het stilstaande αβ-stelsel [V]
void Motor::applyVoltageAlphaBeta(float valpha, float vbeta) {
  // Inverse Clarke: (Vα, Vβ) → (Va, Vb, Vc)
  float Va =  valpha;
  float Vb = -0.5f * valpha + 0.8660254f * vbeta;
  float Vc = -0.5f * valpha - 0.8660254f * vbeta;

  ledcWrite(U, (int)constrain((Va / VOLTAGE_LIMIT * 0.5f + 0.5f) * PWM_MAX, 0, PWM_MAX));
  ledcWrite(V, (int)constrain((Vb / VOLTAGE_LIMIT * 0.5f + 0.5f) * PWM_MAX, 0, PWM_MAX));
  ledcWrite(W, (int)constrain((Vc / VOLTAGE_LIMIT * 0.5f + 0.5f) * PWM_MAX, 0, PWM_MAX));
}

void Motor::brake() {
  int mid = PWM_MAX / 2;
  ledcWrite(U, mid); ledcWrite(V, mid); ledcWrite(W, mid);
  _id_integral = _iq_integral = 0.0f;
}
