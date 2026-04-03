
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Wire.h>

// ─── PWM-instellingen ─────────────────────────────────────────────────────────
#define PWM_FREQ    20000   // [Hz]
#define PWM_RES     10      // [bit]  0–1023
#define PWM_MAX     1023

// ─── AS5600 encoder ───────────────────────────────────────────────────────────
#define AS5600_ADDR         0x36
#define ENCODER_TIMEOUT_MS  5

// ─── Spanningslimiet ──────────────────────────────────────────────────────────
// Maximale fasespanning. 4S LiPo ≈ 14.8 V → Vbus/2 ≈ 7.4 V.
#define VOLTAGE_LIMIT   7.0f    // [V]

// ─── Uitlijnspanning ──────────────────────────────────────────────────────────
#define ALIGN_VOLTAGE   3.0f    // [V]

// ─── Tijdstap & snelheidsfilter ───────────────────────────────────────────────
#define MOTOR_DT        0.002f  // [s]  500 Hz
#define VELOCITY_ALPHA  0.80f   // low-pass filter coefficient

// ─── Stroombeveiliging ────────────────────────────────────────────────────────
#define MAX_CURRENT_A   6.0f    // [A]

// ─────────────────────────────────────────────────────────────────────────────
class Motor {
public:
  Motor(int u, int v, int w, int en,
        int ia, int ib,
        TwoWire *wbus,
        int poles);

  void begin();
  void alignStart();
  void alignFinish();

  // ── Twee aansturingsmodi ─────────────────────────────────────────────────

  // Open-loop: geef direct een fasespanning [V] op.
  // Handig om te controleren of de motor draait en de encoder werkt,
  // zonder de complexiteit van de stroomregelaar.
  // voltage: bereik [-VOLTAGE_LIMIT, +VOLTAGE_LIMIT]
  void loopOpenLoop(float voltage);

  // FOC gesloten lus: geef een gewenste koppelstroom [A] op.
  // De interne PI-regelaars regelen Id op 0 en Iq op iqTarget.
  // iqTarget: typisch ±0.5..2 A voor deze motor
  void loopFOC(float iqTarget);

  // ── FOC gains aanpassen tijdens runtime ──────────────────────────────────
  // Handig voor het afstellen via de seriële monitor.
  void setFocGains(float kp, float ki);

  // ── Status & telemetrie ──────────────────────────────────────────────────
  bool  isOk()            const { return _encoderOk && !_overcurrent; }
  bool  hasEncoderError() const { return !_encoderOk; }
  bool  isOvercurrent()   const { return _overcurrent; }

  float getAngle()        const { return lastMechAngle; }   // [rad]
  float getVelocity()     const { return _velocity; }       // [rad/s]
  float getAvgCurrent()   const { return avgI; }            // [A] gefilterd
  float getPeakCurrent()  const { return peakI; }           // [A] direct
  float getId()           const { return _id; }             // [A] d-as stroom
  float getIq()           const { return _iq; }             // [A] q-as koppelstroom

private:
  int U, V, W, EN;
  int IA_PIN, IB_PIN;
  TwoWire *wire;
  int polePairs;

  uint16_t rawAngle;
  float    lastMechAngle;
  float    prevMechAngle;
  float    offset;
  float    _velocity;

  float avgIa, avgIb, avgIc, avgI, peakI;

  // FOC toestandsvariabelen
  float _id, _iq;
  float _id_integral, _iq_integral;

  // FOC gains — instelbaar via setFocGains()
  float _focKp;      // [V/A]
  float _focKi;      // [V/(A·s)]
  float _focIntMax;  // [V]  anti-windup limiet

  // Shunt-versterker (MKS-ESP32FOC V2.0)
  float shunt;    // [Ω]
  float gain;     // [-]
  float adcMid;   // [ADC-ticks]

  bool _encoderOk;
  bool _overcurrent;

  // Gemeenschappelijke stap 1–3 voor beide loopXxx-methoden:
  // encoder + snelheid + stromen + overstroom-check.
  // Geeft false als de motor gestopt moet worden (encoder-fout of overstroom).
  bool commonUpdate(float &Ia, float &Ib, float &Ic);

  bool  readSensor();
  float readCurrent(int pin);
  void  applyVoltageAlphaBeta(float valpha, float vbeta);
  void  brake();
};

#endif // MOTOR_H
