// version 29 juin 15h20
#include <Wire.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "lib_I2CLCD.h"
#include <math.h>

// Base de temps (1000 ms = 1 seconde)
const unsigned long CHANGE_INTERVAL_MIN = 60000;
const unsigned long CHANGE_INTERVAL_SEC = 1000;
unsigned long CHANGE_INTERVAL = CHANGE_INTERVAL_SEC; 

const float MAX_TEMP_RAMP_RATE = 2; 
unsigned long lastRampUpdateTime = 0;

char seconde[] = "sec";
char minute[] = "min";

// =================================================================
//  TABLEAU DES TEMPS DE PALIER (Profil 4 paliers + maintien initial)
// =================================================================
struct {
  unsigned long duration;
  float temp_offset; // Delta par rapport à la température de départ capturée
} forcedSetpoints[] = {
  {CHANGE_INTERVAL * 180, 2.0},  // Palier 1 : +2°C en 3 min (180s)
  {CHANGE_INTERVAL * 180, 4.0},  // Palier 2 : +2°C supplémentaires en 3 min (180s)
  {CHANGE_INTERVAL * 180, 6.0},  // Palier 3 : +2°C supplémentaires en 3 min (180s)
  {CHANGE_INTERVAL * 300, 0.0}, // Palier 4 : Retour à l'origine (-6°C) en 5 min (300s)
  {CHANGE_INTERVAL * 180, 0.0}
};

float alpha = 0.2; 
float Output_Pid_filtered = 0;
float tau = 0.2; 
float T1 = 20;

const int NUM_FORCED_SETPOINTS = sizeof(forcedSetpoints) / sizeof(forcedSetpoints[0]);
int currentSetpointIndex = 0; 
unsigned long setpointStartTime = 0; 
unsigned long palierStartTime = 0;   

// Pins Pont en H BTS7960
const int RPWM = 10;
const int LPWM = 9;
const int R_EN = 7;
const int L_EN = 8;

// Pins LEDs
const int LED_VERTE = 4; 
const int LED_ROUGE = 5; 
const int LED_BLEUE = 6; 

const int STOP_BTN_PIN = 2;
int modeAuto = 0;

// Variables de gestion des phases d'initialisation Auto
unsigned long autoStartTime = 0;
bool tempCaptured = false;
float tempDepartStabilisee = 0.0;

const int NTC1_PIN = A0;  
const int NTC2_PIN = A1;  
const int PIN_SET = A2;   

const float R_SERIE = 10000.0;  
const float BETA = 3950.0;
const float T0_K = 273.15 + 25.0;
const float R0 = 10000.0;  

// =====================================================
//  PID - CONFIGURATION MÉTROLOGIQUE
// =====================================================
double Input_Pid, Output_Pid, Setpoint;

// CORRECTION FIN DE PARCOURS : Ki augmenté pour éliminer l'offset statique résiduel
double Kp = 16.0;  
double Ki = 0.50;  
double Kd = 14.0;  

// Gain d'anticipation (Feed-Forward)
const double KF = 120.0; 
double feedForward = 0;

PID pid(&Input_Pid, &Output_Pid, &Setpoint, Kp, Ki, Kd, DIRECT);

// CORRECTION FIN DE PARCOURS : PWM_MAX augmenté pour donner plus de puissance de refroidissement si nécessaire
const int PWM_MAX = 150; 
const double ABS_MAX_C = 46.0;        
static unsigned long lastMs = 0;
const unsigned long PERIOD_MS = 500;

unsigned int nouveau_pallier = 0;
unsigned long TEMP_RAMP_INTERVAL = CHANGE_INTERVAL * 9999;

void adjustSetpointDurationsToRampRate() {
  // Fonction laissée pour la compatibilité de structure
}

float tempC_fromRaw(int raw) {
  if (raw <= 0) raw = 1;
  if (raw >= 1023) raw = 1022;
  float rntc = R_SERIE * float(1023 - raw) / float(raw);
  float invT = (1.0 / T0_K) + (1.0 / BETA) * log(rntc / R0);
  return (1.0 / invT) - 273.15;
}

float readRpoint() {
  int r = analogRead(PIN_SET);  
  if (r < 70) r = 0;
  if (r > 700) r = 700;
  return r;
}

float readSetpointC() {
  int r = analogRead(PIN_SET);
  if (r < 70) r = 0;
  if (r > 700) r = 700;
  return 15.0 + (r / 700.0) * 30.0;
}

void ledsOff() {
  digitalWrite(LED_ROUGE, LOW);
  digitalWrite(LED_BLEUE, LOW);
}

void ledChauffage() {
  digitalWrite(LED_ROUGE, HIGH);
  digitalWrite(LED_BLEUE, LOW);
}

void ledRefroidissement() {
  digitalWrite(LED_ROUGE, LOW);
  digitalWrite(LED_BLEUE, HIGH);
}

void ledConsigneManuel() { digitalWrite(LED_VERTE, LOW); }
void ledConsigneAutomatique() { digitalWrite(LED_VERTE, HIGH); }

void heaterOff() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  ledsOff();  
}

void heaterOn(double cmd) {
  int pwm = constrain((int)round(fabs(cmd)), 0, PWM_MAX);
  if (cmd > 2) {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (cmd < -2) {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  } else {
    heaterOff();
  }
}

void setup() {    
  Serial.begin(9600);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(LED_VERTE, OUTPUT); 
  pinMode(LED_ROUGE, OUTPUT);  
  pinMode(LED_BLEUE, OUTPUT);  
  
  ledsOff();                    
  pinMode(STOP_BTN_PIN, INPUT_PULLUP);  
  heaterOff();
  pinMode(PIN_SET, INPUT);

  Wire.begin();
  initLCD();
  clearDisplayLCD();
  printDisplayLCD("29 juin 14h33");
  delay(800);
  clearDisplayLCD();
 
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-PWM_MAX, PWM_MAX);  
  pid.SetSampleTime(PERIOD_MS);
}

void loop() {
  unsigned long now = millis();
  if (now - lastMs < PERIOD_MS) return;
  lastMs = now;

  int raw1 = analogRead(NTC1_PIN);
  int raw2 = analogRead(NTC2_PIN);
  float T1_rawValue = tempC_fromRaw(raw1);  
  T1 = (tau * T1_rawValue) + ((1.0 - tau) * T1); 
  float T2 = tempC_fromRaw(raw2);

  int rvalue = readRpoint();
  if (rvalue == 0 && modeAuto == 0) {   
    modeAuto = 1;
    autoStartTime = now;  
    tempCaptured = false;
    currentSetpointIndex = 0; 
    Serial.println("-> [AUTO] Phase 1 : Stabilisation Initiale NTC1 (1 min)...");
  }
  else if (rvalue == 0 && modeAuto == 1) { 
    modeAuto = 1;
  }
  else { 
    modeAuto = 0; 
    tempCaptured = false;
  }

  float Tc = readSetpointC(); 
  float Tcc = Tc;  
  feedForward = 0; 

  if (modeAuto == 1) {
    unsigned long elapsedAuto = now - autoStartTime;

    // 1 minute de stabilisation initiale hors régulation
    if (elapsedAuto < 60000) {
      Tc = -99.0;  
      Tcc = T1;
      heaterOff(); 
    } 
    else {
      // CORRECTION DÉMARRAGE : Initialisation propre du timing de la première rampe à t = 60s
      if (!tempCaptured) {
        tempDepartStabilisee = T1; 
        tempCaptured = true;
        
        palierStartTime = now;
        lastRampUpdateTime = now; 
        nouveau_pallier = 1;
      }

      unsigned long timeInCurrentPalier = now - palierStartTime;
      
      // Calcul des cibles de température absolue basées sur l'origine capturée
      float previousTempTarget = tempDepartStabilisee;
      if (currentSetpointIndex > 0) {
        previousTempTarget = tempDepartStabilisee + forcedSetpoints[currentSetpointIndex - 1].temp_offset;
      }
      float currentTempTarget = tempDepartStabilisee + forcedSetpoints[currentSetpointIndex].temp_offset;
      
      Tcc = currentTempTarget; 
      TEMP_RAMP_INTERVAL = forcedSetpoints[currentSetpointIndex].duration;

      // CORRECTION DÉMARRAGE : Calcul sinusoïdal basé sur le temps local du palier (timeInCurrentPalier)
      if (timeInCurrentPalier <= TEMP_RAMP_INTERVAL) {
        float tempDiff = currentTempTarget - previousTempTarget;
        float progress = ((float)timeInCurrentPalier) / (float)TEMP_RAMP_INTERVAL * PI;
        float sinusoidalProgress = 0.5 * (1.0 - cos(progress));
        
        Tc = previousTempTarget + sinusoidalProgress * tempDiff;    

        // Calcul dynamique du Feed-Forward associé
        float rampSpeed = (sin(progress) * tempDiff * PI) / (TEMP_RAMP_INTERVAL / 1000.0);
        feedForward = rampSpeed * KF;
      }
      else { 
        Tc = currentTempTarget;  
        feedForward = 0;
      }

      // Transition automatique au palier suivant
      if (timeInCurrentPalier >= forcedSetpoints[currentSetpointIndex].duration) {
        currentSetpointIndex++;
        nouveau_pallier = 1;
        if (currentSetpointIndex >= NUM_FORCED_SETPOINTS) {
          currentSetpointIndex = 0; // Reboucle le cycle complet
        }
        palierStartTime = now;
        lastRampUpdateTime = now;
      }
    }
  }

  bool stopPressed = (digitalRead(STOP_BTN_PIN) == LOW);

  Input_Pid = T1;
  Setpoint = Tc; 
  
  // --- SÉCURITÉS ET CALCUL DE RÉGULATION ---
  if (stopPressed || isnan(T1) || (T1 > ABS_MAX_C)) {
    Output_Pid = 0;
    heaterOff();
  } else {
    if (modeAuto == 1 && (now - autoStartTime) < 60000) {
      Output_Pid = 0;
    } else {
      pid.Compute();
      Output_Pid += feedForward;

      // RÉGULATION CONTINUE : L'action intégrale reste active pour gommer l'offset.
      Output_Pid = constrain(Output_Pid, -PWM_MAX, PWM_MAX);
      heaterOn(Output_Pid);
    }
  }

  // Éclairage LEDs
  if (Output_Pid > 2) ledChauffage();
  else if (Output_Pid < -2) ledRefroidissement();
  else ledsOff();

  if (rvalue == 0) ledConsigneAutomatique();
  else ledConsigneManuel();

  // Gestion de l'affichage LCD
  locateCursorLCD(0, 0);
  if (modeAuto == 1) { 
    if ((now - autoStartTime) < 60000) {
      printDisplayLCD("C*:STABILIS."); 
    } else {
      printDisplayLCD("C*:"); 
      printFloatLCD(Tc, 6, 2);
      printDisplayLCD("C");
    }
  } else { 
    printDisplayLCD("C:"); 
    printFloatLCD(Tc, 6, 2);
    printDisplayLCD("C");
  }
  printDisplayLCD(" r:");
  printIntLCD(rvalue, 4); 

  locateCursorLCD(0, 1);
  printDisplayLCD("T1:");
  printFloatLCD(T1, 6, 2);
  printDisplayLCD("C   ");

  locateCursorLCD(0, 2);
  printDisplayLCD("T2:");
  printFloatLCD(T2, 6, 2);
  printDisplayLCD("C   ");

  locateCursorLCD(0, 3);
  printDisplayLCD("OUT:");
  printFloatLCD(Output_Pid, 5, 0);
  printDisplayLCD("   ");

  // Envoi des données vers le script Python (2 Hz)
  Serial.print("Tcc:");
  Serial.print(Tcc, 2);
  Serial.print("\tTc:");
  Serial.print(Tc, 2);
  Serial.print("\tT1:");
  Serial.print(T1, 2);
  Serial.print("\tsortiePID:");
  Serial.println(Output_Pid, 2);
}