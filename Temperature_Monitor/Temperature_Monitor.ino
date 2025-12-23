#include <Wire.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "lib_I2CLCD.h"
#include <math.h>

// TODO rajouter un diode verte si consigne automatique
// fait
// TODO rajouter le changement de température après 1 minute
const unsigned long CHANGE_INTERVAL_MIN = 60000;
// TODO rajouter le changement de température après 1 seconde
const unsigned long CHANGE_INTERVAL_SEC = 1000;
// intervale de temperature utilisé (DEBUG versus actif)
unsigned long CHANGE_INTERVAL=CHANGE_INTERVAL_MIN;

// TODO ajouter une augmentation maximale 
//const float MAX_TEMP_RAMP_RATE = 0.1; // °C toutes les 10 secondes suivant l'interval utilisé
const float MAX_TEMP_RAMP_RATE = 2; // °C toutes les minutes suivant l'interval utilisé
 // Dernière mise à jour de la rampe
unsigned long lastRampUpdateTime = 0;

char seconde[] = "sec";
char minute[] = "min";

// Tableau des consignes forcée : {durée en ms, température en °C}
struct {
  unsigned long duration;
  float temp;
} forcedSetpoints[] = {
  {CHANGE_INTERVAL*1, 25.0}, // 5 sec à 22°C
  {CHANGE_INTERVAL*2, 27.0}, // 5 sec à 24°C 
  {CHANGE_INTERVAL*2, 29.0}, // 5 sec à 26°C
  {CHANGE_INTERVAL*3, 31.0}, // 5 sec à 28°C
  {CHANGE_INTERVAL*2, 25.0}, // 5 sec à 22°C

};

const int NUM_FORCED_SETPOINTS = sizeof(forcedSetpoints) / sizeof(forcedSetpoints[0]);
int currentSetpointIndex = 0; // Index de la consigne actuelle
unsigned long setpointStartTime = 0; // Temps de début de la consigne actuelle

// =====================================================
//  BTS7960 (chauffage uniquement)
//  RPWM -> D10, LPWM -> D9, R_EN -> D7, L_EN -> D8
// =====================================================
const int RPWM = 10;
const int LPWM = 9;
const int R_EN = 7;
const int L_EN = 8;

//  LEDs indication chauffage / refroidissement
// =====================================================
const int LED_VERTE = 4;
const int LED_ROUGE = 7;
const int LED_BLEUE = 8;


// (Optionnel mais recommandé) bouton STOP matériel
// Branche un bouton entre D2 et GND.
// Appui = arrêt (OFF). Relâché = régulation active.
const int STOP_BTN_PIN = 2;
int modeAuto=0;

// =====================================================
//  TEMPERATURE (2x NTC + consigne pot A2 + LCD I2C)
// =====================================================
const int NTC1_PIN = A0;  // capteur 1 sur le Peltier
const int NTC2_PIN = A1;  // capteur 2 (info)
const int PIN_SET = A2;   // potentiomètre consigne

const float R_SERIE = 10000.0;  // 10k
const float BETA = 3950.0;
const float T0_K = 273.15 + 25.0;
const float R0 = 10000.0;  // 10k @ 25°C

void adjustSetpointDurationsToRampRate() {

  Serial.println("==================================================");
  Serial.println("  AJUSTUMENTS DES CONSIGNES :");
  Serial.println("==================================================");
  for (int i = 1; i < NUM_FORCED_SETPOINTS; i++) {

    float duration= forcedSetpoints[i].duration;
    float previousTemp = forcedSetpoints[i-1].temp;
    float currentTemp = forcedSetpoints[i].temp;
    float tempDiff = abs(currentTemp - previousTemp);

    float valeur_de_test=tempDiff/(duration/CHANGE_INTERVAL);
    
    Serial.print("Palier ");
    Serial.print(i);
    Serial.print(" valeur_de_test: ");
    Serial.print(valeur_de_test);
    Serial.print(" seuil: ");
    Serial.print( MAX_TEMP_RAMP_RATE);    

    // Calcule la durée nécessaire pour respecter la rampe
    if ( valeur_de_test > MAX_TEMP_RAMP_RATE) {

      float ratio=valeur_de_test/MAX_TEMP_RAMP_RATE;
      Serial.print(" ratio: ");
      Serial.print( ratio);
      unsigned long NouvelleDuree = duration*ratio;
      Serial.print(" duration: ");
      Serial.print( duration);
      Serial.print(" NouvelleDuree: ");
      Serial.println( NouvelleDuree);
      forcedSetpoints[i].duration = (unsigned long)(NouvelleDuree);   
    }
  }
  Serial.println("==================================================");
}


float tempC_fromRaw(int raw) {
  if (raw <= 0) raw = 1;
  if (raw >= 1023) raw = 1022;

  float rntc = R_SERIE * float(1023 - raw) / float(raw);
  float invT = (1.0 / T0_K) + (1.0 / BETA) * log(rntc / R0);
  float T_K = 1.0 / invT;
  return T_K - 273.15;
}

float readRpoint() {
  
  int r = analogRead(PIN_SET);  
  if (r < 70) { r = 0;}
  if (r > 700) r = 700;
  return r;
}


// Map ~0..3.33V -> 15..45 °C (INCHANGÉ, comme ton code)
float readSetpointC() {
  int r = analogRead(PIN_SET);
  if (r < 70) r = 0;
  if (r > 700) r = 700;
  return 15.0 + (r / 700.0) * 30.0;
}

//  Fonctions LEDs
// =====================================================
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

void ledConsigneManuel() {
  digitalWrite(LED_VERTE, LOW); 
}

void ledConsigneAutomatique() {
  digitalWrite(LED_VERTE, HIGH); 
}

// =====================================================
//  PID (chauffage auto)
// =====================================================
double Input_Pid, Output_Pid, Setpoint;

// Réglage “un peu plus rapide” (point de départ)
// Si overshoot: baisse Kp ou Ki.
// Si trop lent: monte un peu Kp.
double Kp = 80.0;
double Ki = 0.0;
double Kd = 0.0;

PID pid(&Input_Pid, &Output_Pid, &Setpoint, Kp, Ki, Kd, DIRECT);

// Limites PWM (protège ton montage)
const int PWM_MAX = 200;

// Zone douce proche consigne (évite coups près du point)
const double SOFT_ZONE_C = 1.0;  // en dessous de Tc, on limite
const int PWM_SOFT_MAX = 70;

// Sécurités
const double HARD_OVERSHOOT_C = 0.3;  // si T1 > Tc + 0.3 => stop
const double ABS_MAX_C = 46.0;        // sécurité absolue

// Timing
static unsigned long lastMs = 0;
const unsigned long PERIOD_MS = 500;

unsigned int nouveau_pallier=0;
unsigned long TEMP_RAMP_INTERVAL=CHANGE_INTERVAL*9999;

// =====================================================
//  Chauffage (1 seul sens)
// =====================================================
void heaterOff() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);

  ledsOff();  // ✅ AJOUT MINIMUM : quand on coupe le pont, on coupe aussi les LEDs
}

void heaterOn(double cmd) {
  // cmd > 0 : chauffage (RPWM), cmd < 0 : refroidissement (LPWM)
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

// =====================================================
//  Setup
// =====================================================
void setup() {    

  // IMPORTANT: garder ce baud identique à Python
  Serial.begin(9600);
  // Display
  Serial.println("==================================================");
  Serial.println("|    SETUP");
  Serial.println("==================================================");
  Serial.println("  ARDUINO PARAMETERS:");
  Serial.println("==================================================");
  Serial.print("  LED_VERTE:  ");
  Serial.println(LED_VERTE);
  Serial.print("  LED_ROUGE:  ");
  Serial.println(LED_ROUGE);
  Serial.print("  LED_BLEUE:  ");
  Serial.println(LED_BLEUE);

  Serial.println("==================================================");
  Serial.println("  PID PARAMETERS: ");
  Serial.println("==================================================");
  Serial.print("  Mode: ");
  Serial.println("DEBUG");
  Serial.print("  CHANGE_INTERVAL:  ");
  Serial.println(CHANGE_INTERVAL);
  Serial.print("  MAX_TEMP_RAMP_RATE: ");
  Serial.println(MAX_TEMP_RAMP_RATE);
  Serial.print("  NUM_FORCED_SETPOINTS: ");
  Serial.println(NUM_FORCED_SETPOINTS);

  Serial.println("==================================================");
  Serial.println("  CONSIGNES non ajustées :");
  
  Serial.println("==================================================");
  for (int i = 0; i < NUM_FORCED_SETPOINTS; i++) {
    Serial.print("Palier ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(forcedSetpoints[i].duration / 1000);
    Serial.print(" sec à ");
    Serial.print(forcedSetpoints[i].temp);
    Serial.println("°C");
  }

    adjustSetpointDurationsToRampRate();

  Serial.println("==================================================");
  Serial.println("  CONSIGNES ajustées :");
  Serial.println("==================================================");
  for (int i = 0; i < NUM_FORCED_SETPOINTS; i++) {
    Serial.print("Palier ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(forcedSetpoints[i].duration / 1000);
    Serial.print(" sec à ");
    Serial.print(forcedSetpoints[i].temp);
    Serial.println("°C");
  }

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  pinMode(LED_VERTE, OUTPUT); 
  pinMode(LED_ROUGE, OUTPUT);  // ✅ AJOUT MINIMUM
  pinMode(LED_BLEUE, OUTPUT);  // ✅ AJOUT MINIMUM
  ledsOff();                   // ✅ (optionnel mais propre)

  pinMode(STOP_BTN_PIN, INPUT_PULLUP);  // bouton vers GND

  heaterOff();

  pinMode(PIN_SET, INPUT);

  Wire.begin();
  initLCD();
  clearDisplayLCD();
  printDisplayLCD("Code PID 23/12 16h41");
  delay(800);
  clearDisplayLCD();
 

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-PWM_MAX, PWM_MAX);  // chauffage (+) / refroidissement (-)
  pid.SetSampleTime(PERIOD_MS);
}

// =====================================================
//  Loop
// =====================================================
void loop() {
  unsigned long now = millis();
  if (now - lastMs < PERIOD_MS) return;
  lastMs = now;

  

  // Consigne par potentiomètre
  int rvalue = readRpoint();
  if (rvalue==0 && modeAuto==0) {   
    //on passe en mode automatique si on n'est 
    modeAuto=1;
    // on définit le temps 0 par rapport à ce moment 
    setpointStartTime=now;
    lastRampUpdateTime=now;
    // Index de la consigne actuelle
    currentSetpointIndex = 0; 
    Serial.println("==================================================");
    Serial.println("  RESETING setpointStartTime du mode AUTO");
    Serial.println("==================================================");
  }
  else if (rvalue==0 && modeAuto==1) { 
    //on reste en mode automatique mais on ne change pas le  setpointStartTime
    modeAuto=1;
  }
  else   {modeAuto=0;}

  // pour vérifier si on fonctionne en mode automatique ou manuel
  //Serial.print("modeAuto=");
  //Serial.println(modeAuto);
  
  // pour lire la consigne indiqué par le potentiomètre
  float Tc = readSetpointC(); 
  float Tcc = Tc;  //consigne classic issu du tableau // on lit le potentiometre aussi

  if (modeAuto==1) {

  Serial.println("-----------------------------------------------------------------------------------------");
  Serial.print("| Auto depuis : ");
  Serial.print(setpointStartTime);
  unsigned long elapsed = now - setpointStartTime;  
  Serial.print(" | Actif depuis: ");
  Serial.print(elapsed);
  
  float current_duration = forcedSetpoints[currentSetpointIndex].duration;
  Serial.print(" | Duration: ");
  Serial.print(current_duration);

       

  Serial.print(" | Index tableau consigne: ");
  Serial.print(currentSetpointIndex);
  // detecte un nouveau pallier
  Serial.print(" | Changement : ");
  Serial.println(nouveau_pallier);
  Serial.println("-----------------------------------------------------------------------------------------");
  // lit la consigne donnée par le tableau

  float autoTc=forcedSetpoints[currentSetpointIndex].temp;
  float autoTcRamp=forcedSetpoints[currentSetpointIndex].temp;
  // on assigne Tcc 
  Tcc=autoTc;

  // il faut ici déterminer la durée de la rampe et on doit le faire que si on arrive sur un nouveau pallier pas à chaque instant
  // ceci est vrai uniquement pour le second point
    if (currentSetpointIndex>0 && nouveau_pallier==1) {
      Serial.println("==================================================");
      Serial.println("  ESTIMATION DU TEMP_RAMP_INTERVAL :");
      Serial.println("==================================================");
      float previousTemp = forcedSetpoints[currentSetpointIndex-1].temp;
      float currentTemp = forcedSetpoints[currentSetpointIndex].temp;
      float tempDiff = abs(currentTemp - previousTemp);
      TEMP_RAMP_INTERVAL= tempDiff / MAX_TEMP_RAMP_RATE * CHANGE_INTERVAL;
      Serial.print(" tempDiff: ");
      Serial.print(tempDiff);
      Serial.print(" TEMP_RAMP_INTERVAL: ");
      Serial.println(TEMP_RAMP_INTERVAL);
      Serial.println("==================================================");
      // on remet à 0 ce booleen
      nouveau_pallier=0;
    }

    Serial.print(" now - lastRampUpdateTime: ");
    Serial.print(now - lastRampUpdateTime);
    Serial.print(" | TEMP_RAMP_INTERVAL: ");
    Serial.println(TEMP_RAMP_INTERVAL);
    
    // Applique la rampe de température ou non 
    if (currentSetpointIndex>0 && (now - lastRampUpdateTime) <= TEMP_RAMP_INTERVAL  ) {

      float previousTemp = forcedSetpoints[currentSetpointIndex-1].temp;  
      float currentTemp = forcedSetpoints[currentSetpointIndex].temp;  
      float tempDiff = currentTemp - previousTemp;
      Serial.print("  Rampe Active :");
      Serial.println( 1);
      float signe=1;
      if (tempDiff<0) {signe=-1;}
      //autoTcRamp = previousTemp + (now - lastRampUpdateTime)/1000*MAX_TEMP_RAMP_RATE*signe;   
      float progress=((float)now - (float)lastRampUpdateTime)/(float)TEMP_RAMP_INTERVAL * PI;
      Serial.print(" |  progress :");
      Serial.print( progress); 
      float sinusoidalProgress = 0.5 * (1.0 - cos(progress));
      Serial.print(" |  sinusoidalProgress :");
      Serial.print( sinusoidalProgress); 
      Serial.print(" |  autoTcRamp :");
      Serial.println( autoTcRamp); 
      autoTcRamp = previousTemp + sinusoidalProgress * tempDiff;
      Serial.print(" |  autoTcRamp :");
      Serial.println( autoTcRamp); 
      Tc=autoTcRamp;    
    }
    else
    { 
      Serial.print("  Rampe Active :");
      Serial.println( 0);
      Serial.print(" | autoTc:");
      Serial.println(autoTc);
      Tc=autoTc;  
    }

    // detecte l'index du pallier , on met cela à la fin pour ne pas prendre de retard
    // Si le temps écoulé dépasse la durée de la consigne actuelle, on passe à la suivante
    if (elapsed >= forcedSetpoints[currentSetpointIndex].duration) {
      currentSetpointIndex++;
      nouveau_pallier=1;
      if (currentSetpointIndex >= NUM_FORCED_SETPOINTS) {
        currentSetpointIndex = 0; // Recommence depuis le début du tableau
      }
      setpointStartTime = now; // Réinitialise le temps de départ
      lastRampUpdateTime= now;
    }
    
  }
  
  // Lecture températures
  analogRead(NTC1_PIN);
  delayMicroseconds(200);
  int raw1 = analogRead(NTC1_PIN);

  analogRead(NTC2_PIN);
  delayMicroseconds(200);
  int raw2 = analogRead(NTC2_PIN);

  float T1 = tempC_fromRaw(raw1);
  float T2 = tempC_fromRaw(raw2);
 

  // STOP matériel
  bool stopPressed = (digitalRead(STOP_BTN_PIN) == LOW);

  // Prépare PID
  Input_Pid = T1;
  Setpoint = Tc;
  

  // Sécurités et régulation
  if (stopPressed || isnan(T1) || (T1 > ABS_MAX_C) || false /*overshoot handled by cooling*/) {
    Output_Pid = 0;
    heaterOff();
  } else {


    pid.Compute();

    // Limitation douce proche consigne (pour éviter d'arriver trop vite)
    double err = Tc - T1;

    if (Output_Pid < 0) { Output_Pid = 1.25 * Output_Pid; }

    if (Output_Pid > 200) { Output_Pid = 200; }
    if (Output_Pid < -200) { Output_Pid = -200; }

    // Deadband : on coupe si très proche de la consigne
    //if (fabs(err) < 0.2) {
    //  Output_Pid = 0;
    //  heaterOff();
    //}
    //else
    //{
    // Zone douce : limite la magnitude |OUT| quand on est proche de la consigne
    //  if (fabs(err) < SOFT_ZONE_C && fabs(Output_Pid) > PWM_SOFT_MAX) {
    //    Output_Pid = (Output_Pid > 0 ? PWM_SOFT_MAX : -PWM_SOFT_MAX);
    //  }
    //
    //}

    heaterOn(Output_Pid);
  }

  //  LEDs : placées ICI pour refléter l'état FINAL (après deadband/heaterOn/heaterOff)
  if (Output_Pid > 2) {
    ledChauffage();
  } else if (Output_Pid < -2) {
    ledRefroidissement();
  } else {
    ledsOff();
  }

  if (rvalue==0) {
     ledConsigneAutomatique();}
  else  {
     ledConsigneManuel(); 
     }

  // LCD (C/T1/T2 inchangé + ajout OUT en bas)
  locateCursorLCD(0, 0);

    if (modeAuto==1) {
    printDisplayLCD("C*:");
  } else {
    printDisplayLCD("C:");
  }
  printFloatLCD(Tc, 6, 2);
  printDisplayLCD("C   ");
  printDisplayLCD(" r:");
  printIntLCD(rvalue, 4); // Affiche la valeur brute r (ex: "XXXX")
  printDisplayLCD("   "); // Efface les éventuels résidus

  

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

  // Sortie propre pour Python (2 courbes)
  Serial.print("Tcc:");
  Serial.print(Tcc, 2);
  Serial.print("\tTc:");
  Serial.print(Tc, 2);
  Serial.print("\tT1:");
  Serial.print(T1, 2);
  Serial.print("\tsortiePID:");
  Serial.println(Output_Pid, 2);
}
