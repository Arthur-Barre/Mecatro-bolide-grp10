/*Etat du script :
/Connexion multiplexeur : OK
/Connexion 2 Encodeurs : OK
/Connexion 2 Moteurs : OK
/Connexion Wifi : OUI
/Connexion SensorBar : OK
/MODE AUTONOME : Pas besoin de MATLAB
*/

#include "MecatroUtils.h"

// Include the SensorBar library (for Line Follower) 
#include "sensorbar.h"
// Include the AS5600 library (for the encoders) 
#include "AS5600.h"
// Include Sparkfun I2C Mux (for multiplexer)
#include "SparkFun_I2C_Mux_Arduino_Library.h"
QWIICMUX multiplexer;
AS5600 rightEncoder(&Wire1);
AS5600 leftEncoder(&Wire1);

// Header for I2C communication
#include "Wire.h"

// Define the name and password of the wifi network
#define WIFI_SSID "Group10"
#define WIFI_PASSWRD "password1234"

// Define the control loop period, in ms.
#define CONTROL_LOOP_PERIOD 3
#define TAU 100.0e-3

// Définition des Pins
SensorBar mySensorBar(0x3E);
#define SENSORBAR_PIN 4
#define RIGHT_ENCODER_PIN 7
#define LEFT_ENCODER_PIN 0

#define PI 3.1415926
#define DEG_TO_RAD 2*PI/360

// ===== GAINS DU CONTROLEUR (MODIFIABLES) =====
float Kp = 15.0;        // Gain proportionnel vitesse
float Ki = 8.0;         // Gain intégral vitesse
float K_moins = 15.0;   // Gain proportionnel direction
float K_lambda = 8.0;   // Gain intégral direction
float Vbatt = 11.2;     // Tension batterie

// Définition des variables utiles
float PosSensorBar = 0.0;
float CumulativePosRight_0 = 0.0;
float CumulativePosLeft_0 = 0.0;

float speed=0.0;
float delta=0.0;
float compteur=0;

float lambda = 0.0;

float RawAngle[2]={0.0,0.0};
float RawAngle_new[2]={0.0,0.0};

float OmegaD = 0.0;
float OmegaG = 0.0;
float OmegaD_prev = 0.0;
float OmegaG_prev = 0.0;

// Coeff du filtre dérivateur
float a1 =-(1 - 2*TAU / (CONTROL_LOOP_PERIOD*0.001));
float b1 = (1 + 2*TAU / (CONTROL_LOOP_PERIOD*0.001));

// Paramètres du système
float p = 0.04;           // rayon des roues, en m
float u_barre = 0.1;      // vitesse d'équilibre souhaitée, en m/s (augmentée)
float U_plus_barre = 6.0; // Tension d'équilibre (augmentée)

// Variables d'état du controleur
float dt = CONTROL_LOOP_PERIOD*0.001;
float eta_u = 0.0;
float eta_lambda = 0.0;
float u = 0.0;

// Les Tensions utiles
float U;
float U_plus = 0.0;
float U_moins = 0.0;


void Controleur(float lambda, float OmegaD, float OmegaG, float *U_plus, float *U_moins, float *u) {
  // Calcul de U_moins via PI (contrôle direction)
  eta_lambda += lambda*dt;
  float dU_moins = -K_moins * lambda - K_lambda*eta_lambda;  // coeffs positifs ? car si lambda < 0, tourne vers la gauche, 
  *U_moins = dU_moins;   // U_moins_barre = 0 
  *U_moins = constrain(*U_moins, -12.0, 12.0);

  // Calcul de U_plus via PI (contrôle vitesse)
  *u = p * (OmegaD + OmegaG) / 2.0;
  float du = u_barre - *u;
  eta_u += du * dt;
  float dU_plus = -Ki * eta_u - Kp * du;  // coeffs négatifs pour augmenter U_plus si on est sous u_barre
  *U_plus = U_plus_barre //+ dU_plus;
  *U_plus = constrain(*U_plus, -12.0, 12.0);
}

void setup() {
  Serial.begin(230400);
  Wire1.begin();

  Serial.println("=== ROBOT SUIVEUR DE LIGNE ===");
  Serial.println("Mode autonome - pas besoin de MATLAB");

  if (!multiplexer.begin(0x70,Wire1)){
    Serial.println("ERREUR: multiplexeur non fonctionnel");
    while(1); // Bloque si erreur critique
  }
  else {
    Serial.println("Multiplexeur OK");
    bool isInit = true;

    // Configuration SensorBar
    mySensorBar.clearBarStrobe();
    mySensorBar.clearInvertBits(); // Ligne NOIRE sur fond BLANC
    // Si ligne blanche sur fond noir, décommentez la ligne suivante :
    // mySensorBar.setInvertBits();
    
    multiplexer.setPort(SENSORBAR_PIN);
    uint8_t returnStatus = mySensorBar.begin();

    if(!returnStatus)
    {
      Serial.println("ERREUR: SensorBar non fonctionnelle");
      isInit = false;
    }
    else {
      Serial.println("SensorBar OK");
    }
  
    // Init encodeur droit
    multiplexer.setPort(RIGHT_ENCODER_PIN);
    rightEncoder.begin();
    if (!rightEncoder.isConnected())
    {
      Serial.println("ERREUR: encodeur droit non connecté");
      isInit = false;
    }
    else {
      Serial.println("Encodeur droit OK");
    }
 
    // Init encodeur gauche
    multiplexer.setPort(LEFT_ENCODER_PIN);
    leftEncoder.begin();
    if (!leftEncoder.isConnected())
    {
      Serial.println("ERREUR: encodeur gauche non connecté");
      isInit = false;
    }
    else {
      Serial.println("Encodeur gauche OK");
    }

    // Initialisation positions
    multiplexer.setPort(RIGHT_ENCODER_PIN);
    RawAngle[0] = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    CumulativePosRight_0 = RawAngle[0];
    
    multiplexer.setPort(LEFT_ENCODER_PIN);
    RawAngle[1] = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    CumulativePosLeft_0 = RawAngle[1];

    OmegaD_prev = 0;
    OmegaG_prev = 0;

    if (isInit)
    {
      Serial.println("=== INITIALISATION REUSSIE ===");
      Serial.println("Configuration des parametres:");
      Serial.print("  K_moins = "); Serial.println(K_moins);
      Serial.print("  K_lambda = "); Serial.println(K_lambda);
      Serial.print("  Kp = "); Serial.println(Kp);
      Serial.print("  Ki = "); Serial.println(Ki);
      Serial.print("  U_plus_barre = "); Serial.println(U_plus_barre);
      Serial.print("  Vitesse cible = "); Serial.print(u_barre); Serial.println(" m/s");
      
      // Test SensorBar
      Serial.println("\nTest SensorBar (5 secondes):");
      Serial.println("Placez le robot sur la ligne et observez les valeurs");
      multiplexer.setPort(SENSORBAR_PIN);
      for(int i=0; i<50; i++) {
        int pos = mySensorBar.getPosition();
        uint8_t density = mySensorBar.getDensity();
        Serial.print("Position: "); 
        Serial.print(pos);
        Serial.print(" | Density: "); 
        Serial.println(density);
        delay(100);
      }
      
      Serial.println("\n=== DEMARRAGE DANS 3 SECONDES ===");
      delay(3000);
      
      // Initialisation télémétrie (optionnelle)
      unsigned int const nVariables = 10;
      String variableNames[nVariables] = {"angleDroite","angleGauche","Omega Droite","Omega Gauche","SensorBar","lambda","U","u","U_plus","U_moins"};
      mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);

      // PAS DE RECEPTION DE GAINS - on utilise les valeurs hardcodées
      
      Wire1.setClock(400000);
      mecatro::configureArduino(CONTROL_LOOP_PERIOD);
      
      Serial.println("=== ROBOT EN MARCHE ===");
    }
    else {
      Serial.println("ERREUR: Initialisation echouee");
      while(1); // Bloque si erreur
    }
  }
}

void loop() {
  mecatro::run();
}

void mecatro::controlLoop()
{
  // Lecture SensorBar
  multiplexer.setPort(SENSORBAR_PIN);
  PosSensorBar = mySensorBar.getPosition()*0.0458/127;
  lambda = PosSensorBar; // - ?

  // Encodeur droit
  multiplexer.setPort(RIGHT_ENCODER_PIN);
  RawAngle_new[0] = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;

  // Encodeur gauche
  multiplexer.setPort(LEFT_ENCODER_PIN);
  RawAngle_new[1] = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
 
  // Calcul vitesses filtrées
  OmegaD = (2.0/(CONTROL_LOOP_PERIOD*0.001)*(RawAngle_new[0]-RawAngle[0])*DEG_TO_RAD + a1 * OmegaD_prev)/b1;
  OmegaG = (2.0/(CONTROL_LOOP_PERIOD*0.001)*(RawAngle_new[1]-RawAngle[1])*DEG_TO_RAD + a1 * OmegaG_prev)/b1;

  // Appel au controleur
  Controleur(lambda, OmegaD, OmegaG, &U_plus, &U_moins, &u);
  
  // Calcul duty cycles
  float delta_d = (U_plus + U_moins)/Vbatt;
  float delta_g = (U_plus - U_moins)/Vbatt;

  // Commande moteurs
  mecatro::setMotorDutyCycle(delta_g, delta_d);

  // Télémétrie
  mecatro::log(0, RawAngle[0]-CumulativePosRight_0);
  mecatro::log(1, RawAngle[1]-CumulativePosLeft_0);
  mecatro::log(2, OmegaD);
  mecatro::log(3, OmegaG);
  mecatro::log(4, PosSensorBar); 
  mecatro::log(5, lambda);
  mecatro::log(6, U);
  mecatro::log(7, u);
  mecatro::log(8, U_plus);
  mecatro::log(9, U_moins);
  
  // Mise à jour valeurs précédentes
  for (int i = 0; i < 2; i++) {
    RawAngle[i] = RawAngle_new[i];
  }
  OmegaG_prev = OmegaG;
  OmegaD_prev = OmegaD;
}
