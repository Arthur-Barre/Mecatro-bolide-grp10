/*Etat du script :
/Connexion multiplexeur : OK
/Connexion 2 Encodeurs : OK
/Connexion 2 Moteurs : OK
/Connexion Wifi : OUI
/Connexion SensorBar : OK
*/

#include "MecatroUtils.h"

// Include the SensorBar library (for Line Follower) 
#include "sensorbar.h"
// Include the AS5600 library (for the encoders) 
#include "AS5600.h"
// Include Sparkfun I2C Mux (for multiplexer)
#include "SparkFun_I2C_Mux_Arduino_Library.h"
QWIICMUX multiplexer;  // Init le multiplexer

// Définition des capteurs d'angles
AS5600 rightEncoder(&Wire1);
AS5600 leftEncoder(&Wire1);

// Header for I2C communication
#include "Wire.h"

// Define the name and password of the wifi network
#define WIFI_SSID "Group10"
#define WIFI_PASSWRD "password1234"

// Définition des constantes de temps utiles
#define CONTROL_LOOP_PERIOD 3  // en ms
#define TAU 100.0e-3  //en s

// Définition des Pins
SensorBar mySensorBar(0x3E);
#define SENSORBAR_PIN 4    // SDA : blue | SCL : yellow
#define RIGHT_ENCODER_PIN 7  // branchement sur le Main côté PIN 4
#define LEFT_ENCODER_PIN 0

#define PI 3.1415926
#define DEG_TO_RAD 2*PI/360

// Les gains à recevoir de MatLab
float Kp = -1;
float Ki = -1;
float K_moins = 1;
float K_lambda = 1;
float K_psi = 1;
float Vbatt = 11.2;

// Définition des variables utiles
float PosSensorBar = 0.0;
float CumulativePosRight_0 = 0.0;
float CumulativePosLeft_0 = 0.0;

float speed=0.0;
float compteur = 0;

float lambda = 0.0;
float psi = 0.0;

// Tableaux de stockage des angles actuels et précédents
float RawAngle[2]={0.0,0.0};
float RawAngle_new[2]={0.0,0.0};

float OmegaD = 0.0;
float OmegaG = 0.0;
float OmegaD_prev = 0.0;  // les vitesses précédentes
float OmegaG_prev = 0.0;

// Coefficients du filtre dérivateur
float a1 =-(1 - 2*TAU / (CONTROL_LOOP_PERIOD*0.001));
float b1 = (1 + 2*TAU / (CONTROL_LOOP_PERIOD*0.001));

// Paramètres du système
float p = 0.04;          // rayon des roues, en m
float l = 0.187;         // distance entre les roues, en m
float u_barre = 0.1;     // vitesse d'équilibre souhaitée, en m/s
float k = 0.338;         // constante du moteur

float coef=1;            // coefficients rectificatif du moteur droit

// Variables d'état du controleur
float dt = CONTROL_LOOP_PERIOD*0.001;   // pas de temps en secondes 
float eta_u = 0.0;   // intégrateur
float eta_lambda = 0.0;
float u = 0.0;  // la vitesse du robot

// Les Tensions utiles
float U_plus = 0.0;
float U_plus_barre=1;  // vraie initialisation plus tard, U_moins_barre = 0
float U_moins = 0.0;


void Controleur(float lambda, float OmegaD, float OmegaG, float *U_plus, float *U_moins, float *u, float psi) {  // on modifie via des références les U, appel via &U
  // Calcul de U_moins via PI (écart à la ligne)
  eta_lambda += CONTROL_LOOP_PERIOD*0.001*lambda*dt;
  float dU_moins = -K_moins * lambda - K_lambda*eta_lambda - K_psi*psi;  // coeff positifs  car si lambda<0, on veut augmenter Ur donc U_moins
  *U_moins = dU_moins;                                                   // U_moins_barre=0
  *U_moins = constrain(*U_moins, -12.0, 12.0);

  // Calcul de U_plus via PI (translation)
  *u = p * (OmegaD + OmegaG) / 2.0;  // vitesse instantanée
  float du = u_barre - *u;                 // écart 
  eta_u += du * dt;                        // intégration
  float dU_plus = -Ki * eta_u - Kp * du;   // écart à la valeur d'équilibre, coeff négatifs car si du<0 ie vitesse trop faible, on veut augmenter U_plus
  *U_plus = U_plus_barre + dU_plus;   
  *U_plus = constrain(*U_plus, -12.0, 12.0); // évite la saturation
}

void setup() {
  Serial.begin(230400);
  Wire1.begin();   // communication I2C avec le port QWIIC

  if (!multiplexer.begin(0x70,Wire1)){
    Serial.println("Errorc: multiplexer non fonctionnel");
  }
  else {
    Serial.println("Multiplexer fonctionnel");
    bool isInit = true;

    //Command for the IR to run all the time
    mySensorBar.clearBarStrobe();
    mySensorBar.clearInvertBits();
   
    multiplexer.setPort(SENSORBAR_PIN);
    uint8_t returnStatus = mySensorBar.begin();

    if(!returnStatus)
    {
      Serial.println("sx1509 IC communication FAILED!");
      isInit = false;
    }
  
    multiplexer.setPort(RIGHT_ENCODER_PIN);
    rightEncoder.begin();
    if (!rightEncoder.isConnected())
    {
      Serial.println("Error: could not connect to right encoder. Check wiring.");
      isInit = false;
    }
 
    multiplexer.setPort(LEFT_ENCODER_PIN);
    leftEncoder.begin();
    if (!leftEncoder.isConnected())
    {
      Serial.println("Error: could not connect to left encoder. Check wiring.");
      isInit = false;
    }

    // Initialise AVANT de filtrer pour éviter d'avoir une vitesse initiale aberrante et faussée (l'écart initial Delta Thêta serait environ égal à Thêta[N+1] et non à zéro)
    multiplexer.setPort(RIGHT_ENCODER_PIN);
    RawAngle[0] = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    CumulativePosRight_0 = RawAngle[0];   // la valeur initiale, à soustraire pour commencer à 0

    multiplexer.setPort(LEFT_ENCODER_PIN);
    RawAngle[1] = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    CumulativePosLeft_0 = RawAngle[1];

    OmegaD_prev = 0;
    OmegaG_prev = 0;

    if (isInit)
    {
      // Initialisation télémétrie
      unsigned int const nVariables = 10;
      String variableNames[nVariables] = {"angleDroite","angleGauche","Omega Droite","Omega Gauche","lambda","delta_d","delta_g","u","U_plus","U_moins"};
      mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);
      // ATTENTION Après, plus de Serial.print > car mauvais ordre pour Matlab, mais on peut en mettre avant
      // InitTelemetry allume puis éteint les LEDs > checks 

      // Reception par télémétrie
      float floatArray[8]; // même nombre qu'on envoie
      mecatro::recieveGains(8, floatArray);
      // On update les gains
      K_moins = floatArray[0];
      Kp = floatArray[1];
      Ki = floatArray[2];
      K_lambda = floatArray[3];
      Vbatt = floatArray[4];
      coef = floatArray[5];
      u_barre = floatArray[6];
      K_psi = floatArray[7];

      U_plus_barre=2*k*u_barre/p;  // cf la relation dynamique_bolide.pdf

      // Set I2C clock speed to 400kHz (fast mode). Do it after initializing everyone so that the clock speed
      // is not reset by a particular device. 
      Wire1.setClock(400000);
      // Configure motor control and feedback loop call.
      mecatro::configureArduino(CONTROL_LOOP_PERIOD);
    }
  }
}

void loop() {
  mecatro::run();
}

void mecatro::controlLoop()
{
  // SensorBar
  multiplexer.setPort(SENSORBAR_PIN);
  lambda = mySensorBar.getPosition()*0.0458/127;   // renvoie un nombre entre +127 (distance à droite) et -127 (distance à gauche) sur 9.16 cm soit 127 = 0.0458 m

  // Encodeur de droite
  multiplexer.setPort(RIGHT_ENCODER_PIN);
  RawAngle_new[0]  = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
  speed=rightEncoder.getAngularSpeed(AS5600_MODE_DEGREES);  // uniquement à des fins de débuggage et de test

  // Encodeur de gauche
  multiplexer.setPort(LEFT_ENCODER_PIN);
  RawAngle_new[1] = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;

  // Calcul de psi, en degrés
  psi = p*(RawAngle_new[0]-RawAngle_new[1])/l;  

  // Vitesses 
  OmegaD = (2.0/(CONTROL_LOOP_PERIOD*0.001)*(RawAngle_new[0]-RawAngle[0])*DEG_TO_RAD + a1 * OmegaD_prev)/b1;   // rad/s                                                                       // rad/s
  OmegaG = (2.0/(CONTROL_LOOP_PERIOD*0.001)*(RawAngle_new[1]-RawAngle[1])*DEG_TO_RAD + a1 * OmegaG_prev)/b1;

  // Appel au controleur
  Controleur(lambda, OmegaD, OmegaG, &U_plus, &U_moins, &u, psi);
  float delta_d = (U_plus + U_moins)/Vbatt*coef; // coef : facteur correctif
  float delta_g = (U_plus - U_moins)/Vbatt;    


  // Fait en sorte que le robot s'arrête au bout de 1s s'il est sorti du parcours
  multiplexer.setPort(SENSORBAR_PIN);
  int temp = mySensorBar.getDensity();

  if (temp==0 || temp>7){  // si toutes les LEDs sont allumées ou éteintes
    compteur+=CONTROL_LOOP_PERIOD; 
  } else {
    compteur = 0;   // si on retombe sur la ligne on remet à 0
  }
  
  if (compteur<1000) {   
    mecatro::setMotorDutyCycle(-delta_g,-delta_d);  // gauche = moteur gauche (-) | droite = moteur droit (-)
  } else {
    mecatro::setMotorDutyCycle(0.0,0.0);  // si toutes les LEDs sont allumées depuis plus d'1s, on s'arrête
  }

  // Envoie des données utiles
  mecatro::log(0,RawAngle[0]-CumulativePosRight_0); // soustraire la valeur initiale, pour commencer à 0
  mecatro::log(1,RawAngle[1]-CumulativePosLeft_0);
  mecatro::log(2,OmegaD);
  mecatro::log(3,OmegaG);
  mecatro::log(4,lambda); 
  mecatro::log(5,delta_d);   
  mecatro::log(6,delta_g);
  mecatro::log(7,u);
  mecatro::log(8,U_plus);
  mecatro::log(9,U_moins);
  
  // On remplace les anciennes valeurs
  for (int i = 0; i < 2; i++) {
    RawAngle[i] = RawAngle_new[i];
  }
  OmegaG_prev = OmegaG;
  OmegaD_prev = OmegaD;
}



