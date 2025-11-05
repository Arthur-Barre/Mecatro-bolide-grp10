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
AS5600 rightEncoder(&Wire1);  // définit les capteurs d'angles
AS5600 leftEncoder(&Wire1);

// Header for I2C communication
#include "Wire.h"

// Define the name and password of the wifi network
#define WIFI_SSID "Group10"
#define WIFI_PASSWRD "password1234"

// Define the control loop period, in ms.
#define CONTROL_LOOP_PERIOD 3
#define TAU 100.0e-3  //en s

// Définition des Pins
SensorBar mySensorBar(0x3E);
#define SENSORBAR_PIN 4    // SDA : blue | SCL : yellow
#define RIGHT_ENCODER_PIN 7  // branchement sur le Main côté PIN 4
#define LEFT_ENCODER_PIN 0

// Les gains à recevoir de MatLab
float Kp = 1;
float Ki = 1;
float K_moins = 1 ;


// Définition des variables utiles
float PosSensorBar = 0.0;
float CumulativePosRight_0 = 0.0;
float CumulativePosLeft_0 = 0.0;

float speed=0.0;
float delta=0.0;
float compteur=0;

float PosSensorBar_new = 0.0;
float dlambda = 0.0;

float RawAngle[2]={0.0,0.0};
float RawAngle_new[2]={0.0,0.0};

float OmegaD = 0.0;
float OmegaG = 0.0;
float OmegaD_prev = 0.0;  // les vitesses précédentes
float OmegaG_prev = 0.0;

// Les Tensions utiles
float U;
float Vbatt = 11.2;
float U_plus = 0.0;
float U_moins = 0.0;

// Coeff du filtre dérivateur
float a1 =-(1 - 2*TAU / (CONTROL_LOOP_PERIOD*0.001));
float b1 = (1 + 2*TAU / (CONTROL_LOOP_PERIOD*0.001));

// Paramètres du système
float alpha_u = 1;
float beta_u = 1;
float p = 0.04;         // rayon des roues, en m
float u_barre = 0.2 ;   // vitesse d'équilibre souhaitée, en m/s

// Variables d'état du controleur
float dt = CONTROL_LOOP_PERIOD*0.001;   // pas de temps en secondes 
float eta_u = 0.0;   // intégrateur
float u = 0.0;  // la vitesse du robot


void Controleur(float dlambda, float OmegaD, float OmegaG, float *U_plus, float *U_moins, float *u) {  // on modifie via des références les U, appel via &U
  // Calcul de U_moins
  float dU_moins = -K_moins * dlambda;
  *U_moins += dU_moins;

  // Calcul de U_plus
  *u = p * (OmegaD + OmegaG) / 2.0;  // vitesse instantanée
  float du = u_barre - *u;                 // écart 
  eta_u += du * dt;                       // intégration
  float dU_plus = -Ki * eta_u - Kp * du;
  *U_plus += dU_plus;
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
    //Default dark on light
    mySensorBar.clearInvertBits();
    //Other option: light line on dark
    //mySensorBar.setInvertBits();
    
    //Don't forget to call .begin() to get the bar ready.  This configures HW.
    multiplexer.setPort(SENSORBAR_PIN);
    uint8_t returnStatus = mySensorBar.begin();

    if(!returnStatus)
    {
      Serial.println("sx1509 IC communication FAILED!");
      isInit = false;
    }

    // Initialisation de la position pour la 1ère itération
    PosSensorBar = mySensorBar.getPosition()*0.0458/127;  

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
      String variableNames[nVariables] = {"angleDroite","angleGauche","Omega Droite","Omega Gauche","SensorBar","dlambda","U","u","U_plus","U_moins"};
      mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);
      // ATTENTION Après, plus de Serial.print > car mauvais ordre pour Matlab, mais on peut en mettre avant
      // InitTelemetry allume puis éteint les LEDs > checks 

      // Reception par télémétrie
      float floatArray[3]; // même nombre qu'on envoie
      mecatro::recieveGains(3, floatArray);
      // On update les gains
      K_moins = floatArray[0];
      Kp = floatArray[1];
      Ki = floatArray[2];

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
  PosSensorBar_new = mySensorBar.getPosition()*0.0458/127;   // renvoie un nombre entre +127 (distance à droite) et -127 (distance à gauche) sur 9.16 cm soit 127 = 0.0458 m
  dlambda = PosSensorBar-PosSensorBar_new;

  // Encodeur de droite
  multiplexer.setPort(RIGHT_ENCODER_PIN);
  RawAngle_new[0]  = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
  speed=rightEncoder.getAngularSpeed(AS5600_MODE_DEGREES); 

  // Encodeur de gauche
  multiplexer.setPort(LEFT_ENCODER_PIN);
  RawAngle_new[1] =leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
 
  // Vitesses 
  OmegaD = (2.0/(CONTROL_LOOP_PERIOD*0.001)*(RawAngle_new[0]-RawAngle[0]) + a1 * OmegaD_prev)/b1;
  OmegaG = (2.0/(CONTROL_LOOP_PERIOD*0.001)*(RawAngle_new[1]-RawAngle[1]) + a1 * OmegaG_prev)/b1;

  
  compteur+=CONTROL_LOOP_PERIOD;
  /*if (compteur<100){
    delta = 0;
  } else if (compteur>100 && compteur<4000){
    if (((int)compteur % 500) ==0){
      delta+=0.04;
    }
  } else if (compteur >= 4000 && compteur < 8000) {
    if (((int)compteur % 500) ==0){
      delta-=0.03;
    }  
  } else if (compteur<12000){
    if (((int)compteur % 500) ==0){
      delta+=0.02;
    }  
  } else if (compteur>12000){
    delta=0;
  }
  
  if (500<compteur && compteur<1000) {    //  6 x 2 mesures en créneau à 0.5, 0.4, 0.3
    delta = 0.2;
  } else if (1500<=compteur && compteur < 2000) {
    delta = 0.5;
  } else {
    delta=0.0;
  }*/
  //delta=PosSensorBar/0.0458;

  // Appel au controleur
  Controleur(dlambda, OmegaD, OmegaG, &U_plus, &U_moins, &u);
  float delta_d = (U_plus + U_moins)/Vbatt;
  float delta_g = (U_plus - U_moins)/Vbatt;
  U=delta*Vbatt;              // Vbatt = tension batterie, la tension fournie au moteur est celle de la batterie modulée par le PWM du shield  

  // Keep the motor off, i.e. at 0 duty cycle (1 is full forward (i.e. trigonometric sense in the motor frame), -1 full reverse)
  mecatro::setMotorDutyCycle(delta_g,-delta_d);  // gauche = moteur gauche (+) | droite = moteur droit (-)

  // Envoie des données utiles
  mecatro::log(0,RawAngle[0]-CumulativePosRight_0); // soustraire la valeur initiale, pour commencer à 0
  mecatro::log(1,RawAngle[1]-CumulativePosLeft_0);
  mecatro::log(2,OmegaD);
  mecatro::log(3,OmegaG);
  mecatro::log(4,PosSensorBar_new); 
  mecatro::log(5,dlambda);
  mecatro::log(6,U);
  mecatro::log(7,u);
  mecatro::log(8,U_plus);
  mecatro::log(9,U_moins);
  
  // On remplace les anciennes valeurs
  for (int i = 0; i < 2; i++) {
    RawAngle[i] = RawAngle_new[i];
  }
  OmegaG_prev = OmegaG;
  OmegaD_prev = OmegaD;
  PosSensorBar = PosSensorBar_new;
}



