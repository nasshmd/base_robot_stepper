#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>
//#include <Capteur.h>

//Adresse I2C du module de navigation
#define ADRESSE 60

int timeStart;
// Etat des déplacements
#define FINI 0
#define EN_COURS 1
#define PREVU 2

// Type de ROBOT
#define ROBOT_PRIMAIRE 1
#define ROBOT_SECONDAIRE 0

// Etat de la nouvelle position demand�e
#define VALIDEE 0    // Nouvelle position valid�e et prise en compte
#define DISPONIBLE 1 // Nouvelle position enregistr�e
#define ERRONEE 2    // nouvelle position erron�e. CRC nok.

const int16_t centerOffset[2] = {0, 0};

/* Constantes pour les broches */
const byte TRIGGER_PIN = 2; // Broche TRIGGER
const byte ECHO_PIN = 3;    // Broche ECHO

/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;

//Variable Pin Moteur
const int pinStep1 = 11;
const int pinDir1 = 22;

const int pinStep2 = 12;
const int pinDir2 = 23;

const int pinSleep = 4;

const int pinM0 = 18;
const int pinM1 = 19; //cas driver non industriels
const int pinM2 = 21;

// Pin du type de robot et strategie
const int pinRobot = 17;        //principal ou secondaire
const int pinCote = 15;         // bleu ou jaune
const int pinStrategie = 16;    //strategie 1 ou 2
const int pinHomologation = 14; //homologation ou match

//pin corde
const int magneto = 5;

// Declaration des objets AccelStepper
AccelStepper MGauche(AccelStepper::DRIVER, pinStep1, pinDir1);
AccelStepper MDroit(AccelStepper::DRIVER, pinStep2, pinDir2);

byte newPos = VALIDEE;

// Declaration des variables liées à la detection d'adversaire
bool presenceArriere = 0, presenceAvant = 0;
bool presenceArriereTemp = 0, presenceAvantTemp = 0;
int adversaireArriere = 45;
int adversaireAvant = 13;
int angleBalise = A3;
int angleAvant = 0, angleArriere = 0;
int seuilAvant = 150, seuilArriere = 150;

double AskX, AskRot, TempGauche, TempDroit, NewRot, NewX;
double posi[2], posa[1]; //posi(initiale)=[x;y;theta]
double posg[2];

int sensorTime = 1000;
int avantTimeInit = 0;
int arriereTimeInit = 0;

bool optionAdversaire = false;
bool optionRecalage = false;
bool optionRalentit = false;

char etatRotation = 'a', etatAvance = 'a';
bool etatABS = false;
bool etatLastRot = false;

bool typeRobot = ROBOT_PRIMAIRE;

int16_t targetRot = 0;

// Variable par defaut pour le réglage des déplacements
float FacteurX = 1.03;     //Ancien : 154.8
float FacteurDroit = 8.0;  //Ancien : 154.8
float FacteurGauche = 8.0; //Ancien : 154.8
float FacteurRot = 3.640;  // Secondaire : 3.655 | Primaire : 3.600

float VitesseMaxDroite = 4000.0; //Ancien : 3000 11/05/2018
float VitesseMaxGauche = 4000.0; //Ancien : 3000 11/05/2018
float VitesseMinDroite = 2000.0; //Ancien : 4000 23/06/2018
float VitesseMinGauche = 2000.0; //Ancien : 4000 23/06/2018
float AccelRot = 2500.0;         //Ancien : 2000
float AccelMinDroite = 3000.0;   //Ancien : 2000
float AccelMinGauche = 3000.0;   //Ancien : 2000
float AccelMaxDroite = 5000.0;   //Ancien : 5000
float AccelMaxGauche = 5000.0;   //Ancien : 5000
float AccelStop = 4000.0;        //Ancien : 8000

// ----------Paramètres selon type de robot-----------

// --SECONDAIRE--
const float secondaireFacteurX = 1.03;       //Ancien : 154.8
const float secondaireFacteurDroit = 8.090;  //Ancien : 154.8
const float secondaireFacteurGauche = 8.064; //Ancien : 154.8
const float secondaireFacteurRot = 13.359;   // Ancien : 3.655

const float secondaireVitesseMaxDroite = 3000.0; //Ancien : 3000 11/05/2018
const float secondaireVitesseMaxGauche = 2840.0; //Ancien : 3000 11/05/2018
const float secondaireVitesseMinDroite = 1500.0; //Ancien : 4000 23/06/2018
const float secondaireVitesseMinGauche = 1450.0; //Ancien : 4000 23/06/2018

const float secondaireAccelRot = 2000.0; //Ancien : 2000

const float secondaireAccelMaxDroite = 3000.0; //Ancien : 5000
const float secondaireAccelMaxGauche = 2840.0; //Ancien : 5000
const float secondaireAccelMinDroite = 1500.0; //Ancien : 2000
const float secondaireAccelMinGauche = 1450.0; //Ancien : 2000

const float secondaireAccelStop = 6000.0; //Ancien : 8000

// --PRIMAIRE--
const float primaireFacteurX = 1.03;       //Ancien : 154.8
const float primaireFacteurDroit = 8.064;  //Ancien : 8.064
const float primaireFacteurGauche = 8.090; //Ancien : 8.128
const float primaireFacteurRot = 13.149;   // Ancien : 3.600

const float facteurPrimaireMax = 3.0;
const float facteurPrimaireAccelMax = 2.0;
const float facteurPrimaireMin = 1.0;
const float facteurPrimaireAccelMin = 1.0;
const float facteurPrimaireAccelRot = 1.0;

const float primaireVitesseMaxDroite = 900.0 * facteurPrimaireMax; //Ancien : 2963.253
const float primaireVitesseMaxGauche = 900.0 * facteurPrimaireMax; //Ancien : 3000 11/05/2018
const float primaireVitesseMinDroite = 550.0 * facteurPrimaireMin; //Ancien : 1481.627
const float primaireVitesseMinGauche = 550.0 * facteurPrimaireMin; //Ancien : 1500

const float primaireAccelRot = 2000.0 * facteurPrimaireAccelRot; //Ancien : 2000

const float primaireAccelMaxDroite = 2840.0 * facteurPrimaireAccelMax; //Ancien : 2963.253
const float primaireAccelMaxGauche = 3000.0 * facteurPrimaireAccelMax; //Ancien : 3000
const float primaireAccelMinDroite = 1450.0 * facteurPrimaireAccelMin; //Ancien : 1975.502
const float primaireAccelMinGauche = 1500.0 * facteurPrimaireAccelMin; //Ancien : 2000

const float primaireAccelStop = 6000.0; //Ancien : 8000

byte BORDURE = 0;
// AV_DROIT , AV_GAUCHE , AR_DROIT , AR_GAUCHE
int PIN_BORDURE[4] = {13, 14, 15, 16};
void debug();

void updatePos();

void turnGo();

void goTo();

void recalage();

void bordure();

void adversaire();

void changeTypeRobot(bool type);

//Fin de match
void FIN_MATCH();

void receiveEvent(int howMany);
void requestEvent();

void rouh_hna(int optionAdversaire, int optionRecalage, int optionRalentit, int NewRot, int NewX);
void match_secondaire();

void garage();
//bool detection(int capteur, int iteration);

void calcul_distangle();

void actualiser_position();