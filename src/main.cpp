#include <main.h>

int test_adv = 0;

void setup()
{

  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  //------Déclaration des I/O------
  pinMode(magneto, INPUT);

  pinMode(pinM0, OUTPUT);
  pinMode(pinM1, OUTPUT);
  pinMode(pinM2, OUTPUT);

  pinMode(adversaireAvant, INPUT_PULLUP);
  //pinMode(adversaireArriere, INPUT_PULLUP);

  //pinMode(pinRobot, INPUT_PULLUP);

  //------Initilisation des I/O------
  // Passage en mode 1/4 de pas
  digitalWrite(pinM0, LOW);
  digitalWrite(pinM1, HIGH);
  digitalWrite(pinM2, LOW);
  // Pause des drivers avant le premier mouvement
  digitalWrite(pinSleep, HIGH);
  // Init du type de robot
  //typeRobot = digitalRead(pinRobot);
  //changeTypeRobot(typeRobot);

  FacteurX = primaireFacteurX;
  FacteurDroit = primaireFacteurDroit;
  FacteurGauche = primaireFacteurGauche;
  FacteurRot = primaireFacteurRot;

  VitesseMaxDroite = primaireVitesseMaxDroite;
  VitesseMaxGauche = primaireVitesseMaxGauche;
  VitesseMinDroite = primaireVitesseMinDroite;
  VitesseMinGauche = primaireVitesseMinGauche;
  AccelRot = primaireAccelRot;
  AccelMinDroite = primaireAccelMinDroite;
  AccelMinGauche = primaireAccelMinGauche;
  AccelMaxDroite = primaireAccelMaxDroite;
  AccelMaxGauche = primaireAccelMaxGauche;
  AccelStop = primaireAccelStop;

  //------Initialisation des communications------
  Serial.begin(9600);
  //Wire.begin(ADRESSE);
  //Wire.onReceive(receiveEvent);
  //Wire.onRequest(requestEvent);

  //------Initialisation Moteur------
  MGauche.setMaxSpeed(VitesseMaxGauche);
  MGauche.setAcceleration(AccelMaxGauche);

  MDroit.setMaxSpeed(VitesseMaxDroite);
  MDroit.setAcceleration(AccelMaxDroite);
  timeStart = millis();
  Serial.println("millis = ");
  Serial.println(millis());
  delay(1000);
  Serial.println(millis());
}

void loop()
{

  Serial.println("début match ");
  while (digitalRead(magneto))
  {
    Serial.println("retire l'aimant , LA BGHIT ZA3MA -.- ");
    delay(50);
  }

  if (digitalRead(pinRobot) & digitalRead(pinCote) & digitalRead(pinStrategie) & digitalRead(pinHomologation))
  {
    Serial.println("homologation robot principal cote bleu ");
  }
  else if (!digitalRead(pinRobot) & digitalRead(pinCote) & digitalRead(pinStrategie) & digitalRead(pinHomologation))
  {
    Serial.println("homologation robot secondaire cote jaune");
  }
  else if (!digitalRead(pinRobot) & !digitalRead(pinCote) & digitalRead(pinStrategie) & digitalRead(pinHomologation))
  {
    Serial.println("homologation robot secondaire cote bleu");
  }
  else if (!digitalRead(pinRobot) & digitalRead(pinCote) & digitalRead(pinStrategie) & !digitalRead(pinHomologation))
  {
    Serial.println("strategie match secondaire cote jaune");
    match_secondaire();
  }
  else
  {
    Serial.println("rien du tout");
    delay(1000);
    match_secondaire();
  }

  Serial.println("sortie loop ");
}

void match_secondaire()
{

  Serial.println("x et y sont à 0 ( t= 0 )");

  rouh_hna(1, 0, 0, 0, 10000);

  rouh_hna(1, 0, 0, 500, -6500);

  rouh_hna(1, 0, 0, 400, 7500);
}

void rouh_hna(int optionAdversaire_local, int optionRecalage_local, int optionRalentit_local, int NewRot_local, int NewX_local)
{
  Serial.println("Newx=");
  Serial.println(NewX_local);
  delay(1000);
  posi[2] += NewRot_local;
  optionAdversaire = optionAdversaire_local;
  optionRecalage = optionRecalage_local;
  optionRalentit = optionRalentit_local;
  NewRot = NewRot_local;
  NewX = NewX_local;

  etatRotation = PREVU;
  etatAvance = PREVU;
  while (etatAvance != FINI || etatRotation != FINI)
  {
    //bordure();
    adversaire();
    turnGo();

    MGauche.run();
    MDroit.run();

    /*if (millis() - timeStart > 90000)
    {
      actualiser_position();
      garage();
    }*/
  }
  Serial.println("x=");
  Serial.println(posa[0]);
  Serial.println("y=");
  Serial.println(posa[1]);
}

void turnGo()
{

  if ((presenceAvant && NewX >= 0 && etatAvance == EN_COURS) || (presenceArriere && NewX < 0 && etatAvance == EN_COURS))
  {
    TempGauche = MGauche.distanceToGo();
    TempDroit = MDroit.distanceToGo();

    MGauche.setAcceleration(AccelStop);
    MDroit.setAcceleration(AccelStop);
    actualiser_position();

    MGauche.move(0); // Stop as fast as possible: sets new target
    MDroit.move(0);  // Stop as fast as possible: sets new target

    TempGauche = TempGauche + MGauche.distanceToGo();
    TempDroit = TempDroit + MDroit.distanceToGo();

    MGauche.run();
    MDroit.run();

    // Attendre que l'adversaire soit partit
    for (unsigned long i = 0; i <= 80000; i++) //Attendre XXXX iterations avant de recommencer
    {
      adversaire();
      if ((presenceAvant && NewX >= 0) || (presenceArriere && NewX < 0))
      {
        i = 0; //RAZ de l'iteration si toujours un obstacle
      }
      //delayMicroseconds(10);
      MGauche.run();
      MDroit.run();
      /*if (millis() - timeStart > 90000)
      {
        garage();
      }*/
    }

    MGauche.setAcceleration(AccelMaxGauche);
    MDroit.setAcceleration(AccelMaxDroite);

    MGauche.move(TempGauche);
    MDroit.move(TempDroit);
  }
  else
  {
    if (etatRotation == PREVU)
    {

      MGauche.setAcceleration(AccelRot);
      MDroit.setAcceleration(AccelRot);
      if (optionRalentit)
      {
        MGauche.setMaxSpeed(VitesseMinGauche);
        MDroit.setMaxSpeed(VitesseMinDroite);
      }
      else
      {
        MGauche.setMaxSpeed(VitesseMaxGauche);
        MDroit.setMaxSpeed(VitesseMaxDroite);
      }
      MDroit.move(NewRot);
      MGauche.move(NewRot);
      etatRotation = EN_COURS;
    }
    if (MDroit.distanceToGo() == 0 && MGauche.distanceToGo() == 0 && etatRotation == EN_COURS) // rajoute ça plus tard , MDroit.distanceToGo() == 0 &&
    {

      etatRotation = FINI;
      if (etatLastRot)
        etatLastRot = false;
      etatAvance = PREVU;
    }
    if (etatAvance == PREVU && etatRotation == FINI)
    {
      if (optionRalentit)
      {
        MGauche.setMaxSpeed(VitesseMinGauche);
        MDroit.setMaxSpeed(VitesseMinDroite);
        MGauche.setAcceleration(AccelMinGauche);
        MDroit.setAcceleration(AccelMinDroite);
      }
      else
      {
        MGauche.setMaxSpeed(VitesseMaxGauche);
        MDroit.setMaxSpeed(VitesseMaxDroite);
        MGauche.setAcceleration(AccelMaxGauche);
        MDroit.setAcceleration(AccelMaxDroite);
      }
      MDroit.move(-NewX);
      MGauche.move(NewX);
      etatAvance = EN_COURS;
    }
    /*if (etatAvance == EN_COURS && optionRecalage)
    {
      MGauche.setMaxSpeed(VitesseMinGauche);
      // MGauche.setAcceleration(AccelMax);
      MDroit.setMaxSpeed(VitesseMinDroite);
      // MDroit.setAcceleration(AccelMax);
      MGauche.setAcceleration(AccelMinGauche);
      MDroit.setAcceleration(AccelMinDroite);
      // Si on est à la fin du mouvement
      recalage();
      Serial.println("problème ");
    }*/
    if (MDroit.distanceToGo() == 0 && MGauche.distanceToGo() == 0 && etatAvance == EN_COURS) //rajoute ça plus tard , MDroit.distanceToGo() == 0 &&
    {
      Serial.println("avance finie ");
      etatAvance = FINI;
      digitalWrite(pinSleep, LOW);
      actualiser_position();
      posi[0] = posa[0];
      posi[1] = posa[1];
    }
  }
}

void adversaire()
{
  presenceAvant = digitalRead(8);
  presenceArriere = digitalRead(9);
}

void recalage()
{
  // A MODIFIER !!!!!!!!
  // AV_DROIT , AV_GAUCHE , AR_DROIT , AR_GAUCHE
  if ((bitRead(BORDURE, 0) && NewX > 0) || (bitRead(BORDURE, 3) && NewX < 0))
  {
    MDroit.setAcceleration(AccelStop);
    MDroit.move(0); // Stop as fast as possible: sets new target
    MDroit.setCurrentPosition(0);
    while (MDroit.distanceToGo() != 0)
    {
      MDroit.run();
    }
    MDroit.setMaxSpeed(VitesseMaxDroite);
    MDroit.setAcceleration(AccelMaxDroite);
  }
  if ((bitRead(BORDURE, 1) && NewX > 0) || (bitRead(BORDURE, 2) && NewX < 0))
  {
    MGauche.setAcceleration(AccelStop);
    MGauche.move(0); // Stop as fast as possible: sets new target
    MGauche.setCurrentPosition(0);
    while (MGauche.distanceToGo() != 0)
    {
      MGauche.run();
    }
    MGauche.setMaxSpeed(VitesseMaxGauche);
    MGauche.setAcceleration(AccelMaxGauche);
  }
}

void garage()
{
  posi[0] = posa[0];
  posi[1] = posa[1];
  while (1)
  {
    Serial.println("je me gare");
    delay(1000);
  }
}

void actualiser_position()
{
  posa[0] = posi[0] + (NewX - MGauche.distanceToGo()) * cos(posi[2]);
  posa[1] = posi[1] + (NewX - MGauche.distanceToGo()) * sin(posi[2]);
}

void calcul_distangle()
{
  double X, Y, Theta;
  X = posg[0] - posa[0];
  Y = posg[1] - posa[1];
  NewX = sqrt(X * X + Y * Y);

  if ((X >= 0 && Y >= 0) || (X >= 0 && Y <= 0))
    NewRot = atan(Y / X);
  else if ((X <= 0 && Y >= 0) || (X <= 0 && Y <= 0))
  {
    NewRot = atan(Y / X) + 180;
  }
}
