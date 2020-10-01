/*==========================================================================
PROJET ROBUS - Parcoura

Code réalisée par l'équipe T1

Date: 8/23/2020

But:
  Reussir a suivre un parcours au sol en encodant des moteurs et sans senseur.

Details sur le robot:
  -Les roues font 3" de diametre
  -Pour chaque tour de roues il y a 3200 pulses
============================================================================*/

#include <Arduino.h>
#include <LibRobus.h>

void forward(float speed, float distance); //Fonction pour faire avancer le robot en ligne droite sur une distance en metre
void turn(float angle,float speed); //Fonction pour faire tourner le robot selon un angle precis.
void stop(void); //Fonction pour faire arreter le Robot


/*==========================================================================
Fonction MAIN pour realiser le parcours
============================================================================*/

void setup() {BoardInit();} //Initialisation du board selon la libraire RobUS

void loop() {
  while (!ROBUS_IsBumper(3)); //Le robot va attendre d'avoir le bumper en arriere avant de partir le code
  
  //Étape du parcour a programmer ici

}

/*==========================================================================
Fonction pour faire avancer le robot
Input:
  Speed (float): valeur de vitesse entre -1 et 1
  Distance (float): distance que le Robos va parcourir en metre
Details:
  Comme on connait le nombre de pulse par tour de roue ainsi que le diametre 
  des roues, il est possible de calculer que pour avancer d'un metre le robot
  nescessitera de 13367.32 pulse. On peut donc convertir la valeur en mettre en
  un nombre de pulse.
  Il est egalement important de noter que la vitesse du moteur droit est
  multiplier par une constante. Cette valuer permet d'equilibrer la vitesse
  entre les deux pour s'assurer quelle soit la meme pour une meme vitesse
============================================================================*/
void forward(float speed, float distance){ 
  int nombrePulse=floor(distance*13367.32); //Convertion distance en nombre de pulse
  ENCODER_ReadReset(LEFT); //Reset du compteur de l'encodeur gauche
  ENCODER_ReadReset(RIGHT); //Reset du compteur de l'encodeur droit
  
  while(ENCODER_Read(LEFT)<nombrePulse){ //tant que l'encodeur lit un nombre de pulse inferieur a la valeur nescessaire le robot va continuer. On a ici choisit de lire la valeur de l'encodeur gauche en assumant que la valeur de l'encodeur droit est identique
    MOTOR_SetSpeed(LEFT,speed); //Faire avancer le moteur gauche
    MOTOR_SetSpeed(RIGHT,speed*1.05); //Faire avancer le moteur droit
  }
  stop(); //Pour arreter de faire tourner le Robot lorsquil arrive a la bonne distance

}


/*==========================================================================
Fonction pour faire tourner le Robot
Input:
  Speed (float): Valeur de vitesse entre -1 et 1
  Angle (float): Valeur de l'angle que le robot va tourner.
    -Si l'angle < 0, le robot tourne dans le sens Anti-Horaire
    -Si l'angle > 0, le robot tourne dans le sens Horaire
Details:
  Comme on connait le nombre de pulse par tour de roue, le diametre des roues et
  la distance entre les roues, il est possible de calculer que pour chaque pulse,
  le robot va tourner de 0.047629 degré. Il est donc possible de convertir l'angle
  que l'on veux en un nombre de pulse que les deux moteurs doivent donner.
============================================================================*/
void turn(float speed, float angle){
  int nombrePulse=floor(abs(angle/0.047629)); //Convertion de l'angle en nombre de pulse
  ENCODER_ReadReset(LEFT); //Reset du compteur de l'encodeur gauche
  ENCODER_ReadReset(RIGHT); //Reset du compteur de l'encodeur droit

  if(angle>=0){ //Pour tourner dans le sens Horaire
    while(ENCODER_Read(LEFT)<nombrePulse){ //tant que l'encodeur lit un nombre de pulse inferieur a la valeur nescessaire le robot va continuer a tourner
      MOTOR_SetSpeed(LEFT,speed); //Le moteur gauche va tourner dans le sens Anti-Horaire
      MOTOR_SetSpeed(RIGHT,-speed*1.05); //Le moteur droit va tourner dans le sens Anti-Horaire
    }
    stop(); //Pour arreter de faire tourner le Robot lorsquil arrive au bon angle

  }else{ //Pour tourner dans le sens Anti-Horaire
    while(ENCODER_Read(RIGHT)<nombrePulse){ //tant que l'encodeur lit un nombre de pulse inferieur a la valeur nescessaire le robot va continuer a tourner
      MOTOR_SetSpeed(LEFT,-speed); //Le moteur gauche a tourner dans le sens Horaire
      MOTOR_SetSpeed(RIGHT,speed*1.05); //Le moteur droit a tourner dans le sens Horaire
    }
    stop(); //Pour arreter de faire tourner le Robot lorsquil arrive au bon angle

  }
}

/*==========================================================================
Fonction pour faire arreter le robot
============================================================================*/
void stop(void){
  MOTOR_SetSpeed(LEFT,0); //On met la vitesse du moteur gauche a 0
  MOTOR_SetSpeed(RIGHT,0); //On met la vitesse du moteur droit a 0
}



/*
servo stuff
*/

void elevate(char moteur,char direction){
  

}

