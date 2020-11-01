/*==========================================================================
PROJET ROBUS - Parcoura

Code realisee par l'equipe P1

Date: 8/23/2020

But:
  Reussir a suivre un parcours au sol en encodant des moteurs et sans senseur.

Details sur le robot:
  -Les roues font 3" de diametre
  -Pour chaque tour de roues il y a 3200 pulses
============================================================================*/

#include <Arduino.h>
#include <LibRobus.h>
//test
//---Declaration des differentes fonctions-----//
void forward(float speed, float distance); //Fonction pour faire avancer le robot en ligne droite sur une distance en metre
void turn(float speed, float angle); //Fonction pour faire tourner le robot selon un angle precis.
void stop(void); //Fonction pour faire arreter le Robot
unsigned int sifflet (void);//Fonction pour detecter le sifflet

//--Enlever les commentaires des variables selon le robot a programmer--//
//valeurs PID robot A:
	/*float kP = 0.002;
	float kI = 0.0003;
	float kD = 0.000076;*/

//Valeurs PID robot B:
	float kP = 0.0013;
	float kI = 0.00043;
	float kD = 0.000070;
//----------------------------------------------------------------------//

float kP_Turn=0.0028;
float kI_Turn=0; //I nest pas utilise pour les virages
float kD_Turn=0; //D nest pas utilise pour les virages

float SPEEDTURN = 0.2; //Vitesse des virages [0,1]

//-------(somme de SPEEDFOWARD et SPEEDrun ne peut pas depasser 0.9)--------//
float SPEEDFORWARD = 0.6; //Vitesse des lignes droites [0,1]
float SPEEDrun = 0.0; //Vitesse ajoutee aux lignes droites pour le retour [0,1]
//--------------------------------------------------------------------------//

float capValeurI = 0.04;
float correctAngle = 2.;

/*==========================================================================
Fonction MAIN pour realiser le parcours
============================================================================*/

void setup() {BoardInit();} //Initialisation du board selon la libraire RobUS

void loop() {
  while (sifflet()==1); //Le robot va attendre que le bumper en arriere soit active avant de partir le code
  turn(0.4,360);

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
  delay(200); //pas touche!!!
  uint32_t nombrePulse=floor(distance*13367.32); //Convertion distance en nombre de pulse
  //Declaration des variables
  float memErreur = 0;
  float erreurAvant = 0;
  float diff = 0;
  float valeurP = 0;
  float valeurI = 0;
  float valeurD = 0;
  float rapport_Vitesse=0.3;

  ENCODER_ReadReset(LEFT); //Reset du compteur de l'encodeur gauche
  ENCODER_ReadReset(RIGHT); //Reset du compteur de l'encodeur droit
  delay(200); //pas touche!!!

  while(ENCODER_Read(LEFT)<nombrePulse){ //tant que l'encodeur lit un nombre de pulse inferieur a la valeur nescessaire le robot va continuer. On a ici choisit de lire la valeur de l'encodeur gauche en assumant que la valeur de l'encodeur droit est identique
    
    delay(100);

    diff = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT));

    //Serial.println(diff);

    valeurP = (diff * kP);
    valeurD = ((erreurAvant - diff)*kD);
    erreurAvant = diff; 
    memErreur = (memErreur + diff);
    valeurI = (memErreur * kI);
    
    if(valeurI > capValeurI){
      valeurI = capValeurI;
    }
    else if(valeurI < -capValeurI){
      valeurI = -capValeurI;
    }
    else;

    MOTOR_SetSpeed(LEFT,(speed*rapport_Vitesse)); //Faire avancer le moteur gauche
    MOTOR_SetSpeed(RIGHT,(rapport_Vitesse*(speed+valeurP+valeurI+valeurD))); //Faire avancer le moteur droit
    
    if(rapport_Vitesse < 1 && ENCODER_Read(LEFT) < float(nombrePulse*0.50)){
        rapport_Vitesse+=0.1;
    } 
    if(ENCODER_Read(LEFT) > (int32_t(nombrePulse)-5347)&& rapport_Vitesse > 0.23){
      rapport_Vitesse-=0.1;
    }
  }
  stop(); //Pour arreter de faire avancer le Robot lorsquil arrive a la bonne distance
  //Serial.println("FIN DU TEST");
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
  le robot va tourner de 0.047629 degre. Il est donc possible de convertir l'angle
  que l'on veux en un nombre de pulse que les deux moteurs doivent donner.
============================================================================*/
void turn(float speed, float angle){

  if(angle<0){ angle+=correctAngle;}
  else if(angle>0){angle-=correctAngle;}

  delay(200); //pas touche!!!
  int32_t nombrePulse=floor(abs(angle*21.8)); //21.99 ---> 20.8 
  //Serial.println(nombrePulse);
  
  float memErreur = 0;
  float diff = 0;
  float valeurP = 0;
  float valeurI = 0;
  float valeurD = 0;

  ENCODER_ReadReset(LEFT); //Reset du compteur de l'encodeur gauche
  ENCODER_ReadReset(RIGHT); //Reset du compteur de l'encodeur droit

  delay(200);//pas touche!!!

  if(angle>=0){ //Pour tourner dans le sens Horaire
    while(ENCODER_Read(LEFT)<nombrePulse){ 

      for(int i=0;i<10;i++){
        delay(5);
        if(!(ENCODER_Read(LEFT)<nombrePulse)){
          break;
        }
      }

      if(!(ENCODER_Read(LEFT)<nombrePulse)){
          break;
      }

      diff = (abs(ENCODER_Read(LEFT)) - abs(ENCODER_Read(RIGHT)));
      Serial.println(diff);

      valeurP = (diff * kP_Turn);
      valeurD = ((memErreur - diff)*kD_Turn);
      memErreur = (memErreur + diff);
      valeurI = (memErreur * kI_Turn);
      
      MOTOR_SetSpeed(LEFT,speed); 
      MOTOR_SetSpeed(RIGHT,-(speed+valeurP+valeurI+valeurD)); 
    }
    stop();  
  }
  else{ 
    while(abs(ENCODER_Read(LEFT))<nombrePulse){ 
      for(int i=0;i<10;i++){
        delay(5);
        if(!(abs(ENCODER_Read(LEFT))<nombrePulse)){
          break;
        }
      }

      if(!(abs(ENCODER_Read(LEFT))<nombrePulse)){
          break;
      }
      diff = (abs(ENCODER_Read(LEFT)) - abs(ENCODER_Read(RIGHT)));

      valeurP = (diff * kP_Turn);
      valeurD = ((memErreur - diff)*kI_Turn);
      memErreur = (memErreur + diff);
      valeurI = (memErreur * kD_Turn);
      
      MOTOR_SetSpeed(LEFT,-(speed)); 
      MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD));
      }
    stop();
  }
}

/*==========================================================================
Fonction pour faire arreter le robot
============================================================================*/
void stop(void){
  MOTOR_SetSpeed(LEFT,0); //On met la vitesse du moteur gauche a 0
  MOTOR_SetSpeed(RIGHT,0); //On met la vitesse du moteur droit a 0
}
/*==========================================================================
Fonction pour détecter le coup de sifflet, à mettre condition ==0 ou ==1 dans 
le while du main, nbvolt est la diff de tension entre bruit ambiant et 5kz, 
pin ADC 0 et 1
============================================================================*/
unsigned int sifflet (void){
  float nbvolt=0.2;
  int valeurADC0 = analogRead(0);
  int valeurADC1 = analogRead(1);
  if(valeurADC0<valeurADC1+(floor((nbvolt/5)*1024))){
    return 1;
  }
  return 0;
}
