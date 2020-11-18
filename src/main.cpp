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

Adafruit_TCS34725 capteurCouleur = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

//---Declaration des differentes fonctions-----//
void forward(float speed, float distance); //Fonction pour faire avancer le robot en ligne droite sur une distance en metre
void forwardB(float speed);
void forwardHenri(float speed, float distance);
void turn(float speed, float angle); //Fonction pour faire tourner le robot selon un angle precis.
void stop(void); //Fonction pour faire arreter le Robot
unsigned int sifflet (void);//Fonction pour detecter le sifflet
void takeBall(void);
void dropBall(void);
uint8_t getColor(void);
uint8_t detectionLigne(void);
bool detecteObstacle(void);

void triangle(void);
void carre(void);

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

/*
ROBOT B
*/
void loop() 
{
//forward(0.4, 1);


}

/*==========================================================================
Fonction pour faire avancer le robot
Input:
  Speed (float): valeur de vitesse entre -1 et 1
  Distance (float): distance que le Robos va parcourir en metre
Details:
  Comme on connait le nombre de pulse par tour de roue ainsi que le diametre 
  des roues, il est possible de calculer que pour avancer d'un metre le robot
  nescessitera de 13367.32 pulses. On peut donc convertir la valeur en mettre en
  un nombre de pulse.
  Il est egalement important de noter que la vitesse du moteur droit est
  multiplier par une constante. Cette valuer permet d'equilibrer la vitesse
  entre les deux pour s'assurer quelle soit la meme pour une meme vitesse
============================================================================*/
void forward(float speed, float distance){ 
  delay(250); //pas touche!!!
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
  delay(250); //pas touche!!!
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
void forwardB(float speed){ 
  delay(200); //pas touche!!!
  uint32_t nombrePulse=floor((4.7)*13367.32); //Convertion distance en nombre de pulse
  //Declaration des variables
  float memErreur = 0;
  float erreurAvant = 0;
  float diff = 0;
  float valeurP = 0;
  float valeurI = 0;
  float valeurD = 0;
  //float rapport_Vitesse=0.3;

  //ENCODER_Reset(0);
  //ENCODER_Reset(1);

  delay(200); //pas touche!!!

  while((ENCODER_Read(LEFT) < nombrePulse) && (detecteObstacle() == false)) 
  { //tant que l'encodeur lit un nombre de pulse inferieur a la valeur nescessaire le robot va continuer. On a ici choisit de lire la valeur de l'encodeur gauche en assumant que la valeur de l'encodeur droit est identique
    delay(100);

   diff = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT));

    //Serial.println(diff);
  
    valeurP = (diff * kP);
    valeurD = ((erreurAvant - diff)*kD);
    erreurAvant = diff; 
    memErreur = (memErreur + diff);
    valeurI = (memErreur * kI);
    if(valeurI > capValeurI)
    {
      valeurI = capValeurI;
    }
    else if(valeurI < -capValeurI)
    {
      valeurI = -capValeurI;
    }
    else;

  /*if(detectionLigne() == 5)
  {
    MOTOR_SetSpeed(LEFT,(speed)); //Faire avancer le moteur gauche
    MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD)); //Faire avancer le moteur droit
  }
  else if (detectionLigne() == 6)
  {
    MOTOR_SetSpeed(LEFT,(speed)); //Faire avancer le moteur gauche
    MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD)); //Faire avancer le moteur droit
  }
  else
  {
    MOTOR_SetSpeed(LEFT,(speed)); //Faire avancer le moteur gauche
    MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD)); //Faire avancer le moteur droit
  }*/
  MOTOR_SetSpeed(LEFT,(speed)); //Faire avancer le moteur gauche
  MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD)); //Faire avancer le moteur droit

  if(detecteObstacle()== true)
  {
    break;
  }
  else
  {
    
  }
  
}
stop(); 
}

void forwardHenri(float speed,float distance){ 
delay(200); //pas touche!!!
uint32_t nombrePulse=floor((distance)*13367.32); //Convertion distance en nombre de pulse
//Declaration des variables
float memErreur = 0;
float erreurAvant = 0;
float diff = 0;
float valeurP = 0;
float valeurI = 0;
float valeurD = 0;
//float rapport_Vitesse=0.3;

ENCODER_Reset(0);
ENCODER_Reset(1);

delay(200); //pas touche!!!

while(ENCODER_Read(LEFT) < nombrePulse) 
{ //tant que l'encodeur lit un nombre de pulse inferieur a la valeur nescessaire le robot va continuer. On a ici choisit de lire la valeur de l'encodeur gauche en assumant que la valeur de l'encodeur droit est identique
  delay(100);

  diff = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT));

  //Serial.println(diff);
  
  valeurP = (diff * kP);
  valeurD = ((erreurAvant - diff)*kD);
  erreurAvant = diff; 
  memErreur = (memErreur + diff);
  valeurI = (memErreur * kI);
  if(valeurI > capValeurI)
  {
    valeurI = capValeurI;
  }
  else if(valeurI < -capValeurI)
  {
    valeurI = -capValeurI;
  }
  else;

  /*if(detectionLigne() == 5)
  {
    MOTOR_SetSpeed(LEFT,(speed)); //Faire avancer le moteur gauche
    MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD)); //Faire avancer le moteur droit
  }
  else if (detectionLigne() == 6)
  {
    MOTOR_SetSpeed(LEFT,(speed)); //Faire avancer le moteur gauche
    MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD)); //Faire avancer le moteur droit
  }
  else
  {
    MOTOR_SetSpeed(LEFT,(speed)); //Faire avancer le moteur gauche
    MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD)); //Faire avancer le moteur droit
  }*/
  MOTOR_SetSpeed(LEFT,(speed)); //Faire avancer le moteur gauche
  MOTOR_SetSpeed(RIGHT,(speed+valeurP+valeurI+valeurD)); //Faire avancer le moteur droit

}
stop(); 
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

  delay(250); //pas touche!!!
  int32_t nombrePulse=floor(abs(angle*21.8)); //21.99 ---> 20.8 
  //Serial.println(nombrePulse);
  
  float memErreur = 0;
  float diff = 0;
  float valeurP = 0;
  float valeurI = 0;
  float valeurD = 0;

  ENCODER_ReadReset(LEFT); //Reset du compteur de l'encodeur gauche
  ENCODER_ReadReset(RIGHT); //Reset du compteur de l'encodeur droit

  delay(250);//pas touche!!!

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
Fonction de detection de couleur (pour le capteur Adafruit_TCS34725)
Input:
  Aucun input
Output:
  -Retourne 1 si la couleur rose est detectee
  -Retourne 2 si la couleur jaune est detectee
  -Retourne 3 si la couleur bleu est detectee
  -Retourne 0 si la couleur nest pas rose, jaune ou bleu 
Details:
  Fait laquisition de la couleur 10 fois et retourne un chiffre correspondant
  a celle-ci si au moins la moite des mesures correspondent a une des trois
  possibilites
============================================================================*/
uint8_t getColor(void){
  if (capteurCouleur.begin()) {
    //Serial.println("Found sensor");
  } else {
    //Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
  uint16_t r, g, b, c, colorTemp, lux;
  //Valeurs temporaires (les bons chiffres vont etre places directement dans les conditions)
  uint16_t RoseR = 125;
  uint16_t RoseB = 114;
  uint16_t RoseG = 128;
  uint16_t BleuR = 91;
  uint16_t BleuB = 131;
  uint16_t BleuG = 140;
  uint16_t JauneR = 136;
  uint16_t JauneB = 147;
  uint16_t JauneG = 111;
  //---------------------------------------------------------------------------------------//
  uint16_t tolCoul = 5;//60
  uint8_t roseAdd = 0;
  uint8_t bleuAdd = 0;
  uint8_t jauneAdd = 0;

  for(uint8_t i = 0; i < 9; i++){
    capteurCouleur.getRawData(&r, &g, &b, &c);
    if((r - tolCoul < RoseR) && (RoseR < r + tolCoul) && (g - tolCoul < RoseB) && (RoseB < g + tolCoul) && (b - tolCoul < RoseG) && (RoseG < b + tolCoul)){
      roseAdd ++;
    }
    else if((r - tolCoul < BleuR) && (BleuR < r + tolCoul) && (g - tolCoul < BleuB) && (BleuB < g + tolCoul) && (b - tolCoul < BleuG) && (BleuG < b + tolCoul)){
      bleuAdd ++;
    }
    else if((r - tolCoul < JauneR) && (JauneR < r + tolCoul) && (g - tolCoul < JauneB) && (JauneB < g + tolCoul) && (b - tolCoul < JauneG) && (JauneG < b + tolCoul)){
      jauneAdd ++;
    }
    else;
  }
  if(roseAdd > 5){
    Serial.println("rose");
    digitalWrite(22,HIGH);//Led rouge allume
    digitalWrite(23,LOW);//Led bleu fermee
    digitalWrite(24,LOW);//Led jaune fermee
    return 1;
  }
  else if(bleuAdd > 5){
    Serial.println("bleu");
    digitalWrite(22,LOW);//Led rouge fermee
    digitalWrite(23,HIGH);//Led bleu allumee
    digitalWrite(24,LOW);//Led jaune fermee
    return 2;
  }
  else if(jauneAdd > 5){
    Serial.println("jaune");
    digitalWrite(22,LOW);//Led rouge fermee
    digitalWrite(23,LOW);//Led bleu fermee
    digitalWrite(24,HIGH);//Led jaune allumee
    return 3;
  } 
  else{
    Serial.println("No match found");
    digitalWrite(22,LOW);//Led rouge fermee
    digitalWrite(23,LOW);//Led bleu fermee
    digitalWrite(24,LOW);//Led jaune fermee
    return 0;
  }
}
/*==========================================================================
Fonction de detection de ligne(s)
Input:
  Aucun input
Output:
  -Retourne 0 si aucune ligne est detectee
  -Retourne 1 si ligne (B) est detectee
  -Retourne 2 si ligne (J) est detectee
  -Retourne 3 si ligne (R) est detectee
  -Retourne 4 si lignes (RB) sont detectees
  -Retourne 5 si lignes (RB) sont detectees
  -Retourne 6 si lignes (RJ) sont detectees
  -Retourne 7 si 3 lignes sont detectees 
Details:
  Fait laquisition de la tension sur la broche A2 et retourne un chiffre 
  correspondant a la combinaison de capteur qui detectent un ligne
============================================================================*/
uint8_t detectionLigne(void){
  uint16_t tension = analogRead(2);
  uint8_t tolTen = 40;
  Serial.println(tension);

  if(tension < 50){
    return 0;//Aucun
  }
  else if((tension - tolTen < 142) && (142 < tension + tolTen)){
    return 1; //B
  }
  else if((tension - tolTen < 284) && (284 < tension + tolTen)){
    return 2; //J
  }
  else if((tension - tolTen < 570) && (570 < tension + tolTen)){
    return 3; //R
  }
  else if((tension - tolTen < 426) && (426 < tension + tolTen)){
    return 4; //JB
  }
  else if((tension - tolTen < 711) && (711 < tension + tolTen)){
    return 5; //RB
  }
  else if((tension - tolTen < 852) && (852 < tension + tolTen)){
    return 6; //RJ
  }
  else return 7; //RJB
  
}

/*==========================================================================
Fonction pour faire arreter le robot
============================================================================*/
void stop(void){
  MOTOR_SetSpeed(LEFT,0); //On met la vitesse du moteur gauche a 0
  MOTOR_SetSpeed(RIGHT,0); //On met la vitesse du moteur droit a 0
}
void takeBall(void){
   SERVO_SetAngle(0,85);
   delay(2000);
}
void dropBall(void){
   SERVO_SetAngle(0,35);
   delay(2000);
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

  if(valeurADC1>450){
    return 1;
  }
  return 0;
}
/*==========================================================================
Fonction de détection d'obstacle avec le capteur infra-rouge.
============================================================================*/
bool detecteObstacle(void)
{
  int distance1 = ROBUS_ReadIR(3);
  int distance2 = ROBUS_ReadIR(3);
  Serial.println(distance1);
  if(distance1 >= 100 && distance2 >= 100)
  {
      if(ROBUS_ReadIR(2) >= 90){
        return true;
      }
      else {
        return false;
      } 
  }
  else return false;

  //Code du sonar (on ne l'utilise pas)
  /*int distance2 = SONAR_GetRange(SONAR_1);
  if(distance2 <= 58)
  {
    return true;
  }
  else
  {
    return false;
  }*/
}


void triangle(){
  //Commence par le coin du top
  delay(30);
  turn(0.2, -30);
  delay(30);
  forward(0.4, 1);
  delay(30);
  turn(0.2, 120);
  delay(30);
  forward(0.4, 1);
  delay(30);
  turn(0.2, 120);
  delay(30);
  forward(0.4, 1);
  delay(30);
}

void carre(){
  delay(30);
  turn(0.2, -90);
  delay(30);
  forward(0.4, 0.5);
  delay(30);
  turn(0.2, 90);
  delay(30);
  forward(0.4, 1);
  delay(30);
  turn(0.2, 90);
  delay(30);
  forward(0.4, 1);
  turn(0.2, 90);
  delay(30);
  forward(0.4, 1);
  turn(0.2, 90);
  delay(30);
  forward(0.4, 0.5);
}
