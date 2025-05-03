#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <hd44780.h>
#include <Adafruit_MCP23X17.h>

#include <iostream>

//#include <LCD.h>
//#include <math.h>

// led
#define LED1 27  // LED labyrinth bas gauche
#define LED2 14  // LED labyrinth
#define LED3 32  // LED labyrinth
#define LED4 12  // LED labyrinth
//#define LEDCouleurDroite 30 //LED bouton couleur droit
//#define LEDCouleurMilieu 30 //LED bouton couleur milieu
//#define LEDCouleurGauche 30 //LED bouton couleur gauche
#define ledLaby 13  //LED labyrinth complete
#define ledBoutons 13 //LED 2 boutons appuyé complete
#define LEDTiroirMag 5 //LED tiroir magique complete
#define LEDPoignee 6 //LED poignée complete
#define LEDTiroir2 7 //LED tiroir complete
#define LEDScrew 11 //LED visser complete
#define LEDUnscrew 15 //LED devisser complete
//#define LEDLevier 14 //LED levier complete
#define LEDEncodeur1 7 //LED encodeur complete
#define LEDEncodeur2 3 //LED encodeur complete
#define LEDEncodeur3 2 //LED encodeur complete
#define LEDUSB 8 //LED usb complete
#define LEDXLR 4 //LED XLR complete
#define LEDEthernet 9 //LED Ethernet complete

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_MCP23X17 mcp1; //0x20 -> #1
Adafruit_MCP23X17 mcp2; //0x21 -> #2

//#define RAND_MAX 4

// Encoder 1
#define CLK1 4
#define DT1 5  
#define SW1 6

// Encoder 2
#define CLK2 0
#define DT2 1
#define SW2 2

// Encoder 3
#define CLK3 18  // 34
#define DT3 19
#define SW3 4  // 32

// limitSwitch
#define switch1 34  // limitSwitch labyrinth
#define switch2 35  // limitSwitch labyrinth
#define switch3 25  // limitSwitch labyrinth
#define switch4 26  // limitSwitch labyrinth
#define switch5 11  //tiroir magique
#define switch6 13  //poignée
#define switch7 12  //tiroir 2
#define switch8 9  //clé usb
//#define switch9 31  //XLR
#define switch10 8  //Ethernet
#define switch11 0  //Bouton couleur droite
#define switch12 1  //Bouton couleur milieu
#define switch13 2  //Bouton couleur gauche
#define switch14 3  //Emergency stop
#define switch15 10 //Visser
#define switch16 12  //Dévisser
//#define switch18 31  //Levier
#define boutons 14 //Double bouton

// nb point
#define ptsEncoder 50

uint8_t counter1 = 0;
uint8_t counter2 = 0;
uint8_t counter3 = 0;
uint8_t currentCLK1;
uint8_t currentCLK2;
uint8_t currentCLK3;
uint8_t lastCLK1;
uint8_t lastCLK2;
uint8_t lastCLK3;
uint8_t number_goal;
uint8_t LED_goal;
uint8_t posColorButton;
uint8_t limitState = 0;
int del = 34;
uint8_t ledlaby = 0;
bool oldState15;
bool newStateTiroir2 = true;
bool newStateTiroirMag = true;
bool newStatePoignee = true;
bool newStateUSB = true;
bool newStateXLR = true;
bool newStateEthernet = true;
bool newStateEStop = true;
bool newStateScrew = true;
bool newStateUnscrew = true;
bool newStateBouton = true;
// hw_timer_t * timer = NULL;
int seconde2 = 0;
int seconde1 = 0;
int minute1 = 0;
int minute2 = 0;
uint8_t totalPoints = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

void addPoints(int Points)
{
    totalPoints = totalPoints + Points;

}
void addPointsTask(void *pvParameters){
  int points = (int)(intptr_t)pvParameters;
  addPoints(points);
}
void createAddPointsTask(int points){

  xTaskCreate(addPointsTask, "addPoints", 4096,(void*)(intptr_t)points, 2,NULL);
}

void affichageEcran(void *){
    while(1){
      //updatescreen
      lcd.setCursor(0, 1);
      lcd.print("Pointage:");
      lcd.print(totalPoints);
      delay(10);
    }
}

void right_position(int id, int currentCLK, int lastCLK)
{
    if (currentCLK != lastCLK)
    {
        Serial.print("Appuyé sur bouton ");
        Serial.print(id);
        createAddPointsTask(2 * ptsEncoder);
    }
}

void showLEDLaby(void *)
{
    while(1){
    if (LED_goal == 1)
    {
        digitalWrite(LED1, HIGH);
    }
    else if (LED_goal == 2)
    {
        digitalWrite(LED2, HIGH);
    }
    else if (LED_goal == 3)
    {
        digitalWrite(LED3, HIGH);
    }
    else
    {
        digitalWrite(LED4, HIGH);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Partie labyrinthe
void labyrinthe(void *)
{ 
    while(1){
    lcd.print("labyrinthe");
  
    while (ledlaby < 4)
    {  // allumer et eteindre 4 DEL pour avoir pts
      int etatswitch1 = digitalRead(switch1);
      int etatswitch2 = digitalRead(switch2);
      int etatswitch3 = digitalRead(switch3);
      int etatswitch4 = digitalRead(switch4);

        if (LED_goal == 1 && etatswitch1 == LOW)
        {
            digitalWrite(LED1, LOW);  // eteindre LED
            ledlaby++;
            LED_goal = random(2, 4);
        
        }
        else if (LED_goal == 2 && etatswitch2 == LOW)
        {                             
            digitalWrite(LED2, LOW);  // eteindre LED
            ledlaby++;
            LED_goal = random(1, 4);
            
        }
        else if (LED_goal == 3 && etatswitch3 == LOW)
        {                             
            digitalWrite(LED3, LOW);  // eteindre LED
            ledlaby++;
            LED_goal = random(1, 4);
            
        }
        else if (LED_goal == 4 && etatswitch4 == LOW)
        {                           
            digitalWrite(LED4, LOW);  // eteindre LED
            ledlaby++;
            LED_goal = random(1, 3);
            
        }
    }

    addPoints(350);
    //createAddPointsTask(350);
    digitalWrite(ledLaby, HIGH);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void tiroirMagique(void *){
  while(1){
  if (mcp2.digitalRead(switch5) == HIGH && newStateTiroirMag == true){
    mcp2.digitalWrite(LEDTiroirMag,HIGH);
    lcd.setCursor(0,0);
    lcd.print("Tiroir ouvert!");
    addPoints(100);
    //createAddPointsTask(100);
    newStateTiroirMag = false;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}
}
//fonctionne
void poignee(void *){
  while(1){
  if (mcp2.digitalRead(switch6) == LOW && newStatePoignee == true){
    mcp2.digitalWrite(LEDPoignee,HIGH);
    lcd.setCursor(0,0);
    lcd.print("Poignee unlocked!");
    //createAddPointsTask(150);
    addPoints(150);
    newStatePoignee = false;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}
}
//fonctionne
void tiroir2(void *){
  while(1){
    if (mcp2.digitalRead(switch7) == HIGH && newStateTiroir2 == true){
      mcp2.digitalWrite(LEDTiroir2,HIGH);
      lcd.setCursor(0,0);
      lcd.print("Tiroir ouvert!");
      //createAddPointsTask(100);
      addPoints(100);
      newStateTiroir2 = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void usb(void *){
  while(1){
    if (mcp2.digitalRead(switch8) == LOW && newStateUSB == true){
      mcp2.digitalWrite(LEDUSB,HIGH);
      //createAddPointsTask(100);
      addPoints(100);
      newStateUSB = false;
    }
  vTaskDelay(pdMS_TO_TICKS(50));
  }
}
/*
void xlr(void *){
  if (digitalRead(switch9) == HIGH && newStateXLR ==true){
    digitalWrite(LEDXLR,HIGH);
    createAddPointsTask(150);
    newStateXLR = false;
  }
}
*/
void ethernet(void *){
  while(1){
  if (mcp1.digitalRead(switch10) == HIGH && newStateEthernet ==true){
    mcp1.digitalWrite(LEDEthernet,HIGH);
    //createAddPointsTask(150);
    addPoints(150);
    newStateEthernet = false;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}
}

/*
void initColorButtons(void *){ //allumer le bouton de couleur à éteindre
  if (posColorButton ==1){
    digitalWrite(LEDCouleurDroite,HIGH);
  }else if (posColorButton ==2){
    digitalWrite(LEDCouleurMilieu,HIGH);
  }else{
    digitalWrite(LEDCouleurGauche,HIGH);
  }
}

void closeColorButtons(void *){ //eteindre le bouton de couleur allume

  if (posColorButton ==1 && digitalRead(switch11) == HIGH){
    digitalWrite(LEDCouleurDroite, LOW);
    affichagePoints(50);
  }
  if (posColorButton ==2 && digitalRead(switch12)==HIGH){
    digitalWrite(LEDCouleurMilieu, LOW);
    affichagePoints(50);
  }
  if (posColorButton ==3 && digitalRead(switch13) ==HIGH){
    digitalWrite(LEDCouleurGauche,LOW);
    affichagePoints(50);
  }

}
*/
void eStop(void *){
  while(1){
    if (mcp2.digitalRead(switch14) == LOW && newStateEStop == true){
      //createAddPointsTask(150);
      addPoints(150);
      lcd.setCursor(0,0);
      lcd.print("FIN DU PANNEAU!");
      newStateEthernet = false;
    }
  vTaskDelay(pdMS_TO_TICKS(50));
  }
}
//fonctionne
void screw(void *){
  while(1){
    if (mcp1.digitalRead(switch15) == HIGH && newStateScrew == true){
      mcp1.digitalWrite(LEDScrew,HIGH);
      lcd.setCursor(0,0);
      lcd.print("Termine screw!");
      addPoints(150);
      //createAddPointsTask(150);
      newStateScrew = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
//fonctionne
void unscrew(void *){
  while(1){
    if (mcp1.digitalRead(switch16) == LOW && newStateUnscrew == true){
      lcd.print("switch");
      mcp1.digitalWrite(LEDUnscrew, HIGH);
      lcd.setCursor(0,0);
      lcd.print("Termine unscrew!");
      addPoints(150);
      //createAddPointsTask(50);
      newStateUnscrew = false;
    }
  vTaskDelay(pdMS_TO_TICKS(50));
  }
}

//Partie appuyé sur les deux boutons en même temps
void twoButtonPressed(void *){
  while(1){
    //lcd.print(mcp1.digitalRead(boutons)) ; 
    if (mcp1.digitalRead(boutons) == LOW && newStateBouton == true){
        mcp1.digitalWrite(ledBoutons, HIGH);
        lcd.setCursor(0,0);
        lcd.print("Termine tache!");
        //createAddPointsTask(50);
        addPoints(50);
        newStateBouton = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void encodeur1(void *){
    
  while(1){
  // si encoder 1 est tourné
    currentCLK1 = digitalRead(CLK1);
if (currentCLK1 != lastCLK1 && currentCLK1 == 1)
{
    // si DT différent de CLK, va sens anti-horaire, soustraire CLK au compteur
    if (digitalRead(DT1) != currentCLK1)
    {
        counter1--;
    }
    else
    {
        counter1++;  // si DT pareil à CLK, va dans sens horaire, ajouter CLK au compteur
    }
    lcd.setCursor(0, 3);
    lcd.print("Compteur : ");
    lcd.print(counter1);
}

// si encoder 1 est appuyé, donc il est au GND (LOW)
int buttonState = digitalRead(SW1);
if (buttonState == LOW)
{
    lcd.print("Bouton 1 appuyé");
    delay(200);
    createAddPointsTask(ptsEncoder);
}
if (counter1 == number_goal)
    {
        right_position(1, currentCLK1, lastCLK1);
    }
    lastCLK1 = currentCLK1;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void encodeur2(void *){
  while(1){

  // si encoder 2 est tourné
currentCLK2 = digitalRead(CLK2);
if (currentCLK2 != lastCLK2 && currentCLK2 == 1)
{
    // si DT différent de CLK, va sens anti-horaire, soustraire CLK au compteur
    if (digitalRead(DT2) != currentCLK2)
    {
        counter2--;
    }
    else
    {
        counter2++;  // si DT pareil à CLK, va dans sens horaire, ajouter CLK au compteur
    }
    lcd.setCursor(0, 3);
    lcd.print("Compteur 2 : ");
    lcd.print(counter2);
}

// si encoder est appuyé, donc il est au GND (LOW)
int buttonState2 = digitalRead(SW2);
if (buttonState2 == LOW)
{
    lcd.print("Bouton 2 appuyé");
    delay(200);
    createAddPointsTask(ptsEncoder);
}
if (counter2 == number_goal)
    {
        right_position(2, currentCLK2, lastCLK2);
    }
         lastCLK2 = currentCLK2;
  
         vTaskDelay(pdMS_TO_TICKS(50));
        }

}
void encodeur3(void *){
  while(1){

  // si encoder 3 est tourné
  currentCLK3 = digitalRead(CLK3);
  if (currentCLK3 != lastCLK3 && currentCLK3 == 1)
  {
      // si DT différent de CLK, va sens anti-horaire, soustraire CLK au compteur
      if (digitalRead(DT3) != currentCLK3)
      {
          counter3--;
      }
      else
      {
          counter3++;  // si DT pareil à CLK, va dans sens horaire, ajouter CLK au compteur
      }
      lcd.setCursor(0, 3);
      lcd.print("Compteur 3 : ");
      lcd.print(counter3);
  }

  // si encoder est appuyé, donc il est au GND (LOW)
  int buttonState3 = digitalRead(SW3);
  if (buttonState3 == LOW)
  {
      lcd.print("Bouton 3 appuye");
      delay(200);
      createAddPointsTask(ptsEncoder);
  }

  if (counter3 == number_goal)
  {
      right_position(3, currentCLK3, lastCLK3);
  }

  lastCLK3 = currentCLK3;
  vTaskDelay(pdMS_TO_TICKS(50));
}
}

void setup()
{
    
    Serial.begin(9600);
/*
    unsigned long timer1 = millis();
    unsigned long timer2 = millis();
    while (true)
    {
        if (millis() > timer1 + 1000UL)
        {
            digitalWrite(12, !digitalRead(12));
            timer1 = millis();
        }

        if (millis() > timer2 + 500UL)
        {
            digitalWrite(14, !digitalRead(14));
        }
    }*/

    Wire.begin(SDA_PIN,SCL_PIN);
    Wire.begin();
    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.begin(20, 4);
    Serial.begin(1000);
    lcd.clear();
    //Initialize MCP23017 expanders
   if (!mcp1.begin_I2C(0x20)) {//20
      lcd.print("MCP23017 #1 not found!");
      while (1);
  }
    if (!mcp2.begin_I2C(0x21)) {//21
      lcd.print("MCP23017 #2 not found!");
      while (1);
  }

    // setup LED
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    // pinMode(led5, OUTPUT);
    // pinMode(led6, OUTPUT);
    // pinMode(led7, OUTPUT);
    pinMode(ledLaby, OUTPUT);
    mcp1.pinMode(ledBoutons, OUTPUT);
    mcp2.pinMode(LEDTiroirMag, OUTPUT);
    mcp2.pinMode(LEDPoignee, OUTPUT);
    mcp2.pinMode(LEDTiroir2, OUTPUT);
    mcp1.pinMode(LEDScrew, OUTPUT);
    mcp1.pinMode(LEDUnscrew, OUTPUT);
    // pinMode(LEDLevier, OUTPUT);
    mcp1.pinMode(LEDEncodeur1, OUTPUT);
    mcp1.pinMode(LEDEncodeur2, OUTPUT);
    pinMode(LEDEncodeur3, OUTPUT);
    mcp2.pinMode(LEDUSB, OUTPUT);
    mcp2.pinMode(LEDXLR, OUTPUT);
    mcp1.pinMode(LEDEthernet, OUTPUT);

    // setup switch
    pinMode(switch1, INPUT_PULLUP);
    pinMode(switch2, INPUT_PULLUP);
    pinMode(switch3, INPUT_PULLUP);
    pinMode(switch4, INPUT_PULLUP);
    mcp2.pinMode(switch5, INPUT_PULLUP);
    mcp2.pinMode(switch6, INPUT_PULLUP);
    mcp2.pinMode(switch7, INPUT_PULLUP);
    mcp2.pinMode(switch8, INPUT_PULLUP);
    // pinMode(switch9, INPUT_PULLUP);
    mcp1.pinMode(switch10, INPUT_PULLUP);
    mcp2.pinMode(switch11, INPUT_PULLUP);
    mcp2.pinMode(switch12, INPUT_PULLUP);
    mcp2.pinMode(switch13, INPUT_PULLUP);
    mcp2.pinMode(switch14, INPUT_PULLUP);
    mcp1.pinMode(switch15, INPUT_PULLUP);
    mcp1.pinMode(switch16, INPUT_PULLUP);
    // pinMode(switch17, INPUT_PULLUP);
    // pinMode(switch18, INPUT_PULLUP);
    mcp1.pinMode(boutons, INPUT_PULLUP);
    
      //Encodeur
        mcp1.pinMode(CLK1,INPUT);
        mcp1.pinMode(DT1,INPUT);
        mcp1.pinMode(SW1,INPUT_PULLUP);
        mcp1.pinMode(CLK2,INPUT);
        mcp1.pinMode(DT2,INPUT);
        mcp1.pinMode(SW2,INPUT_PULLUP);
        pinMode(CLK3,INPUT);
        pinMode(DT3,INPUT);
        pinMode(SW3,INPUT_PULLUP);
    
    //delay(1000);
    lastCLK1 = digitalRead(CLK1);
    lastCLK2 = digitalRead(CLK2);
    lastCLK3 = digitalRead(CLK3);
    number_goal = random(-20, 20);

    // Ecran
    lcd.setCursor(0, 2);
    lcd.print("Mettre encodeurs:");
    lcd.print(number_goal);
    
    // Aléatoire Bouton Couleur
    //posColorButton = random(1, 3);
    // initColorButtons();

      // Labyrinthe
      LED_goal = random(1, 4);
      mcp1.digitalWrite(LEDScrew, LOW);
      mcp1.digitalWrite(LEDUnscrew, LOW);
      mcp1.digitalWrite(ledBoutons, LOW);
      mcp2.digitalWrite(LEDPoignee,LOW);
      mcp2.digitalWrite(LEDTiroir2,LOW);
      mcp2.digitalWrite(LEDTiroirMag,LOW);
      mcp2.digitalWrite(LEDUSB,LOW);

      xTaskCreate(affichageEcran,"affichageEcran",4096, NULL,2,NULL);
      xTaskCreate(screw,"screw", 4096,NULL,2,NULL);
      xTaskCreate(unscrew,"unscrew", 4096,NULL,2,NULL);
      xTaskCreate(showLEDLaby, "OpenLEDLaby", 4096,NULL,2,NULL);
      xTaskCreate(labyrinthe,"labyrinthe",4096,NULL,2,NULL);
      xTaskCreate(twoButtonPressed,"twoButtonPressed", 4096,NULL,2,NULL);
      xTaskCreate(eStop,"eStop", 4096,NULL,2,NULL);
      xTaskCreate(ethernet,"ethernet", 4096,NULL,2,NULL);
      //xTaskCreate(xlr,"xlr", 4096,NULL,2,NULL);
      xTaskCreate(usb,"usb", 4096,NULL,2,NULL);
      xTaskCreate(tiroirMagique,"tiroirMagique", 4096,NULL,2,NULL);
      xTaskCreate(tiroir2,"tiroir2", 4096,NULL,2,NULL);
      xTaskCreate(poignee,"poignee", 4096,NULL,2,NULL);
      //xTaskCreate(initLaby, "initLaby", 4096,NULL,2,NULL);
      //xTaskCreate(addPointsTask,"addPoints", 4096, NULL,2,NULL);

}

void loop(){
    // timer();
    /*if (LED_goal ==1 && digitalRead(switch1)==LOW){
        digitalWrite(LED1,LOW); //eteindre LED
        Serial.print("allume!");
        //ledlaby++;
        LED_goal = random(2,4);
        //initLaby(); //allumer nouvelle LED
      }
    //timer;
    /*
    for(minute2;minute2<=6;minute2){
      lcd.setCursor(15,0);
      lcd.print(minute2);
      for(minute1; minute1<10;minute1){
          lcd.setCursor(16,0);
          lcd.print(minute1);
          for(seconde2; seconde2<6; seconde2){
            lcd.setCursor(18,0);
            lcd.print(seconde2);
            for(seconde1; seconde1<10; seconde1++){
              lcd.setCursor(17,0);
              lcd.print(":");
              lcd.setCursor(19,0);
              lcd.print(seconde1);
              delay(1000);
            }
            seconde1 =0;
            seconde2 ++;
          }
        seconde2 = 0;
        minute1++;
      }
      minute1= 0;
      minute2++;
    }
    */
}
