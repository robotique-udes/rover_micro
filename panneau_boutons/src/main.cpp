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
#define LED4 14  // LED labyrinth b7
#define ledLaby 13  //LED labyrinth complete
#define ledBoutons 13 //LED 2 boutons appuyé complete
#define LEDTiroirMag 5 //LED tiroir magique complete
#define LEDPoignee 6 //LED poignée complete
#define LEDTiroir2 7 //LED tiroir complete
#define LEDScrew 11 //LED visser complete
#define LEDUnscrew 15 //LED devisser complete
#define LEDLevier 12 //LED levier complete
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
#define switch9 27  //XLR ou 27
#define switch10 8  //Ethernet
#define switch11 0  //Bouton couleur droite
#define switch12 1  //Bouton couleur milieu
#define switch13 2  //Bouton couleur gauche
#define switch14 3  //Emergency stop
#define switch15 10 //Visser
#define switch16 12  //Dévisser
#define switchlevier 10  //Levier
#define boutons 14 //Double bouton

#define capteurIR 15 //Capteur infrarouge

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
uint8_t limitState = 0;
int del = 34;
uint8_t ledlaby = 0;
bool oldState15;
bool newStateTiroir2 = true;
bool newStateTiroirMag = true;
bool newStatePoignee = true;
bool doneUSB = false;
bool newStateXLR = true;
bool doneEthernet = false;
bool lockEStop = false;
bool newStateScrew = true;
bool newStateUnscrew = true;
bool newStateBouton = true;
bool doneCapteur = false;
bool doneLevier = false;
bool doneEStop = false;
bool doneEncodeur1 = false;
bool encod1appui = false;
bool doneEncodeur2 = false;
bool encod2appui = false;
bool doneEncodeur3 = false;
bool encod3appui = false;
bool doneBoutonGauche = false;
bool doneBoutonMilieu = false;
bool doneBoutonDroit = false;
bool doneCouleur = false;
bool donelaby = false;
uint16_t totalPoints = 0;
char TABLABY[4] = {LED1, LED2, LED3, LED4};
QueueHandle_t LCDQueue;
typedef struct {
    char texte[20];
    uint8_t ligne;
}LCDmessage;

LiquidCrystal_I2C lcd(0x27, 20, 4);

void addPoints(int Points)
{
    totalPoints = totalPoints + Points;

}

void affichageEcran(void *){
  LCDmessage message;  
  while(1){
      //updatescreen
      if(xQueueReceive(LCDQueue, &message, 0) == pdTRUE){
          lcd.setCursor(0,message.ligne);
          lcd.print("                    "); //effacer la ligne
          lcd.setCursor(0,message.ligne);
          lcd.print(message.texte); //ecrire message
          //lcd.setCursor(0,1)

      }
      
      lcd.setCursor(0, 1);
      lcd.print("Pointage:");
      lcd.print(totalPoints);
      //delay(10);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void labyrinthe(void *) {
  while (1) {
      if (ledlaby >= 4 && donelaby == false) {
          addPoints(20);  // Bonus pour avoir trouvé les 4
          digitalWrite(ledLaby, HIGH);  // LED finale ON
          donelaby = true;
          // Éteindre toutes les LED
          for (int i = 0; i < 4; i++) {
              digitalWrite(TABLABY[i], LOW);
          }
          vTaskDelay(pdMS_TO_TICKS(100));
          continue;  // Fin du labyrinthe
      }

      // Lire les états des interrupteurs
      int etats[4] = {
          digitalRead(switch1),
          digitalRead(switch2),
          digitalRead(switch3),
          digitalRead(switch4)
      };

      // Vérifie si la switch correspondante a été activée
      if (etats[LED_goal] == LOW) {
          // Éteindre LED actuelle
          if(LED_goal == 4 ){
            mcp2.digitalWrite(TABLABY[4], LOW);
          }else{
            digitalWrite(TABLABY[LED_goal], LOW);
          }
          ledlaby++;  // Incrémenter le nombre d'étapes réussies

          // Choisir une nouvelle LED différente
          int nouvelleLED;
          do {
              nouvelleLED = random(0, 4);  // de 0 à 3
          } while (nouvelleLED == LED_goal);  // éviter la même LED

          LED_goal = nouvelleLED;
          if(LED_goal == 4){
            mcp2.digitalWrite(TABLABY[LED_goal], HIGH);
          }
          else{
            digitalWrite(TABLABY[LED_goal], HIGH);  // Allumer nouvelle LED
          }
          
      }

      vTaskDelay(pdMS_TO_TICKS(50));
  }
}
//fonctionne
void tiroirMagique(void *){
  while(1){
  if (mcp2.digitalRead(switch5) == HIGH && newStateTiroirMag == true){
    mcp2.digitalWrite(LEDTiroirMag,HIGH);
    //lcd.setCursor(0,0);
    //lcd.print("Tiroir ouvert!");
    LCDmessage message = {"Tiroir ouvert!",0};
    xQueueSend(LCDQueue,&message, portMAX_DELAY);
    addPoints(30);
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
    //lcd.setCursor(0,0);
    //lcd.print("Poignee unlocked!");
    //createAddPointsTask(150);
    LCDmessage message = {"Poignee unlocked!",0};
    xQueueSend(LCDQueue,&message, portMAX_DELAY);
    addPoints(5);
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
      //lcd.setCursor(0,0);
      //lcd.print("Tiroir ouvert!");
      //createAddPointsTask(100);
      LCDmessage message = {"Tiroir ouvert!",0};
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      addPoints(20);
      newStateTiroir2 = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
//fonctionne
void usb(void *){
  while(1){
    if (mcp2.digitalRead(switch8) == HIGH && doneUSB == false){
      mcp2.digitalWrite(LEDUSB,HIGH);
      LCDmessage message = {"Termine USB!",0};
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      addPoints(20);
      doneUSB = true;
    }
  vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void xlr(void *){
  while(1)
  {
    if (digitalRead(switch9) == HIGH && newStateXLR ==true){
      mcp2.digitalWrite(LEDXLR,HIGH);
      addPoints(20);
      newStateXLR = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
//fonctionne
void ethernet(void *){
  while(1){
  if (mcp1.digitalRead(switch10) == HIGH && doneEthernet == false){
    mcp1.digitalWrite(LEDEthernet,HIGH);
    LCDmessage message = {"Termine ethernet!",0};
    xQueueSend(LCDQueue,&message, portMAX_DELAY);
    addPoints(20);
    doneEthernet = true;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}
}

void ColorButtons(void *){ 
  while(1){
  if (mcp2.digitalRead(switch11) == LOW){
    doneBoutonGauche = true;
    //Serial.println("Bouton gauche appuyé");
  }
  if (mcp2.digitalRead(switch12) == LOW){
    doneBoutonMilieu = true;
    //Serial.println("Bouton milieu appuyé");
  }
  if (mcp2.digitalRead(switch13) == LOW){
    doneBoutonDroit = true;
    //Serial.println("Bouton droit appuyé");
  }
  if(doneBoutonGauche == true && doneBoutonMilieu == true && doneBoutonDroit == true && doneCouleur == false){
    addPoints(5);
    LCDmessage message;
    snprintf(message.texte,sizeof(message.texte),"Couleur termine!");
    message.ligne = 0;
    xQueueSend(LCDQueue,&message, portMAX_DELAY);
    doneCouleur = true;
  }
    vTaskDelay(pdMS_TO_TICKS(50));
}
}
//fonctionne
void eStop(void *){
  while(1){
    if (mcp2.digitalRead(switch14) == LOW && lockEStop == false){
      addPoints(5);
      LCDmessage message = {"EStop !!!",0};
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      lockEStop = true;
    }
    if(mcp2.digitalRead(switch14) == HIGH && lockEStop == true && doneEStop == false){
      addPoints(15);
      LCDmessage message = {"EStop unlock!",0};
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      doneEStop = true;
    }
  vTaskDelay(pdMS_TO_TICKS(50));
  }
}
//fonctionne
void screw(void *){
  while(1){
    if (mcp1.digitalRead(switch15) == HIGH && newStateScrew == true){
      mcp1.digitalWrite(LEDScrew,HIGH);
      LCDmessage message = {"Termine screw!",0};
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      //lcd.setCursor(0,0);
      //lcd.print("Termine screw!");
      addPoints(15);
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
      LCDmessage message = {"Termine unscrew!",0};
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      //lcd.setCursor(0,0);
      //lcd.print("Termine unscrew!");
      addPoints(15);
      //createAddPointsTask(50);
      newStateUnscrew = false;
    }
  vTaskDelay(pdMS_TO_TICKS(50));
  }
}
//fonctionne
void twoButtonPressed(void *){
  while(1){
    //lcd.print(mcp1.digitalRead(boutons)) ; 
    if (mcp1.digitalRead(boutons) == LOW && newStateBouton == true){
        mcp1.digitalWrite(ledBoutons, HIGH);
        //lcd.setCursor(0,0);
        //lcd.print("Termine tache!");
        LCDmessage message = {"Termine tache ",0};
        xQueueSend(LCDQueue,&message, portMAX_DELAY);
        addPoints(10);
        newStateBouton = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
//fonctionne
void levier(void *){
  while(1){
  if (mcp2.digitalRead(switchlevier) == LOW && doneLevier == false){
    digitalWrite(LEDLevier,HIGH);
    LCDmessage message = {"Levier complete!",0};
    xQueueSend(LCDQueue,&message, portMAX_DELAY);
    addPoints(10);
    doneLevier = true;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}
}

void encodeur1(void *){
    
  while(1){
  // si encoder 1 est tourné
  currentCLK1 = mcp1.digitalRead(CLK1);

    if (currentCLK1 != lastCLK1 && currentCLK1 == 1){

    // si DT différent de CLK, va sens anti-horaire, soustraire CLK au compteur
      if (mcp1.digitalRead(DT1) != currentCLK1){
        counter1--;
      }
      else{
        counter1++;  // si DT pareil à CLK, va dans sens horaire, ajouter CLK au compteur
      }
      LCDmessage message;
      snprintf(message.texte,sizeof(message.texte),"Compteur1: %d",counter1);
      message.ligne = 3;
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
    }

    // si encoder 1 est appuyé, donc il est au GND (LOW)
    if (mcp1.digitalRead(SW1) == LOW && encod1appui == false)
    {
        LCDmessage message;
        snprintf(message.texte,sizeof(message.texte),"Bouton1 appuye");
        message.ligne = 0;
        xQueueSend(LCDQueue,&message, portMAX_DELAY);
        addPoints(5);
        encod1appui = true;
    }
    if (counter1 == number_goal && doneEncodeur1 == false){
        
      mcp1.digitalWrite(LEDEncodeur1,HIGH);
      LCDmessage message;
      snprintf(message.texte,sizeof(message.texte),"Encodeur1 termine");
      message.ligne = 0;
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      addPoints(20);
      doneEncodeur1 = true;
      
    }

    lastCLK1 = currentCLK1;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void encodeur2(void *){
  while(1){
  // si encoder 2 est tourné
  currentCLK2 = mcp1.digitalRead(CLK2);  
  if (currentCLK2 != lastCLK2 && currentCLK2 == 1)
  {
      // si DT différent de CLK, va sens anti-horaire, soustraire CLK au compteur
      if (mcp1.digitalRead(DT2) != currentCLK2)
      {
        counter2--;
      }
      else
      {
        counter2++;  // si DT pareil à CLK, va dans sens horaire, ajouter CLK au compteur
      } 
      LCDmessage compteur2;
      snprintf(compteur2.texte,sizeof(compteur2.texte),"Compteur2:%d",counter2);
      compteur2.ligne = 3;
      xQueueSend(LCDQueue,&compteur2, portMAX_DELAY);
  }

    // si encoder est appuyé, donc il est au GND (LOW)
    if (mcp1.digitalRead(SW2) == LOW && encod2appui == false)
    {
        LCDmessage appui;
        snprintf(appui.texte,sizeof(appui.texte),"Encodeur2 appuye");
        appui.ligne = 0;
        xQueueSend(LCDQueue,&appui, portMAX_DELAY);
        addPoints(5); 
        encod2appui = true;
    }
if (counter2 == number_goal && doneEncodeur2 == false)
    {
        mcp1.digitalWrite(LEDEncodeur2,HIGH);
        LCDmessage message;
        snprintf(message.texte,sizeof(message.texte),"Encodeur2 termine");
        message.ligne = 0;
        xQueueSend(LCDQueue,&message, portMAX_DELAY);
        addPoints(20);
        doneEncodeur2 = true;
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
      LCDmessage compteur3;
      snprintf(compteur3.texte,sizeof(compteur3.texte),"Compteur3:%d",counter3);
      compteur3.ligne = 3;
      xQueueSend(LCDQueue,&compteur3, portMAX_DELAY);
    
  }

  // si encoder est appuyé, donc il est au GND (LOW)
  if (digitalRead(SW3) == LOW && encod3appui == false)
  {
    LCDmessage appui;
    snprintf(appui.texte,sizeof(appui.texte),"Encodeur3 appuye");
    appui.ligne = 0;
    xQueueSend(LCDQueue,&appui, portMAX_DELAY);
    addPoints(5); 
    encod3appui = true;
  }

  if (counter3 == number_goal && doneEncodeur3 == false)
  {
      digitalWrite(LEDEncodeur3,HIGH);
      LCDmessage message;
      snprintf(message.texte,sizeof(message.texte),"Encodeur3 termine");
      message.ligne = 0;
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      addPoints(20);
      doneEncodeur3 = true;
  }

  lastCLK3 = currentCLK3;
  vTaskDelay(pdMS_TO_TICKS(50));
}
}

//fonctionne PAS
void tuyau(void *){
  while(1){
    if(mcp2.digitalRead(capteurIR) == LOW && doneCapteur == false){
      LCDmessage message;
      snprintf(message.texte,sizeof(message.texte),"Termine valve!");
      message.ligne = 0;
      xQueueSend(LCDQueue,&message, portMAX_DELAY);
      addPoints(25);
      doneCapteur = true;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(SDA_PIN,SCL_PIN);
    Wire.begin();
    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.clear();
    Serial.print("ici");

    //Initialize MCP23017 expanders
   if (!mcp1.begin_I2C(0x20)) {
      lcd.print("MCP23017 #1 not found!");
      while (1);
  }
    if (!mcp2.begin_I2C(0x21)) {
      lcd.print("MCP23017 #2 not found!");
      while (1);
  }

    // setup LED
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT); // mal branché
    pinMode(ledLaby, OUTPUT);
    mcp1.pinMode(ledBoutons, OUTPUT);
    mcp2.pinMode(LEDTiroirMag, OUTPUT);
    mcp2.pinMode(LEDPoignee, OUTPUT);
    mcp2.pinMode(LEDTiroir2, OUTPUT);
    mcp1.pinMode(LEDScrew, OUTPUT);
    mcp1.pinMode(LEDUnscrew, OUTPUT);
    pinMode(LEDLevier, OUTPUT);
    mcp1.pinMode(LEDEncodeur1, OUTPUT);
    mcp1.pinMode(LEDEncodeur2, OUTPUT);
    pinMode(LEDEncodeur3, OUTPUT);
    mcp2.pinMode(LEDUSB, OUTPUT);
    mcp2.pinMode(LEDXLR, OUTPUT);
    mcp1.pinMode(LEDEthernet, OUTPUT);

    //setup switch
    pinMode(switch1, INPUT_PULLUP);
    pinMode(switch2, INPUT_PULLUP);
    pinMode(switch3, INPUT_PULLUP);
    pinMode(switch4, INPUT_PULLUP);
    mcp2.pinMode(switch5, INPUT_PULLUP);
    mcp2.pinMode(switch6, INPUT_PULLUP);
    mcp2.pinMode(switch7, INPUT_PULLUP);
    mcp2.pinMode(switch8, INPUT_PULLUP);
    pinMode(switch9, INPUT_PULLUP);
    mcp1.pinMode(switch10, INPUT_PULLUP);
    mcp2.pinMode(switch11, INPUT_PULLUP);
    mcp2.pinMode(switch12, INPUT_PULLUP);
    mcp2.pinMode(switch13, INPUT_PULLUP);
    mcp2.pinMode(switch14, INPUT_PULLUP);
    mcp1.pinMode(switch15, INPUT_PULLUP);
    mcp1.pinMode(switch16, INPUT_PULLUP);
    mcp2.pinMode(switchlevier, INPUT_PULLUP);
    mcp1.pinMode(boutons, INPUT_PULLUP);
    mcp2.pinMode(capteurIR, INPUT_PULLUP);

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
    
    lastCLK1 = mcp1.digitalRead(CLK1);
    lastCLK2 = mcp1.digitalRead(CLK2);
    lastCLK3 = digitalRead(CLK3);
    number_goal = random(-20, 20);
    mcp2.digitalWrite(LED1,HIGH);
    //digitalWrite(LED2,HIGH); //bas droit
    //digitalWrite(LED3,HIGH); //haut gauche
    digitalWrite(LED4,HIGH);
    delay(5000);
      // Labyrinthe
      LED_goal = random(1, 5);
      digitalWrite(TABLABY[LED_goal], HIGH);
      mcp1.digitalWrite(LEDScrew, LOW);
      mcp1.digitalWrite(LEDUnscrew, LOW);
      mcp1.digitalWrite(ledBoutons, LOW);
      mcp2.digitalWrite(LEDPoignee,LOW);
      mcp2.digitalWrite(LEDTiroir2,LOW);
      mcp2.digitalWrite(LEDTiroirMag,LOW);
      mcp2.digitalWrite(LEDUSB,LOW);
      mcp1.digitalWrite(LEDEthernet,LOW);
      mcp1.digitalWrite(LEDEncodeur1, LOW);
      mcp1.digitalWrite(LEDEncodeur2, LOW);
      digitalWrite(LEDEncodeur3, LOW);


      LCDQueue = xQueueCreate(10,sizeof(LCDmessage));
      xTaskCreate(affichageEcran,"affichageEcran",4096, NULL,2,NULL);
      xTaskCreate(screw,"screw", 4096,NULL,2,NULL);
      xTaskCreate(unscrew,"unscrew", 4096,NULL,2,NULL);
      xTaskCreate(labyrinthe,"labyrinthe",4096,NULL,2,NULL);
      xTaskCreate(twoButtonPressed,"twoButtonPressed", 4096,NULL,2,NULL);
      xTaskCreate(eStop,"eStop", 4096,NULL,2,NULL);
      xTaskCreate(ethernet,"ethernet", 4096,NULL,2,NULL);
      xTaskCreate(xlr,"xlr", 4096,NULL,2,NULL);
      xTaskCreate(usb,"usb", 4096,NULL,2,NULL);
      xTaskCreate(tiroirMagique,"tiroirMagique", 4096,NULL,2,NULL);
      xTaskCreate(tiroir2,"tiroir2", 4096,NULL,2,NULL);
      xTaskCreate(poignee,"poignee", 4096,NULL,2,NULL);
      xTaskCreate(levier,"Levier", 4096,NULL,2,NULL);
      xTaskCreate(tuyau,"tuyau",4096, NULL,2,NULL);
      xTaskCreate(encodeur1, "encodeur 1", 4096, NULL, 2, NULL);
      xTaskCreate(encodeur2, "encodeur 2", 4096, NULL, 2, NULL);
      xTaskCreate(encodeur3, "encodeur 3", 4096, NULL, 2, NULL);
  
      // Ecran
      LCDmessage pointage;
      snprintf(pointage.texte,sizeof(pointage.texte),"Pointage:%d",totalPoints);
      pointage.ligne = 1;
      xQueueSend(LCDQueue,&pointage, portMAX_DELAY);
      LCDmessage message;
      snprintf(message.texte,sizeof(message.texte),"Mettre encodeur:%d",number_goal);
      message.ligne = 2;
      xQueueSend(LCDQueue,&message, portMAX_DELAY);

}

void loop(){
}
