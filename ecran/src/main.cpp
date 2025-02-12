#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <wire.h>
#include <hd44780.h>
//#include <lcd.h> 
//#include <math.h>

//connecte ESP1 a ESP2
//#define RX_PIN_1 16 //recevoir info UART1
//#define TX_PIN_1 17 //transmission info UART1

//connecte ESP2 a ESP3
//#define RX_PIN_2 4 //recevoir info UART1
//#define TX_PIN_2 5 //transmission info UART1

//led
#define LED1 27 //LED labyrinth bas gauche
#define LED2 14 //LED labyrinth
#define LED3 12 //LED labyrinth
#define LED4 13 //LED labyrinth
//#define LEDCouleurDroite 30 //LED bouton couleur droit
//#define LEDCouleurMilieu 30 //LED bouton couleur milieu
//#define LEDCouleurGauche 30 //LED bouton couleur gauche
//#define ledLaby 32  //LED labyrinth complete
//#define ledBoutons 14 //LED 2 boutons appuyé complete
//#define LEDTiroirMag 14 //LED tiroir magique complete
//#define LEDPoignee 14 //LED poignée complete
//#define LEDTiroir2 14 //LED tiroir complete
//#define LEDScrew 13 //LED visser complete
//#define LEDUnscrew 14 //LED devisser complete
//#define LEDSelecteur 14 //LED selecteur complete
//#define LEDLevier 14 //LED levier complete
//#define LEDEncodeur1 14 //LED encodeur complete
//#define LEDEncodeur2 14 //LED encodeur complete
//#define LEDEncodeur3 14 //LED encodeur complete
//#define LEDUSB 14 //LED usb complete
//#define LEDXLR 14 //LED XLR complete
//#define LEDEthernet 14 //LED Ethernet complete

//bouton a appuyer en meme temps
#define bouton1 16
#define bouton2 17

//#define SDA_PIN 33
#define SCL_PIN 36

#define RAND_MAX 4

//Encoder 1
#define CLK1 23   
#define DT1 14 //22    
#define SW1 21  

//Encoder 2
#define CLK2 18   
#define DT2 5    
#define SW2 19  

//Encoder 3
#define CLK3 13 //34   
#define DT3 35    
#define SW3 31//32  

//limitSwitch
#define switch1 32  //limitSwitch labyrinth
#define switch2 33  //limitSwitch labyrinth
#define switch3 25  //limitSwitch labyrinth
#define switch4 26  //limitSwitch labyrinth
//#define switch5 31  //tiroir magique
//#define switch6 31  //poignée
//#define switch7 31  //tiroir 2
//#define switch8 31  //clé usb
//#define switch9 31  //XLR
//#define switch10 31  //Ethernet
//#define switch11 31  //Bouton couleur droite
//#define switch12 31  //Bouton couleur milieu
//#define switch13 31  //Bouton couleur gauche
//#define switch14 31  //Emergency stop
//#define switch15 26  //Visser
//#define switch16 31  //Dévisser
//#define switch17 31  //Sélecteur
//#define switch18 31  //Levier

//nb point
#define ptsEncoder 50


int counter1 = 0;
int counter2 = 0;
int counter3 = 0;
int currentCLK1;
int currentCLK2;
int currentCLK3;
int lastCLK1;
int lastCLK2;
int lastCLK3;
int number_goal;
int LED_goal;
int posColorButton;
int limitState = 0;
int del = 34;
bool oldState15;
bool newStateTiroir2 = true ;
bool newStateTiroirMag = true;
bool newStatePoignee = true;
bool newStateUSB = true;
bool newStateXLR = true;
bool newStateEthernet = true;
bool newStateEStop = true;
bool newStateScrew = true;
bool newStateUnscrew = true;
//hw_timer_t * timer = NULL;
int seconde2=0;
int seconde1=0;
int minute1=0;
int minute2=0;


LiquidCrystal_I2C lcd(0x27,20,4);

void affichagePoints(int addPoints){

  int totalPoints = totalPoints + addPoints;
  lcd.setCursor(0,1);
  lcd.print("Pointage:");
  lcd.print(totalPoints);
  lcd.setCursor(19,1);
  lcd.print("*");
  delay(3000);
  lcd.setCursor(19,1);
  lcd.print(" ");

}

void right_position(int id, int currentCLK, int lastCLK){
      
      if(currentCLK != lastCLK){
      Serial.print("Appuyé sur bouton ");
      Serial.print(id);
      affichagePoints(2*ptsEncoder);
      }

}

void ecran(std ::string message){
  
  lcd.setCursor(0, 0);
  //lcd.print(Character);
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("Hello World!");
  delay(1000);
  lcd.setCursor(0, 2);
  lcd.print("Bonjour le monde!");
  delay(1000);
  lcd.setCursor(0, 3);
  lcd.print("Hola Mundo!");

}

void initLaby(){
  if (LED_goal ==1){
    digitalWrite(LED1,HIGH);
  }else if (LED_goal ==2){
    digitalWrite(LED2,HIGH);
  }else if(LED_goal ==3){
    digitalWrite(LED3,HIGH);
  }else{
    digitalWrite(LED4,HIGH);
  }
}

//Partie labyrinthe
void labyrinthe(){
  int ledlaby=0;
  int etatswitch1 = digitalRead(switch1);
  int etatswitch2 = digitalRead(switch2);
  int etatswitch3 = digitalRead(switch3);
  int etatswitch4 = digitalRead(switch4);
    while (ledlaby < 4){ //allumer et eteindre 4 DEL pour avoir pts
      if (LED_goal ==1 && etatswitch1==LOW){
      digitalWrite(LED1,LOW); //eteindre LED
      ledlaby++;
      LED_goal = random(2,4);
      initLaby(); //allumer nouvelle LED
      }
      if (LED_goal==2 && etatswitch2==LOW){ //qd ca marchait cetait else if 
      digitalWrite(LED2,LOW); //eteindre LED
      ledlaby++;
      LED_goal =random(1,4);
      initLaby(); //allumer nouvelle LED
      }
      if (LED_goal == 3 && etatswitch3==LOW){ //qd ca marchait cetait else if 
      digitalWrite(LED3,LOW); //eteindre LED
      ledlaby++;
      LED_goal =random(1,4);
      initLaby(); //allumer nouvelle LED
      }
      if (LED_goal == 4 && etatswitch4== LOW){ //qd ca marchait cetait else if 
      digitalWrite(LED4,LOW); //eteindre LED
      ledlaby++;
      LED_goal =random(1,3);
      initLaby(); //allumer nouvelle LED
      }
    }
    affichagePoints(350);
    //digitalWrite(ledLaby, HIGH);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
}
/*
void tiroirMagique(){
  if (digitalRead(switch5) == LOW && newStateTiroirMag == true){
  digitalWrite(LEDTiroirMag,HIGH);
  lcd.setCursor(0,0);
  lcd.print("Tiroir ouvert!");
  affichagePoints(100);
  newStateTiroirMag = false;
  }
}

void poignee(){
  if (digitalRead(switch6) == HIGH && newStatePoignee == true)
    digitalWrite(LEDPoignee,HIGH);
    lcd.setCursor(0,0);
    lcd.print("Poignee unlocked!");
    affichagePoints(150);
    newStatePoignee = false;
}

void tiroir2(){
  if (digitalRead(switch7) == LOW && newStateTiroir2 == true){
    digitalWrite(LEDTiroir2,HIGH);
    lcd.setCursor(0,0);
    lcd.print("Tiroir ouvert!");
    affichagePoints(100);
    newStateTiroir2 = false;
  }
}

void usb(){
  if (digitalRead(switch8) == HIGH && newStateUSB == true){
    digitalWrite(LEDUSB,HIGH);
    affichagePoints(100);
    newStateUSB = false;
  }
}

void xlr(){
  if (digitalRead(switch9) == HIGH && newStateXLR ==true){
    digitalWrite(LEDXLR,HIGH);
    affichagePoints(150);
    newStateXLR = false;
  }
}

void ethernet(){
  if (digitalRead(switch10) == HIGH && newStateEthernet ==true){
    digitalWrite(LEDEthernet,HIGH);
    affichagePoints(150);
    newStateEthernet = false;
  }
}


void initColorButtons(){ //allumer le bouton de couleur à éteindre
  if (posColorButton ==1){
    digitalWrite(LEDCouleurDroite,HIGH);
  }else if (posColorButton ==2){
    digitalWrite(LEDCouleurMilieu,HIGH);
  }else{
    digitalWrite(LEDCouleurGauche,HIGH);
  }
}

void closeColorButtons(){ //eteindre le bouton de couleur allume
  
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

void eStop(){
  if (digitalRead(switch14) == HIGH && newStateEStop ==true){
    affichagePoints(150);
    lcd.setCursor(0,0);
    lcd.print("FIN DU PANNEAU!");
    newStateEthernet = false;
  }
}

void screw(){
  if (digitalRead(switch15) == HIGH && newStateScrew == true){
    digitalWrite(LEDScrew,HIGH);
    lcd.setCursor(0,0);
    lcd.print("Terminé!");
    affichagePoints(150);
    newStateScrew = false;
  }
}

void unscrew(){
  if (digitalRead(switch16) == LOW && newStateUnscrew == true){
    digitalWrite(LEDUnscrew, HIGH);
    lcd.setCursor(0,0);
    lcd.print("Terminé!");
    affichagePoints(50);
    newStateUnscrew = false;
  }
}

//Partie appuyé sur les deux boutons en même temps
void twoButtonPressed(){
    if (digitalRead(bouton1) == LOW && digitalRead(bouton2) == LOW){
      affichagePoints(50);
      digitalWrite(ledBoutons, HIGH);
    }
}
*/

void timer(){
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
}

void setup() {
  
  //mySerial1.begin(9600, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
  //mySerial2.begin(9600, SERIAL_8N1, RX_PIN_2, TX_PIN_2);

  
  //setup LED
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  //pinMode(led5, OUTPUT);
  //pinMode(led6, OUTPUT);
  //pinMode(led7, OUTPUT);
  //pinMode(ledLaby, OUTPUT); 
  //pinMode(ledBoutons, OUTPUT);
  //pinMode(LEDTiroirMag, OUTPUT);
  //pinMode(LEDPoignee, OUTPUT);
  //pinMode(LEDTiroir2, OUTPUT);
  //pinMode(LEDVis, OUTPUT);
  //pinMode(LEDDevis, OUTPUT);
  //pinMode(LEDSelecteur, OUTPUT);
  //pinMode(LEDLevier, OUTPUT);
  //pinMode(LEDEncodeur1, OUTPUT);
  //pinMode(LEDEncodeur2, OUTPUT);
  //pinMode(LEDEncodeur3, OUTPUT);
  //pinMode(LEDUSB, OUTPUT);
  //pinMode(LEDXLR, OUTPUT);
  //pinMode(LEDEthernet, OUTPUT);

  //setup switch
  pinMode(switch1, INPUT_PULLUP);
  pinMode(switch2, INPUT_PULLUP);
  pinMode(switch3, INPUT_PULLUP);
  pinMode(switch4, INPUT_PULLUP);
  //pinMode(switch5, INPUT_PULLUP);
  //pinMode(switch6, INPUT_PULLUP);
  //pinMode(switch7, INPUT_PULLUP);
  //pinMode(switch8, INPUT_PULLUP);
  //pinMode(switch9, INPUT_PULLUP);
  //pinMode(switch10, INPUT_PULLUP);
  //pinMode(switch11, INPUT_PULLUP);
  //pinMode(switch12, INPUT_PULLUP);
  //pinMode(switch13, INPUT_PULLUP);
  //pinMode(switch14, INPUT_PULLUP);
  //pinMode(switch15, INPUT_PULLUP);
  //pinMode(switch16, INPUT_PULLUP);
  //pinMode(switch17, INPUT_PULLUP);
  //pinMode(switch18, INPUT_PULLUP);
  //pinMode(bouton1, INPUT_PULLUP);
  //pinMode(bouton2, INPUT_PULLUP);
/*
  //Encodeur
    pinMode(CLK1,INPUT);
    pinMode(DT1,INPUT);
    pinMode(SW1,INPUT_PULLUP);
    pinMode(CLK2,INPUT);
    pinMode(DT2,INPUT);
    pinMode(SW2,INPUT_PULLUP);
    pinMode(CLK3,INPUT);
    pinMode(DT3,INPUT);
    pinMode(SW3,INPUT_PULLUP);*/
    Serial.begin(9600);
    delay(1000);
    lastCLK1 = digitalRead(CLK1);
    lastCLK2 = digitalRead(CLK2);
    lastCLK3 = digitalRead(CLK3);
    number_goal = random(-20,20);

  srand(time(0));
  //Ecran
    Wire.begin();
    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.begin(20,4);
    Serial.begin(1000);
    lcd.clear();
    lcd.setCursor(0,1);
    affichagePoints(0); //compteur points a zero
    lcd.setCursor(0,2);
    lcd.print("Mettre encodeurs:");
    lcd.print(number_goal);
  
    //Aléatoire Bouton Couleur
    posColorButton = random(1,3);
    //initColorButtons();

    //Labyrinthe
    LED_goal = random(1,4);
    initLaby();

    //timer = timerBegin(0,80000000,true);

}

void loop(){
  /*
  int etatswitch1 = digitalRead(switch1);
  int etatswitch2 = digitalRead(switch2);
  int etatswitch3 = digitalRead(switch3);
  int etatswitch4 = digitalRead(switch4);*/
  lcd.setCursor(0, 2);
  lcd.write(0);
  //digitalWrite(LED1,HIGH);
  //digitalWrite(LED2,HIGH);
  //digitalWrite(LED3,HIGH);
  //digitalWrite(LED4,HIGH);
  labyrinthe();
  /*
  if (etatswitch1 == LOW){
    digitalWrite(LED1,LOW);
  }
  if (etatswitch2==LOW){
    digitalWrite(LED2,LOW);
  }
  if (etatswitch3==LOW){
    digitalWrite(LED3,LOW);
  }
  if (etatswitch4==LOW){
    digitalWrite(LED4,LOW);
  }
  Serial.print(etatswitch1);*/
  //timer();
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
/*
  tiroirMagique();
  poignee();
  tiroir2();
  usb();
  xlr();
  ethernet();
  closeColorButtons();
  eStop();

  screw();

  unscrew();
  twoButtonPressed();
*/


  //si encoder 1 est tourné
    currentCLK1 = digitalRead(CLK1);
    if(currentCLK1 != lastCLK1 && currentCLK1 == 1){
        
        //si DT différent de CLK, va sens anti-horaire, soustraire CLK au compteur
        if(digitalRead(DT1) != currentCLK1){
            counter1 --;

        }else{
            counter1 ++; //si DT pareil à CLK, va dans sens horaire, ajouter CLK au compteur
        }
        lcd.setCursor(0,3);
        lcd.print("Compteur : ");
        lcd.print(counter1);
        
    }    

    //si encoder 1 est appuyé, donc il est au GND (LOW)
    int buttonState = digitalRead(SW1);
    if(buttonState == LOW){
        lcd.print("Bouton 1 appuyé");
        delay(200);
        affichagePoints(ptsEncoder);
    }
    

    //si encoder 2 est tourné
    currentCLK2 = digitalRead(CLK2);
    if(currentCLK2 != lastCLK2 && currentCLK2 == 1){
        
        //si DT différent de CLK, va sens anti-horaire, soustraire CLK au compteur
        if(digitalRead(DT2) != currentCLK2){
            counter2 --;

        }else{
            counter2 ++; //si DT pareil à CLK, va dans sens horaire, ajouter CLK au compteur
        }
        lcd.setCursor(0,3);
        lcd.print("Compteur 2 : ");
        lcd.print(counter2);
        
    }    


    //si encoder est appuyé, donc il est au GND (LOW)
    int buttonState2 = digitalRead(SW2);
    if(buttonState2 == LOW){
        lcd.print("Bouton 2 appuyé");
        delay(200);
        affichagePoints(ptsEncoder);
    }
    

    //si encoder 3 est tourné
    currentCLK3 = digitalRead(CLK3);
    if(currentCLK3 != lastCLK3  && currentCLK3 == 1){
        
        //si DT différent de CLK, va sens anti-horaire, soustraire CLK au compteur
        if(digitalRead(DT3) != currentCLK3){
            counter3 --;

        }else{
            counter3 ++; //si DT pareil à CLK, va dans sens horaire, ajouter CLK au compteur
        }
        lcd.setCursor(0,3);
        lcd.print("Compteur 3 : ");
        lcd.print(counter3);
        
    }    

    //si encoder est appuyé, donc il est au GND (LOW)
    int buttonState3 = digitalRead(SW3);
    if(buttonState3 == LOW){
        lcd.print("Bouton 3 appuyé");
        delay(200);
        affichagePoints(ptsEncoder);
    }

    if (counter1 == number_goal){
        right_position(1,currentCLK1, lastCLK1);
    }if (counter2 == number_goal){
        right_position(2,currentCLK2, lastCLK2);
    }if(counter3 == number_goal){
        right_position(3, currentCLK3,lastCLK3);
    }

    lastCLK1 = currentCLK1;
    lastCLK2 = currentCLK2;
    lastCLK3 = currentCLK3;
    delay(100);
    
}