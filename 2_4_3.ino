#include <Servo.h>     /*librarie arduino folosita pentru controlul servomotorului - dezactiveaza functia de PWM pe pinii 9 si 10 */
#include <LedControl.h> /*pentru modulul HW-109 matrice leduri */
#include <IRremote.hpp> /*VS 1838B - Ir Receiver!*/

/****************!! SETEAZA modul de operare si daca buzzerul sa fie ON sau OFF ********************************/
int ModOperare = 1;   //  (1)telecomanda / (2) ObstacleAvoidance / (3)LineFollower                                                           
int BuzzerOn_OFF = 0; //  1-On/ 0-OFF                                                                                                        
int NumarIntersectii = 2; //Intersectii pentru LineFollower                                                                                  
/**************** Parametrii robot *****************************************************************************/
float distanta_de_parcurs = 20.00;                         // distanta de parcurs pentru Fata drept -- valoarea o sa se schimbe               
                                                   
int putereMotor = 130;                                      // valoarea initiala a PWM-ului pt motoare                                       
const int motorOffSet = 5;                                  // cu cat sa crestem sau sa scadem valoarea PWM in functie de diferenta d         
const int diametruRoata = 66;                               // in mm / necesar pentru a calcula distanta de deplasare a robotului                
const float circumferintaRoata = 3.14 * diametruRoata;      // circumferinta roata                                                            
const int incrementari_rotatie_completa =40;                // numarul de incrementari ale encoderului pt o rotatie complete                  
 

//LED MATRIX display
int DIN = 12;
int CS = 10;
int CLK  = 11; 
LedControl lc=LedControl(DIN, CLK, CS,0);

// Buzzer pin
int buzzer = 4;
int perioada_beep  = 100;             //timp buzzer in milisecunde 

/**  Servo **/
Servo servo;                          //denumin o clasa de tip Servo conform librarieri Servo.h pentru a putea folosi functiile respective
/** Senzor RPM **/
const int rpmS = 3;                   //modulul optic care citeste sloturile de pe encoderul atasat la roata STANGA
const int rpmD = 2;                   //modulul optic care citeste sloturile de pe encoderul atasat la roata DREAPTA

volatile unsigned long counterS = 0;  //variabila care stocheaza incrementarile de la encoderul STANG
volatile unsigned long counterD = 0;  //variabila care stocheaza incrementarile de la encoderul DREPT

/** Senzor Ultrasonic **/
const byte ultraSoundEcho = 7;   //pin Echo Senzor ultrasonic 
const byte ultraSoundTrig = 9;   //pini Triger Senzor ultrasonic
float duration_us, distance_cm;  //variabile necesare pt a masura si stoca distanta 

/**  senzori infrarosu **/
const byte irStanga = A2;   //Senzor Infrarosu Stanga
const byte irDreapta = A3;  //Senzor Infrarosu Dreapta
const byte irCentru = A1;   //Senzor Infrarosu Centru

/** pin Servomotor */
const byte servoPin = 8;  // variabila de tip "byte" ce stocheaza un unsigned number, from 0 to 255.

/****************************************   MOTOARE    ******************************************************************/
    // Motor stanga                                                                                                      
const byte ENA = 6;         // Pin PWM pt controlul duty cycleului pe motorul stanga                                    
const byte IN1 = A6;        // Pin pentru setarea directiei de rotatie a motorului stanga                                
const byte IN2 = A7;        // Pin pentru setarea directiei de rotatie a motorului stanga                                
                       
    // Motor dreapta                                                                                                      
const byte ENB = 5;         // Pin PWM pt controlul duty cycleului pe motorul dreapta                                    
const byte IN3 = A5;        // Pin pentru setarea directiei de rotatie a motorului dreapta                               
const byte IN4 = A4;        // Pin pentru setarea directiei de rotatie a motorului dreapta                               

/** variabile ce stocheaza distanta la urmatoarele grade: 180 , 157, 135, 112, 90, 68, 45, 22, 0 returnate de functia masurare_Unghiuri
    returneaza 9 valori de la senzorul ultrasonic printr-o singura functie ( pass by pointer*)                                         **/

float stanga180;      // distanta la 180 de grade   (servo la 180)
float stanga157;      // distanta la 157 de grade
float stanga135;      // distanta la 135 de grade
float Stanga112;      // distanta la 112 de grade
float fata90;         // distanta fata (la 90)
float dreapta68;      // distanta la 68 de grade
float dreapta45;      // distanta la 45 de grade
float dreapta22;      // distanta la 22 de grade
float dreapta0;       // distata la  0 grade ( servo la 0)
 

auto myRawdata = IrReceiver.decodedIRData.decodedRawData;   //  variabila ce stocheaza valoarea decodata de sonzorul infrarosul

//________PROTOTIPURI FUNCTII
void LineFollowerV1 ();                       // Line Follower
void obstacleAvoidance();                     // Ocolitor Obstacole
void telecomanda();                          // Control masina prin telecomanda
void verificaComanda_IR();                    // functie ce decoteaza semnalul primit de la telecomanda si stocheaza valoarea in variabila myRawData

/************************************** functii control motoare   *********************************************************************************************************************************/
void mers_fata(int PWM_A, int PWM_B);                         //functie  mers fata necontrolata cu ultrasonic ce masoara distanta                                                                 *
void mers_fata_lf(int PWM_A, int PWM_B);                      //functie  mers fata necontrolata pentru LineFollower (ultrasonicul din functia precedenta incetinea reactia line followerului)     *
void mers_cerc_dreapta(int PWM_A, int PWM_B);                 //functie  mers in cerc spre dreapta necontrolata                                                                                   *
void mers_cerc_stanga(int PWM_A, int PWM_B);                  //functie  mers  in cerc spre stanga necontrolata                                                                                   *

void stop();                                                   // stop motoare (totul 0)                                                                                                          *

void fataDrept( float distanta_de_parcurs, int putereMotor);  //functie pentru mers fata sincronizat prin encodere                                                                                *
void spateDrept(float distanta_de_parcurs, int putereMotor);  // functie pentru mersul in spate sincronizat prin encodere                                                                         *
void Stanga(float distanta_de_parcurs, int putereMotor);       // functie pentru viraj stanga controlat pe o singura roata                                                                        *
void Dreapta(float distanta_de_parcurs, int putereMotor);     // functie pentru viraj dreapta controlat pe o singura roata                                                                        *
void mers_dreapta_controlat (float distanta_de_parcurs, int putereMotor);  //Functie mers controlat prin encodere, pe loc, spre drepata                                                           *
void mers_stanga_controlat (float distanta_de_parcurs, int putereMotor);   //Functie mers controlat prin encodere, pe loc, spre drepata                                                           *
/**************************************************************************************************************************************************************************************************/

float ultrasonic();                                           // functie pentru masurarea distantei de la senzorul ultrasonic
void servomotor();                                            // functie pentru controlul servomotorului de la 0 la 180 grade
void counterSt();                                             // functie ce stocheaza incrementarile de la encoderul stanga
void counterDr();                                             // functie ce stocheaza incrementarile de la encoderul dreapta
void masurare_Unghiuri(float * pStanga180, float * pStanga157, float * pStanga135, float * pStanga112, float * pDreapta0, float * pDreapta22, float * pDreapta68, float * pDreapta45, float *pFata90);  //Functie ce returneaza valorile la unghiuri prestabilite
int indicaDirectia ();      //functie ce compara distantele pana la obstacole in functie de valorile masurate de functia masurare_Unghiuri() si astfel decide directia de deplasare 
void beepBeep(); //functie buzzer        

/****FUNCTII INDICATOARE MATRICE LEDURI **********************/                                                                                                                        
void Matrix_sageata_stanga();                                                                                                               
void Matrix_sageata_dreapta();                                                                                                              
void Matrix_sageata_fata();                                                                                                                 
void Matrix_sageata_spate();                                                                                                                
void Matrix_unghi_45_stanga();                                                                                                              
void Matrix_unghi_45_dreapta();                                                                                                             
void Matrix_stop();                                                                                                                         
void Matrix_idle();                                                                                                                         
void Matrix_cerc_dreapta();                                                                                                                 
void Matrix_cerc_stanga();                                                                                                                  
void Matrix_LineFollower();                                                                                                                 
void Matrix_ObstacleAvoidance();     
void Matrix_minus();
void Matrix_plus();               
void Matrix_egal();           
void Matrix_exclamare();      
void Matrix_model();                                                                  
/***********************************************************/

void setup() {

  Serial.begin(9600);
  
  servo.attach(8);         // Functia servo - pin 8                    
  IrReceiver.begin(13);    //Ir setup??

  pinMode(irStanga, INPUT);     //se configureaza senzorii IR ca si INPUT
  pinMode(irDreapta, INPUT);    //se configureaza senzorii IR ca si INPUT
  pinMode(irCentru, INPUT);     //se configureaza senzorii IR ca si INPUT

  pinMode(ultraSoundTrig, OUTPUT);     //Senzor ultrasonic Trigger - Output 
  pinMode(ultraSoundEcho, INPUT);      //Senzor ultrasonic Echo    - Input

  pinMode(rpmS, INPUT);      // Modul optic stanga  / input
  pinMode(rpmD, INPUT);      // Modul optic dreapta / input

  attachInterrupt (digitalPinToInterrupt(rpmD), counterDr, CHANGE);  //functie arduino pt External Interrupt (de cate ori se schimba starea encoderului din HIGH in LOW se declanseaza interuperea externa si se incrementareaza counterul Dreapta)
  attachInterrupt (digitalPinToInterrupt(rpmS), counterSt, CHANGE);  //functie arduino pt External Interrupt (de cate ori se schimba starea encoderului din HIGH in LOW se declanseaza interuperea externa si se incrementareaza counterul Stanga)

  pinMode(ENA, OUTPUT);   //setare pini motoare ca si OUTPUT pt Directia de rotatie si semnal PWM   
  pinMode(ENB, OUTPUT);   //setare pini motoare ca si OUTPUT pt Directia de rotatie si semnal PWM
  pinMode(IN1, OUTPUT);   //setare pini motoare ca si OUTPUT pt Directia de rotatie si semnal PWM
  pinMode(IN2, OUTPUT);   //setare pini motoare ca si OUTPUT pt Directia de rotatie si semnal PWM
  pinMode(IN3, OUTPUT);   //setare pini motoare ca si OUTPUT pt Directia de rotatie si semnal PWM
  pinMode(IN4, OUTPUT);   //setare pini motoare ca si OUTPUT pt Directia de rotatie si semnal PWM
 
  lc.shutdown(0,false);     //matrix led setup
  lc.setIntensity(0,0);     //matrix led setup
  lc.clearDisplay(0);       //matrix led setup

  pinMode(buzzer, OUTPUT);         //buzzer setat ca output
  digitalWrite(buzzer, HIGH);      // E low trigger astfel il setat ca HIGH ca sa fie silent inca din setup

}

void loop(){

     if (ModOperare==1)
           { telecomanda();}  
     if (ModOperare==2) 
           {obstacleAvoidance();}
     if (ModOperare==3)
           { LineFollowerV1();}
     if(ModOperare!=1 && ModOperare!=2 && ModOperare!=3)
        {
          Matrix_model();
        }
            
            } /*******************  Sfarsit main LOOOP ******************/


void verificaComanda_IR() 
     {
       if(IrReceiver.decode()) {
                 myRawdata = IrReceiver.decodedIRData.decodedRawData;
                IrReceiver.resume();}
      }  //Sfarsit functie verificaComanda_IR () 
void telecomanda()                                                                         // ------- CONTROL cu TELECOMANDA -----------
     {
        verificaComanda_IR();
        switch (myRawdata)
            {
              /*  CH-  */ case 0xBA45FF00 :  
              /*  CH   */ case 0xB946FF00 : masurare_Unghiuri(&stanga180, &stanga157, &stanga135, &Stanga112,   &dreapta0, &dreapta22, &dreapta68, &dreapta45, &fata90); break;
              /*  CH+  */ case 0xB847FF00 : Matrix_exclamare();  break;
              /*  |<<  */ case 0xBB44FF00 : Matrix_cerc_stanga();  mers_stanga_controlat  (20.0, putereMotor);  break;
              /*  >>|  */ case 0xBF40FF00 : Matrix_cerc_dreapta(); mers_dreapta_controlat (20.0, putereMotor);  break;
              /*  >||  */ case 0xBC43FF00 :   break;
              /*  -    */ case 0xF807FF00 : Matrix_cerc_stanga();  mers_stanga_controlat  (40.0, putereMotor);  break;
              /*  +    */ case 0xEA15FF00 : Matrix_cerc_dreapta(); mers_dreapta_controlat (40.0, putereMotor);  break;
              /*  EQ   */ case 0xF609FF00 :   break;
              /*  FOL- */ case 0xE619FF00 : Matrix_model(); break;
              /*  FOL+ */ case 0xF20DFF00 :   break;
              /*  0    */ case 0xE916FF00 :   break;
              /*  1    */ case 0xF30CFF00 :   Matrix_unghi_45_stanga();   mers_stanga_controlat (5.00, putereMotor);                 break;
              /*  2    */ case 0xE718FF00 :   Matrix_sageata_fata();      fataDrept(distanta_de_parcurs, putereMotor);               break;
              /*  3    */ case 0xA15EFF00 :   Matrix_unghi_45_dreapta();  mers_dreapta_controlat (5.00, putereMotor);                break;
              /*  4    */ case 0xF708FF00 :   Matrix_unghi_45_stanga();   mers_stanga_controlat ( 7.50, putereMotor);                break;
              /*  5    */ case 0xE31CFF00 :   Matrix_stop();              stop();                                                    break;
              /*  6    */ case 0xA55AFF00 :   Matrix_unghi_45_dreapta();  mers_dreapta_controlat (7.50, putereMotor);                break;
              /*  7    */ case 0xBD42FF00 :   Matrix_sageata_stanga();    mers_stanga_controlat (10.0, putereMotor);                 break;
              /*  8    */ case 0xAD52FF00 :   Matrix_sageata_spate();     spateDrept(distanta_de_parcurs, putereMotor);              break;
              /*  9    */ case 0xB54AFF00 :   Matrix_sageata_dreapta();   mers_dreapta_controlat (10.0, putereMotor);                break;
                                   default:   Matrix_idleNoBeep();
                                                           }     
     } //sfarsit functie telecomanda()
void LineFollowerV1()                                                                      // ------ LINE FOLLOWER -----------
    {
        int  IrS = digitalRead(irStanga);   //variabila IR Stanga
        int  IrC = digitalRead(irCentru);   //variabila IR Centru
        int  IrD = digitalRead(irDreapta);  //variabila IR Dreapta
 
      if ((IrS==0) && (IrC == 1) && (IrD==0))                                     //linia neagra sub senzor Centru -> mergi fata
             {mers_fata_lf(120, 120);
               } 
      else if (IrS == 0 && IrC == 0 && IrD == 0)                                  //linia neagra intre senzori sau in afara lor                         
            { unsigned long timp = millis();
              static unsigned long timpAnterior = 0;
              unsigned long perioada = 700;
                                                                                            
              if (timp+timpAnterior<perioada) {mers_fata_lf(130, 130);}                                                  
                                                           
              else if (timp - timpAnterior >= perioada )//&&  
                  {mers_spate(130, 130); 
                   timpAnterior = timp;
                    }
            } //sfarsit else if (0 0 0)     
      else if ( IrS == 1 && IrC == 0 && IrD == 0) {mers_fata_lf (20, 120);}         //linia neagra sub senzor stanga
      else if ( IrS == 1 && IrC == 1 && IrD == 0) {mers_fata_lf (20, 120);}         //linia neagra sub senzor stanga si senzor centru - unghi 90 grade spre stanga
      else if ( IrD == 1 && IrC == 0 && IrS == 0) {mers_fata_lf( 120, 20);}         //linia neagra sub senzor dreapta
      else if ( IrD == 1 && IrC == 1 && IrS == 0) {mers_fata_lf( 120, 20);}         //linia neagra sub senzor dreapta si senzor centru - unghi 90 grade spre dreapta

      else if(IrS == 1 && IrC == 1 && IrD == 1)                                     //linia neagra sub toti senzorii - intersectie
          {   static int counterLinie=0;
              unsigned long time = millis();
              static unsigned long currentTime=0;
              unsigned long perioada = 150;

             if(time-currentTime>=perioada)
                { 
                  if( IrS == 1 && IrC == 1 && IrD == 1)
                      {counterLinie++; 
                       currentTime=time;
                       }
                   if(counterLinie>NumarIntersectii)
                        {
                            do {
                            Serial.println("de aici doar cu hard reset se iasa!");
                            mers_fata_lf(0, 0);
                            Matrix_idle(); 
                               }
                          while(true); 
                       }
                }
          }
      } //sfarsit functie lineFollower

void obstacleAvoidance(){  // ----- OCOLITOR OBSTACOLE       -----
                   
 
    masurare_Unghiuri(&stanga180, &stanga157, &stanga135, &Stanga112,   &dreapta0, &dreapta22, &dreapta68, &dreapta45, &fata90); 

    switch(indicaDirectia())
          {   
           case 180 : //Serial.println("Caz 180: Virez stanga pana la 180 grade:");
                      mers_stanga_controlat (10.00, 130);      
                      ultrasonic();
                      distanta_de_parcurs = distance_cm-10.00;
                      fataDrept(distanta_de_parcurs , putereMotor);
                      break;

           case 157 : mers_stanga_controlat (7.5, 130);
                      ultrasonic();
                      distanta_de_parcurs = distance_cm-10.00;
                      fataDrept(distanta_de_parcurs , putereMotor);
                      break;

          case 135 :  mers_stanga_controlat (5.00, 130);
                      ultrasonic();
                      distanta_de_parcurs = distance_cm-10.00;
                      fataDrept( distanta_de_parcurs, putereMotor); 
                      break;

         case 112 :   mers_stanga_controlat (2.5, 130);
                      ultrasonic();
                      distanta_de_parcurs = distance_cm-10.00;
                      fataDrept( distanta_de_parcurs, putereMotor); 
                      break;

         case 90 :    ultrasonic();
                      distanta_de_parcurs = distance_cm-10.00;
                      fataDrept( distanta_de_parcurs, putereMotor); 
                      break;

         case 68 :    mers_dreapta_controlat (2.5, 130);
                      ultrasonic();
                      distanta_de_parcurs =distance_cm-10.00;
                      fataDrept( distanta_de_parcurs, putereMotor); 
                      break;

         case 45 :    mers_dreapta_controlat (5.0, 130); 
                      ultrasonic();
                      distanta_de_parcurs = distance_cm-10.00;
                      fataDrept( distanta_de_parcurs, putereMotor);   
                      break;

          case 22:    mers_dreapta_controlat (7.0, 130);
                      ultrasonic();
                      distanta_de_parcurs = distance_cm-10.00;
                      fataDrept( distanta_de_parcurs, putereMotor); 
                      break;

         case 0:      mers_dreapta_controlat (9.0, 130); 
                      ultrasonic();
                      distanta_de_parcurs = distance_cm-10.00;
                      fataDrept( distanta_de_parcurs, putereMotor);   
                      break;

          case 500:   spateDrept(15, putereMotor); 
                      break;

          case 1000:   stop();
                      break;

           default:  stop();
        } //final switch
    }  //final functie obstacleAvoidance()

  int indicaDirectia () { //functie ce compara toate directiile masurate si stabileste cea mai mare distanta returnand un int sugestiv se urmeaza sa fie folosit intr-un switch pentru a stabili directia de pl
      float distantaMinima = 30.00;
      float distantaMaxima = 250.00;
      int x;
                        if( 
                          (stanga180>=distantaMinima && stanga180<distantaMaxima)  &&
                          (stanga180>stanga157)                                    &&
                          (stanga180>stanga135)                                    &&
                          (stanga180>Stanga112)                                    &&
                          (stanga180>dreapta0)                                     &&
                          (stanga180>dreapta22)                                    &&
                          (stanga180>dreapta68)                                    &&
                          (stanga180>dreapta45)                                    &&
                          (stanga180>fata90)                    
                          )
                          { x=180; }
                        else if( 
                          (stanga157>=distantaMinima && stanga157<distantaMaxima)  &&
                          (stanga157>stanga180)                                    &&
                          (stanga157>stanga135)                                    &&
                          (stanga157>Stanga112)                                    &&
                          (stanga157>dreapta0)                                     &&
                          (stanga157>dreapta22)                                    &&
                          (stanga157>dreapta68)                                    &&
                          (stanga157>dreapta45)                                    &&
                          (stanga157>fata90)                    
                          )
                          { x =157; }
                        else if ( 
                          (stanga135>=distantaMinima && stanga135<distantaMaxima)  &&
                          (stanga135>stanga180)                                    &&
                          (stanga135>stanga157)                                    &&
                          (stanga135>Stanga112)                                    &&
                          (stanga135>dreapta0)                                     &&
                          (stanga135>dreapta22)                                    &&
                          (stanga135>dreapta68)                                    &&
                          (stanga135>dreapta45)                                    &&
                          (stanga135>fata90)                    
                          )
                          { x =135; }
                        else if ( 
                          (Stanga112>=distantaMinima && Stanga112<distantaMaxima)  &&
                          (Stanga112>stanga180)                                    &&
                          (Stanga112>stanga157)                                    &&
                          (Stanga112>stanga135)                                    &&
                          (Stanga112>dreapta0)                                     &&
                          (Stanga112>dreapta22)                                    &&
                          (Stanga112>dreapta68)                                    &&
                          (Stanga112>dreapta45)                                    &&
                          (Stanga112>fata90)                    
                          )
                          { x = 112; }       
                        else if ( 
                          (dreapta0>=distantaMinima && dreapta0<distantaMaxima)    &&
                          (dreapta0>stanga180)                                     &&
                          (dreapta0>stanga157)                                     &&
                          (dreapta0>stanga135)                                     &&
                          (dreapta0>Stanga112)                                     &&
                          (dreapta0>dreapta22)                                     &&
                          (dreapta0>dreapta68)                                     &&
                          (dreapta0>dreapta45)                                     &&
                          (dreapta0>fata90)                    
                          )
                          { x = 0; }       
                        else if ( 
                          (dreapta22>=distantaMinima && dreapta22<distantaMaxima)  &&
                          (dreapta22>stanga180)                                    &&
                          (dreapta22>stanga157)                                    &&
                          (dreapta22>stanga135)                                    &&
                          (dreapta22>Stanga112)                                    &&
                          (dreapta22>dreapta0)                                     &&
                          (dreapta22>dreapta68)                                    &&
                          (dreapta22>dreapta45)                                    &&
                          (dreapta22>fata90)                    
                          )
                          { x = 22; } 
                        else if ( 
                          (dreapta45>=distantaMinima && dreapta45<distantaMaxima)  &&
                          (dreapta45>stanga180)                                    &&
                          (dreapta45>stanga157)                                    &&
                          (dreapta45>stanga135)                                    &&
                          (dreapta45>Stanga112)                                    &&
                          (dreapta45>dreapta0)                                     &&
                          (dreapta45>dreapta68)                                    &&
                          (dreapta45>dreapta22)                                    &&
                          (dreapta45>fata90)                    
                          )
                          { x = 45; } 
                        else if( 
                          (dreapta68>=distantaMinima && dreapta68<distantaMaxima)  &&
                          (dreapta68>stanga180)                                    &&
                          (dreapta68>stanga157)                                    &&
                          (dreapta68>stanga135)                                    &&
                          (dreapta68>Stanga112)                                    &&
                          (dreapta68>dreapta0)                                     &&
                          (dreapta68>dreapta45)                                    &&
                          (dreapta68>dreapta22)                                    &&
                          (dreapta68>fata90)                    
                          )
                          { x = 68; } 
                        else if ( 
                          (fata90>=distantaMinima && fata90<distantaMaxima)        &&
                          (fata90>stanga180)                                       &&
                          (fata90>stanga157)                                       &&
                          (fata90>stanga135)                                       &&
                          (fata90>Stanga112)                                       &&
                          (fata90>dreapta0)                                        &&
                          (fata90>dreapta45)                                       &&
                          (fata90>dreapta22)                                       &&
                          (fata90>dreapta68)                    
                          )
                          { x = 90; } 

                        else if (
                          (stanga180<=30.00 || stanga180>2.00)  &&              
                          (stanga157<=30.00 || stanga157>2.00 ) &&               
                          (stanga135<=30.00 || stanga135>2.00 ) &&             
                          (Stanga112<=30.00 || Stanga112>30.00) &&         
                          (dreapta0 <=30.00 || dreapta0>2.00 )  &&         
                          (dreapta22<=30.00 || dreapta22>2.00)  &&        
                          (dreapta68<=30.00 || dreapta68>30.00) &&       
                          (dreapta45<=30.00 || dreapta45>2.00)  &&          
                          (fata90   <=30.00 || fata90>2.00 )  
                         )
                          { x=500; }

                        else if( 
                          (stanga180<=2.00) &&              
                          (stanga157<=2.00) &&               
                          (stanga135<=2.00) &&             
                          (Stanga112<=2.00) &&         
                          (dreapta0 <=2.00) &&         
                          (dreapta22<=2.00) &&        
                          (dreapta68<=2.00) &&       
                          (dreapta45<=2.00) &&          
                          (fata90   <=2.00)  
                          )
                          { x=1000; }
                         Serial.print("Directie aleasa: "); Serial.println(x);
                         return x;  
                  }  //sfarsit functie indicaDirectia ()

void masurare_Unghiuri(float * pStanga180, float * pStanga157, float * pStanga135, float * pStanga112, float * pDreapta0, float * pDreapta22, float * pDreapta68, float * pDreapta45, float *pFata90){    // ----- DISTANTA 0 45 90 135 180  cu Delay----- 
          //****functie care verifica distanta stanga dreapta si returneaza valorile prin pointers 
         servomotor(0); 
         Matrix_sageata_dreapta(); 
        * pDreapta0 = ultrasonic();

        servomotor(22);  
        Matrix_unghi_45_dreapta();
        * pDreapta22 = ultrasonic();

        servomotor(45);  //48
        Matrix_unghi_45_dreapta();
        * pDreapta45 = ultrasonic();

        servomotor(68);  //6
         Matrix_unghi_45_dreapta();
        * pDreapta68 = ultrasonic();

        servomotor(90);  
        Matrix_sageata_fata ();
        * pFata90 = ultrasonic();

        servomotor(112); 
         Matrix_unghi_45_stanga(); 
        * pStanga112 = ultrasonic();

        servomotor(135);  //135
        Matrix_unghi_45_stanga ();
        * pStanga135  = ultrasonic();

        servomotor(157);   
        Matrix_unghi_45_stanga();
        * pStanga157  = ultrasonic();

        servomotor(180); 
         Matrix_sageata_stanga(); 
        * pStanga180  =  ultrasonic();

        servomotor(95);
        

        // Serial.println("-------debug din functia masurare_Unghiuri -------------");
        // Serial.print("Distanga stanga la 180 grade: "); Serial.println(stanga180);
        // Serial.print("Distanga stanga la 157 grade: "); Serial.println(stanga157);
        // Serial.print("Distanga stanga la 135 grade: "); Serial.println(stanga135);
        // Serial.print("Distanga stanga la 112 grade: "); Serial.println(Stanga112);
        // Serial.print("Distanga  fata  la 90 grade: ");    Serial.println(fata90);
        // Serial.print("Distanga dreapta la 68 grade: "); Serial.println(dreapta68);
        // Serial.print("Distanga dreapta la 45 grade: "); Serial.println(dreapta45);
        // Serial.print("Distanga dreapta la 22 grade: "); Serial.println(dreapta22);
        // Serial.print("Distanga dreapta la 0  grade: ");  Serial.println(dreapta0);
        // Serial.println("---------------------------------------------------------");
      }  //sfarsit functie masurare_Unghiuri()


void fataDrept(float distanta_de_parcurs, int putereMotor){                               // ----- FATA DREPT               ----- 
      unsigned long nrRotiri_s; // variabile necesare pt a determina in ce directie sa ajustam duty cyclelul
      unsigned long nrRotiri_d; // variabile necesare pt a determina in ce directie sa ajustam duty cyclelul
 
      unsigned long diff_s;     // variabile necesare pt a determina in ce directie sa ajustam duty cyclelul
      unsigned long diff_d;     // variabile necesare pt a determina in ce directie sa ajustam duty cyclelul
      
      counterS = 0;  // resetam counterele la 0
      counterD = 0;   // resetam counterele la 0
      
      unsigned long counterS_anterior = counterS;   // valoarea masurata anterior egala cu valoarea de la countere
      unsigned long counterD_anterior = counterD;   // valoarea masurata anterior egala cu valoarea de la countere
       
      int PWM_A = putereMotor;      //setam putere initiala a motoarelor egala cu cea din parametrii
      int PWM_B = putereMotor;      //setam putere initiala a motoarelor egala cu cea din parametrii
      
      float num_rotiri = (distanta_de_parcurs * 10) / circumferintaRoata;         // calculam numarul de rotiri ale rotii necesare pentru a ajunge la distanta dorita
      unsigned long target_count = num_rotiri * incrementari_rotatie_completa;    // calculam numarul de incrementari ale counterelor necesare pentru a ajunge la distanta dorita
      
      while ((counterS < target_count) && (counterD < target_count)){             // instructiune repetitiva de tip while ce se executa pana cand ajungem la numarul de incrementari calculat anterior (distanta dorita)
            
              nrRotiri_s = counterS;          // salveaza numarul de rotiri de la encoder la momentul asta
              nrRotiri_d = counterD;          // salveaza numarul de rotiri de la encoder la momentul asta
              
              mers_fata(PWM_A, PWM_B);        // mers fata
              
              diff_s = nrRotiri_s - counterS_anterior;    // stocheaza numarul de rotiri pe fiecare encoder la ultima verificare
              diff_d = nrRotiri_d - counterD_anterior;    // stocheaza numarul de rotiri pe fiecare encoder la ultima verificare
              
              counterS_anterior = nrRotiri_s;             // salveaza numarul de rotiri pentru urmatorul loop
              counterD_anterior = nrRotiri_d;             // salveaza numarul de rotiri pentru urmatorul loop
              
               if (diff_s > diff_d){                      // in functie de diferenta dintre numarul de rotiri, daca stanga e mai rapid, incetineste si accelereaza dreapata
                        PWM_A -= motorOffSet;
                        PWM_B += motorOffSet;}
               
               else if (diff_s < diff_d){                 // in functie de diferenta dintre numarul de rotiri, daca dreapta e mai rapid, incetineste dreapta si accelereaza stanga
                        PWM_A += motorOffSet;
                        PWM_B -= motorOffSet; }
               }
          stop();      
       }//sfarsit functie! fataDrept
void mers_fata(int PWM_A, int PWM_B)                         // ----- FATA FARA SINCRONIZARE   ----- 
        {     
            ultrasonic();
            if(distance_cm>=20.00)
               {
                  //Serial.println("Merg in fata din functia mers_fata");
                  // Serial.print("distanta de parcurs:"); Serial.println(distanta_de_parcurs);
                 digitalWrite(IN1, HIGH);
                 digitalWrite(IN2, LOW);
                 digitalWrite(IN3, LOW);
                 digitalWrite(IN4, HIGH);
                 analogWrite(ENA, PWM_A);
                 analogWrite(ENB, PWM_B);}
            else if (distance_cm<20.00)
                {
                  stop();
                  obstacleAvoidance();}
         } //sfarsit functie mers_fata

void mers_fata_lf(int PWM_A, int PWM_B)                      // ----- FATA FARA SINCRONIZARE pt LINEFOLLOWER  ----- 
        {     
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, HIGH);
          analogWrite(ENA, PWM_A);
          analogWrite(ENB, PWM_B);
        } //sfarsit functie 
void mers_cerc_dreapta(int PWM_A, int PWM_B)                 // ----- CERC DREAPTA PE LOC FARA SINCRONIZARE   ----- 
        {                                                                                                                 
          digitalWrite(IN1, HIGH);    //  -- fata stanga
          digitalWrite(IN2, LOW);     //  -- spate stanga
          digitalWrite(IN3, HIGH);    //  -- spate dreapta
          digitalWrite(IN4, LOW);     //  -- fata dreapta
          analogWrite(ENA, PWM_A);
          analogWrite(ENB, PWM_B);
        } //sfarsit functie mers_fata()
void mers_cerc_stanga(int PWM_A, int PWM_B)                  // ----- CERC STANGA PE LOC FARA SINCRONIZARE   ----- 
        {                                                                                                                 
          digitalWrite(IN1, LOW);      //   -- fata stanga
          digitalWrite(IN2, HIGH);     //   -- spate stanga
          digitalWrite(IN3, LOW);      //   -- spate dreapta
          digitalWrite(IN4, HIGH);     //   -- fata dreapta
          analogWrite(ENA, PWM_A);
          analogWrite(ENB, PWM_B);
        } //sfarsit functie mers_fata()v
void mers_dreapta_controlat(float distanta_de_parcurs, int putereMotor){                                                                                         
      unsigned long nrRotiri_s;
      unsigned long nrRotiri_d;
      unsigned long diff_s;
      unsigned long diff_d;
      counterS = 0;
      counterD = 0;
      unsigned long counterS_anterior = counterS;
      unsigned long counterD_anterior = counterD;
      //setam putere motoare egala cu cea din parametrii
      int PWM_A = putereMotor;
      int PWM_B = putereMotor;
      float num_rotiri = (distanta_de_parcurs * 10) / circumferintaRoata;  
      unsigned long target_count = num_rotiri * incrementari_rotatie_completa;
      // mers pana cand motoarele ajung la counterul setat ca si target
    while ((counterS < target_count) && (counterD < target_count)) {
            // salveaza numarul de rotiri de la encoder la momentul asta
           nrRotiri_s = counterS;
           nrRotiri_d = counterD;
           mers_cerc_dreapta( PWM_A,  PWM_B);
           // Calculeaza numarul de rotiri pe fiecare encoder la ultima verificare
           diff_s = nrRotiri_s - counterS_anterior;
           diff_d = nrRotiri_d - counterD_anterior;
           // salveaza numarul de rotiri pentru urmatorul loop
           counterS_anterior = nrRotiri_s;
           counterD_anterior = nrRotiri_d;
           // in functie de diferenta dintre numarul de rotiri, daca stanga e mai rapid, incetineste si accelereaza dreapata
           if (diff_s > diff_d) {
             PWM_A -= motorOffSet;
             PWM_B += motorOffSet;
             }
           // in functie de diferenta dintre numarul de rotiri, daca dreapta e mai rapid, incetineste si accelereaza stanga
           else if (diff_s < diff_d) {
             PWM_A += motorOffSet;
             PWM_B -= motorOffSet;
             }
           }
      stop();
     } //sfarsit functie mers_dreapta_controlat()

void mers_stanga_controlat(float distanta_de_parcurs, int putereMotor){                                                                                          
      unsigned long nrRotiri_s;
      unsigned long nrRotiri_d;
      unsigned long diff_s;
      unsigned long diff_d;
      counterS = 0;
      counterD = 0;
      unsigned long counterS_anterior = counterS;
      unsigned long counterD_anterior = counterD;
      //setam putere motoare egala cu cea din parametrii
      int PWM_A = putereMotor;
      int PWM_B = putereMotor;
      float num_rotiri = (distanta_de_parcurs * 10) / circumferintaRoata;  
      unsigned long target_count = num_rotiri * incrementari_rotatie_completa;
          // mers pana cand motoarele ajung la counterul setat ca si target
      while ((counterS < target_count) && (counterD < target_count)){
           // salveaza numarul de rotiri de la encoder la momentul asta
           nrRotiri_s = counterS;
           nrRotiri_d = counterD;
           mers_cerc_stanga( PWM_A,  PWM_B);
           // Calculeaza numarul de rotiri pe fiecare encoder la ultima verificare
           diff_s = nrRotiri_s - counterS_anterior;
           diff_d = nrRotiri_d - counterD_anterior;
           // salveaza numarul de rotiri pentru urmatorul loop
           counterS_anterior = nrRotiri_s;
           counterD_anterior = nrRotiri_d;
           // in functie de diferenta dintre numarul de rotiri, daca stanga e mai rapid, incetineste si accelereaza dreapata
           if (diff_s > diff_d) {
             PWM_A -= motorOffSet;
             PWM_B += motorOffSet;
              }
           // in functie de diferenta dintre numarul de rotiri, daca dreapta e mai rapid, incetineste si accelereaza stanga
           else if (diff_s < diff_d) {
             PWM_A += motorOffSet;
             PWM_B -= motorOffSet;
              }
           delay(10);
         }
    stop();
  }  //sfarsit functie mers_stanga_controlat()
void spateDrept(float distanta_de_parcurs, int putereMotor){        // ----- SPATE DREPT              ----- 
     unsigned long nrRotiri_s;
     unsigned long nrRotiri_d;
     unsigned long diff_s;
     unsigned long diff_d;
     counterS = 0;
     counterD = 0;
     unsigned long counterS_anterior = counterS;
     unsigned long counterD_anterior = counterD;
     //setam putere motoare egala cu cea din parametrii
     int PWM_A = putereMotor;
     int PWM_B = putereMotor;
     float num_rotiri = (distanta_de_parcurs * 10) / circumferintaRoata;  
     unsigned long target_count = num_rotiri * incrementari_rotatie_completa;
     // mers pana cand motoarele ajung la counterul setat ca si target
     while((counterS < target_count) && (counterD < target_count)) {
        // salveaza numarul de rotiri de la encoder la momentul asta
        nrRotiri_s = counterS;
        nrRotiri_d = counterD;
        // mers spate
        mers_spate(PWM_A, PWM_B);
        // Calculeaza numarul de rotiri pe fiecare encoder la ultima verificare
        diff_s = nrRotiri_s - counterS_anterior;
        diff_d = nrRotiri_d - counterD_anterior;
        // salveaza numarul de rotiri pentru urmatorul loop
        counterS_anterior = nrRotiri_s;
        counterD_anterior = nrRotiri_d;
        // in functie de diferenta dintre numarul de rotiri, daca stanga e mai rapid, incetineste si accelereaza dreapata
     if (diff_s > diff_d) {
         PWM_A -= motorOffSet;
         PWM_B += motorOffSet;
        }
        // in functie de diferenta dintre numarul de rotiri, daca dreapta e mai rapid, incetineste si accelereaza stanga
     else if (diff_s < diff_d) {
         PWM_A += motorOffSet;
         PWM_B -= motorOffSet;
       }
     }
   stop();
 }  //sfarsit functie spateDrept
void mers_spate(int PWM_A, int PWM_B){                              // ----- SPATE FARA SINCRONIZARE  ----- 
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, PWM_A);
        analogWrite(ENB, PWM_B);
        } //sfarsit functie mers_spate
void stop() {                                                       // ----- STOP                     ----- 
        analogWrite(IN1, LOW);
        analogWrite(IN2, LOW);
        analogWrite(IN3, LOW);
        analogWrite(IN4, LOW);
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        } //sfarsit functie stop
void counterDr() {                                                  // ----- COUNTER DREAPTA          -----
       counterD++;
       }  //sfarsit functie counterDr
void counterSt() {                                                  // ----- COUNTER STANGA           -----
       counterS++;
       }    //sfarsit functie counterS
float ultrasonic() {                                                // ----- ULTRASONIC - Sensor HC-SR04      -----
          digitalWrite(ultraSoundTrig, LOW);
          delayMicroseconds(2);
          digitalWrite(ultraSoundTrig, HIGH);
          delayMicroseconds(10);
          digitalWrite(ultraSoundTrig, LOW);
      duration_us = pulseIn(ultraSoundEcho, HIGH);  // masoara timpul de raspuns de la ECHO pin
      distance_cm = 0.017 * duration_us;           
      return distance_cm = constrain(distance_cm, 2.0, 400.0);
      } //sfarsit functie ultrasonic
void servomotor(int x) {                                            // ----- SERVOMOTOR               -----
       servo.write(x);
      } //sfarsit functie servomotor
void Matrix_sageata_stanga(){
       byte Sageata_stanga[8] ={B00000000,
                                B00100000,
                                B01100000,
                                B11111111,
                                B11111111,
                                B01100000,
                                B00100000,
                                B00000000};
       for(int i=8;i>=0; --i) 
          {lc.setRow(0,i,Sageata_stanga[i]); 
          delay(10);
          }
          beepBeep();
          lc.clearDisplay(0);
      }  //sfarsit functie Matrix_sageata_stanga()
void Matrix_sageata_dreapta(){
        byte Sageata_dreapta[8] = {B00000000,
                                   B00000100,
                                   B00000110,
                                   B11111111,
                                   B11111111,
                                   B00000110,
                                   B00000100,
                                   B00000000};
        for(int i=0;i<=8; i++) 
           {lc.setRow(0,i,Sageata_dreapta[i]); 
           delay(10);
           }
           beepBeep();
           lc.clearDisplay(0);
       } //sfarsit functie Matrix_sageata_dreapta()
void Matrix_sageata_fata(){
         byte Sageata_fata[8] = {B00011000,
                                 B00111100,
                                 B01111110,
                                 B00011000,
                                 B00011000,
                                 B00011000,
                                 B00011000,
                                 B00011000};
         for(int i=8;i>=0;--i) 
            {
              lc.setRow(0,i,Sageata_fata[i]);
              delay(10);  
            }
            beepBeep();
            lc.clearDisplay(0);
            } //sfarsit functie Matrix_sageata_fata()
void Matrix_unghi_45_stanga(){
         byte Unghi_stanga[8] = {B11111100,
                                 B11110000,
                                 B11110000,
                                 B11111000,
                                 B10011100,
                                 B00001110,
                                 B00000111,
                                 B00000011};
         for(int i=8;i>=0;--i) 
            {lc.setRow(0,i,Unghi_stanga[i]);
            delay(10);
            }
            beepBeep();
            lc.clearDisplay(0);
            } //sfarsit functie Matrix_unghi_45_stanga()
void Matrix_unghi_45_dreapta(){
          byte Unghi_dreapta[8] ={B00011111,
                                  B00001111,
                                  B00001111,
                                  B00011111,
                                  B00111001,
                                  B01110000,
                                  B11100000,
                                  B11000000};
         for(int i=8;i>=0;--i) 
            {lc.setRow(0,i,Unghi_dreapta[i]);
            delay(10);
            }
            beepBeep();
            lc.clearDisplay(0);
            }  //sfarsit functie Matrix_unghi_45_dreapta()
void Matrix_sageata_spate(){
            byte Sageata_spate[8] = {B00011000,
                                     B00011000,
                                     B00011000,
                                     B00011000,
                                     B00011000,
                                     B01111110,
                                     B00111100,
                                     B00011000};
          for(int i=0;i<=8; i++) 
             { lc.setRow(0,i,Sageata_spate[i]);
               delay(10);
               
             }
             beepBeep();
             lc.clearDisplay(0);
             }  // sfarsit functie Matrix_sageata_spate()
void Matrix_stop(){
              byte stop[8]  ={B00111100,
                              B01100110,
                              B11000011,
                              B10000001,
                              B10000001,
                              B01000011,
                              B01100110,
                              B00111100};
             for(int i=0;i<=8; i++) 
                { lc.setRow(0,i, stop [i]);
                  delay(10);
                  
                }
                beepBeep();
                lc.clearDisplay(0);
                } //sfarsit functie Matrix_stop()
void Matrix_cerc_dreapta(){
             byte Matrix_cerc_dreapta[8] =  {B11100100,
                                             B01100010,
                                             B10100001,
                                             B10000001,
                                             B10000001,
                                             B10000101,
                                             B01000110,
                                             B00110111};
             for(int i=8;i>=0; --i) 
                  {lc.setRow(0,i,Matrix_cerc_dreapta[i]); 
                  delay(10);
                  }
                  beepBeep();
                  lc.clearDisplay(0);
                  } //sfarsit functie Matrix_cerc_dreapta ()
void Matrix_cerc_stanga(){
              byte Matrix_cerc_stanga[8] =    {B00100111,
                                               B01000110,
                                               B10000101,
                                               B10000001,
                                               B10000001,
                                               B10100001,
                                               B01100010,
                                               B11100100};
            for(int i=8;i>=0; --i) 
                 {lc.setRow(0,i,Matrix_cerc_stanga[i]); 
                 delay(10);
                 }
                 beepBeep();
                 lc.clearDisplay(0);
                            } //sfarsit functie Matrix_cerc_stanga()
void Matrix_LineFollower(){
               byte Matrix_LineFollower[8] = {B00001000,
                                              B00010000,
                                              B00010000,
                                              B00001110,
                                              B00000001,
                                              B01100001,
                                              B10010010,
                                              B10001100};
              for(int i=8;i>=0; --i)  
                  {lc.setRow(0,i,Matrix_LineFollower[i]); 
                  delay(10);
                  }
                  beepBeep();
                  lc.clearDisplay(0);
                  } //sfarsit functie Matrix_LineFollower()
void Matrix_ObstacleAvoidance(){
               byte Matrix_ObstacleAvoidance[8] ={B11110011,
                                                  B11110011,
                                                  B10000011,
                                                  B10011111,
                                                  B10011111,
                                                  B10000001,
                                                  B10000001,
                                                  B10011111};
              for(int i=8;i>=0; --i) 
                  {lc.setRow(0,i,Matrix_ObstacleAvoidance[i]); 
                  delay(10);
                  }
                  beepBeep();
                  lc.clearDisplay(0);
                            } //sgarsit functie Matrix_ObstacleAvoidance()
void Matrix_idleNoBeep(){
                byte idle[8] =  {B00011100,      // model cu ?
                                 B00100010,
                                 B00100010,
                                 B00001100,
                                 B00001000,
                                 B00001000,
                                 B00000000,
                                 B00001000};
                    
                                      // {B10101010,      // model cu punctulete
                                      // B01010101,
                                      // B10101010,
                                      // B01010101,
                                      // B10101010,
                                      // B01010101,
                                      // B10101010,
                                      // B01010101};
               for(int i=8;i>=0; --i) 
                    {
                    lc.setRow(0,i, idle [i]);
                    
                    }
                    delay(110);//beepBeep();
                    lc.clearDisplay(0);
                    } //sfarsit functie Matrix_idle()
void Matrix_idle(){
                byte idle[8] =  {B00011100,      // model cu ?
                                 B00100010,
                                 B00100010,
                                 B00001100,
                                 B00001000,
                                 B00001000,
                                 B00000000,
                                 B00001000};
                    
                                      // {B10101010,      // model cu punctulete
                                      // B01010101,
                                      // B10101010,
                                      // B01010101,
                                      // B10101010,
                                      // B01010101,
                                      // B10101010,
                                      // B01010101};
               for(int i=8;i>=0; --i) 
                    {
                    lc.setRow(0,i, idle[i]);
                    delay(10);
                    }
                    beepBeep();
                    lc.clearDisplay(0);
                    } //sfarsit functie Matrix_idle()

void Matrix_model(){
                byte Matrix_model[8] = {B10101010,      // model cu punctulete
                                      B01010101,
                                      B10101010,
                                      B01010101,
                                      B10101010,
                                      B01010101,
                                      B10101010,
                                      B01010101};
               for(int i=8;i>=0; --i) 
                    {
                    lc.setRow(0,i, Matrix_model[i]);
                    delay(10);
                    }
                    beepBeep();
                    lc.clearDisplay(0);
                    } //sfarsit functie Matrix_idle()

void Matrix_exclamare(){
                byte Matrix_exclamare[8] =  {B00011000,
                                             B00011000,
                                             B00011000,
                                             B00011000,
                                             B00011000,
                                             B00000000,
                                             B00011000,
                                             B00011000};
               for(int i=8;i>=0; --i) 
                    {
                    lc.setRow(0,i, Matrix_exclamare[i]);
                    delay(10);
                    }
                    beepBeep();
                    lc.clearDisplay(0);
                    } //sfarsit functie Matrix_idle()

 


void beepBeep(){
              unsigned long Timp = millis(); //variabila Timp stocheaza valoarea in milisecunde a functie millis()
                while(millis()<=Timp+perioada_beep)   //daca daca valoarea actuala a functiei millis e mai micat decat suma initiala + perioada beepului (perioada se seteaza din parametrii) buzzerul e ON
                   {  
                     if(BuzzerOn_OFF==1){
                           digitalWrite(buzzer, LOW);}
                     if (BuzzerOn_OFF==0) {
                           digitalWrite(buzzer, HIGH);}
                    }  
                    digitalWrite(buzzer, HIGH);
               } //sfarsit functie beepbeep