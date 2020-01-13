

//////////////////////////////////////////
//
// main
//
//////////////////////////////////////////


// include the library code:

#include <Arduino.h>
#include <Wire.h> //I2C Arduino Library
#include <Servo.h>
#include "grammar.h"
#include <U8x8lib.h>

// Wifi Shield
#include <SPI.h>
#include <WiFiNINA.h>
int WifiStatus = WL_IDLE_STATUS;
WiFiClient WiClient;
//String ssid[] = {"TP-LINK_9692C8", "Livebox-94C0"};
String ssid[] = {"Livebox-94C0"};
#include "passwords.h" // just contains : String *pass[] = { "password1", "password2"};
byte CurrentAP = 0;


// Ecran : référence ici : https://github.com/olikraus/u8g2/wiki et https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
U8X8_SSD1306_128X64_NONAME_HW_I2C ecran(U8X8_PIN_NONE);

#define host "192.168.1.3"
#define port 8002


const int Lidar_address = 0x62; 
// #define BNO055_ADDRESS 0x29
// adresse écran 3C

const int LED = 8;

const int TRIG_U = 2;   // capteur ultrason
const int ECHO_U = 5;   // capteur ultrason

// NewPing SonarAV (TRIG_U, ECHO_U);

char ibuffer[64];  // optimisation, ne pas allouer de mémoire tampondans le scan des SSIDs
char obuffer[64];

bool WifiConnected = false;
//bool CallibCompas = false;
bool DebugMode = false;
bool Initialisation = true;

long ipaddr = 0;
/*
int xmmin = 0, xmmax = 0, ymmin = 0, ymmax = 0, zmmin = 0, zmmax = 0;
int xamin = 0, xamax = 0, yamin = 0, yamax = 0, zamin = 0, zamax = 0;
*/
// roues
Servo ServoLeft;      
Servo ServoRight;

// gestion des primitives de déplacement

unsigned char Primitive = P_NULL;
unsigned char OldPrimitive = P_NULL;
int CapASuivre = -1;
long DistASuivre = -1;

char SequencePrimitive;

int PrimValue = 0;
int OldPrimValue = 0;
unsigned long Decompte = 0xffffffff;
  


// gestion des parametres moteurs

int DistanceMin = 5;  // Nb de cm en deça duquel on interrompt une primitive
int AjustementMoteur = 0;
float FacteurAlignement = 1.0;

// parametres de config


// tourelle

int ServoDelay = 0;
int PngNum = 1;

Servo servo1; // dans la tourelle
Servo servo2; // fixe sur le robot
int posRepos1 = 0;  // 0 en bas, 180 en haut
int posRepos2 = 90; // 0 a gauche, 180 a droite

//////////////////////////////////////////
//
// end of variables declaration
//
//////////////////////////////////////////

void info_print (char *message) {
  
   obuffer[0] = C_LOG;
   strcpy (&(obuffer[1]),  message);
   wifi_write();
  
}

void led_blink (int cycles, int don, int doff) {
int i;

  for (i=0; i<cycles; i++) {
      digitalWrite (LED, HIGH);
      delay (don);
      digitalWrite (LED, LOW);
      delay (doff);
  }
}
void send_robot_status () {
    
  delay (400);
  obuffer[0] = C_PING;
  obuffer[1] = '\0';
  wifi_write();

  delay (400);
  memoryFree();
  delay (400);
  batteryLevel();
  delay (400);
  get_angle ('*');
  ecran.print ("status complete\n");
  delay (400);
}

void reload_config_parameters() {

  obuffer[0] = C_PARM;
  obuffer[1] = '\0';
  wifi_write();
  
}

void(* resetFunc) (void) = 0; 
// call this function : resetFunc() to reset the Arduino (usefull if we loose WIFI --> will reconnect everything)
// juste avant tenter un wifly.reboot() ?
// challenge : comment savoir si la socket est cassée ?

void setup() {

 // for debug only
 
  pinMode (LED, OUTPUT);
  
  pinMode (TRIG_U, OUTPUT);
  pinMode (ECHO_U, INPUT);
  digitalWrite (TRIG_U, LOW);

  Wire.begin();
  digitalWrite (LED, HIGH);
  begin_ecran ();
  digitalWrite (LED, LOW);
  ecran.print ("Hello \n");
 
  CurrentAP = wifi_FindBestAP (1); // passer le nb d'entrées dans le tableau des ssid

  ecran.print (ssid[CurrentAP]);
  ecran.print ("\n");

  while (WifiStatus != WL_CONNECTED) {
    WifiStatus = WiFi.begin(ssid[CurrentAP].c_str(), pass[CurrentAP].c_str());
      ecran.print (".");
      delay(4000);
  }
  
  ecran.print ("ok\n");
}

void terminate_setup () {

  
  //Put the  (compass) into the correct operating mode
  initialize_bno055();
  

//  Wire.beginTransmission(Lidar_address); //open communication with Lidar lite V3
//  Wire.write((int)0x00); // sets register pointer to  (0x00)  
//  Wire.write((int)0x04); // sets register pointer to  (0x00)  
//  Wire.endTransmission(); // stop transmitting
  
  ServoLeft.attach (10);
  ServoRight.attach (12);
 
  ServoLeft.writeMicroseconds(1500);
  ServoRight.writeMicroseconds(1500);
  
  servo1.attach(11);
  servo2.attach(13);


  callibrate_dist_sensors ();
  servo1.write (posRepos1);   // vertical vers le bas
  servo2.write (80);          // debug - lui évite de vibrer
  delay (50);
  servo2.write (posRepos2);   // horizontal

  Primitive = P_NULL;
  OldPrimitive = P_NULL;

  PrimValue = 0;
  OldPrimValue = 0;

}


void loop() {
  long dl;
  char cmd;
  byte len;
  static long loop_cnt;


  if (!WifiConnected) {
      ecran.print ("Server.. ");
      if (!WiClient.connect (host, port)) {
        led_blink (2, 100, 100);
        ecran.print ("\n");          
        return;  
      }
      ecran.print ("ok\n");
      
      WifiConnected = true;
       

      terminate_setup ();
      wifi_display_IP ();
            
      send_robot_status ();

      loop_cnt = 0;
      Primitive = P_NULL;
      OldPrimitive = P_NULL;

      reload_config_parameters(); // en dernier de ce bloc d'init
      ecran.clear();
      ecran.print ("Init complete\n");
  } 

  if (WifiConnected) {

    // Check for any data has come to WiFi  
    len = 0;
    digitalWrite (LED, HIGH);
    len = wifi_read (); 
    digitalWrite (LED, LOW);
    if (len > 0) {    
      ecran.print ("len :");
      ecran.print (len);
      ecran.print ("\n");
      manage_command (len);
    } 
      
    // switch primitive interpret sensor values and send adequate values to engines

    //////revoir tout ce qui suit
    if (Primitive != P_NULL)
    {
      dl = measure_distance (1);

      
      switch (Primitive) {
        
      case P_SPOT_TURN :
        primitive_turn (-10, 10, dl, (char *)"Sturn");
      break;
      case P_TURN :
        primitive_turn (0, 15, dl, (char *)"Turn");
      break;
      case P_LARGE_TURN :
        primitive_turn (10, 20, dl, (char *)"Lturn");
      break;
      
      case P_TEST_LEFT :
      case P_TEST_RIGHT :
        primitive_test_engine (Primitive);
      break;

      case P_AVANT :
      case P_AVANT_CAP : 
        primitive_avant (dl);
      break;
          
      case P_SUIVI_D :
      case P_SUIVI_G :
        primitive_suivi (dl);
                 
      break;

      default :
        // robot is getting idle

 
        
      break;
      }
    }

    if ((Primitive == P_NULL) && (OldPrimitive != P_NULL)) {
    // we just finished something, let's send health information
      send_robot_status ();
    }

/*    if (CallibCompas == true) {
      get_compas (true);    
    }
*/    
    loop_cnt++; // in order to implement some functions at certain time only

    if (loop_cnt > 1000000) // !! 1 000 000 corespond à peine à 7 secondes environ !!! (on peut aller jusqu'à 2 milliards ...)
    {
      if (wifi_check_TCP_connection() == true)
      {
        loop_cnt = 0;
//        info_print ("Connection checked OK") ;
      } else
      {
        close_and_shutdown();
        digitalWrite (LED, LOW);
        WiFi.disconnect();
        resetFunc();  
      }
      if (Initialisation == true)
      {
        Initialisation = false;
//        ecran.setPowerSave(1);
      }
    }
    
    OldPrimitive = Primitive;
    OldPrimValue = PrimValue;
    
  } // if connected
}    
