

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

// Wifi 
//String ssid[] = {"TP-LINK_9692C8", "Livebox-94C0"};
String ssid[] = {"Livebox-94C0"};
#include "passwords.h" // just contains : String *pass[] = { "password1", "password2"};
byte CurrentAP = 0;


U8X8_SSD1306_128X64_NONAME_HW_I2C ecran(U8X8_PIN_NONE);

#define host "192.168.1.3"
#define port 8002


const int Lidar_address = 0x62; 
// #define BNO055_ADDRESS 0x29
// adresse écran 3C

const int LED = 8;
const int WIFI_RST = 3;
const int WIFI_ON = 4;
const int TRIG_U = 2;   // capteur ultrason
const int ECHO_U = 5;   // capteur ultrason
const int BUZZER = 6;


char ibuffer[64];  // optimisation, ne pas allouer de mémoire tampondans le scan des SSIDs
char obuffer[64];

bool WifiConnected = false;
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

void info_print (char *message)
{
  
   obuffer[0] = C_LOG;
   strcpy (&(obuffer[1]),  message);
   wifi_write();
  
}

void led_blink (int cycles, int don, int doff)
{
  int i;

  for (i=0; i<cycles; i++) {
      digitalWrite (LED, HIGH);
      delay (don);
      digitalWrite (LED, LOW);
      delay (doff);
  }
}

/*void send_robot_status ()
{
    
  delay (400);
  memoryFree();
  delay (400);
  batteryLevel();
  delay (400);
  get_angle ();
  ecran.print ("status complete\n");
  delay (400);
}
*/
void send_bin_status ()
{
  unsigned char len = 1;
  
  obuffer[0] = C_STATUS;
  
  len += bin_get_compas (&(obuffer[len]));
  len += bin_get_memory (&(obuffer[len]));
  len += bin_get_battery (&(obuffer[len]));
  len += bin_get_calibration_state (&(obuffer[len]));

  wifi_write_binary (len);
}

void reload_config_parameters()
{

  obuffer[0] = C_PARM;
  obuffer[1] = '\0';
  wifi_write();
  
}

void(* resetFunc) (void) = 0; 
// call this function : resetFunc() to reset the Arduino (usefull if we loose WIFI --> will reconnect everything)


void setup() {

  pinMode (LED, OUTPUT);
  pinMode (WIFI_RST, OUTPUT);
  digitalWrite (WIFI_RST, LOW); // disable chip

  pinMode (WIFI_ON, OUTPUT);
  digitalWrite (WIFI_ON, LOW); // disable power
  
  pinMode (TRIG_U, OUTPUT);
  pinMode (ECHO_U, INPUT);
  digitalWrite (TRIG_U, LOW);

  delay (3000);

  Wire.begin();
  Wire.setClock(100000);
  
  begin_ecran ();
  ecran.print ("Hello \n");

  wifi_start();
 
  CurrentAP = wifi_FindBestAP (1); // passer le nb d'entrées dans le tableau des ssid

  ecran.print (ssid[CurrentAP]);
  ecran.print (" ");

//  while (! wifi_begin (ssid[CurrentAP].c_str(), pass[CurrentAP].c_str()));
  wifi_begin (ssid[CurrentAP].c_str(), pass[CurrentAP].c_str());
  
  ecran.print ("ok\n");
}

void terminate_setup () {

  
  //Put the  (compass) into the correct operating mode
  initialize_bno055();

  initialize_lidar();
   
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
  tone (BUZZER, 500, 20);
}


void loop() {
  long dl;
  char cmd;
  byte len;
  static long loop_cnt;

  
  
  if (!WifiConnected) {
      ecran.print ("Server.. ");
      WifiConnected = wifi_connect (host, port);
      if (! WifiConnected) {
        led_blink (2, 100, 100);
        ecran.print ("\n");          
        return;  
      }
      ecran.print ("ok\n");   

      terminate_setup ();
      wifi_display_IP ();
            
      send_bin_status ();

      loop_cnt = 0;
      Primitive = P_NULL;
      OldPrimitive = P_NULL;

      reload_config_parameters(); // en dernier de ce bloc d'init
      ecran.clear();
      ecran.println ("Init complete");
  } 

  if (WifiConnected) {

    // Check for any data has come to WiFi  
    len = 0;
    len = wifi_read (); 
    if (len > 0) {    
      manage_command (len);
    } 
      
    // switch primitive interpret sensor values and send adequate values to engines

    //////revoir tout ce qui suit
    if (Primitive != P_NULL)
    {
      dl = measure_distance_stable ();

      
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

 //   if ((Primitive == P_NULL) && (OldPrimitive != P_NULL)) {
    // we just finished something, let's send health information
 //     send_robot_status ();
 //   }

/*    if (CallibCompas == true) {
      get_compas (true);    
    }
*/    
    loop_cnt++; // in order to implement some functions at certain time only

    if ((loop_cnt % 10000) == 0) // each 0.7 seconds then... More often is dangerous
     {
        send_bin_status ();
     }
      
    if (loop_cnt > 1000000) // !! 1 000 000 coresponds to about 7 seconds  !!! (can go up to 2 billion ...)
    {
      if (wifi_check_TCP_connection() == true)
      {
        loop_cnt = 0;
      } else
      {
        close_and_shutdown();
        digitalWrite (LED, LOW);
//        WiFi.disconnect();
        resetFunc();  
      }

      
      if (Initialisation == true)
      {
        Initialisation = false;
        ecran.setPowerSave(1);
      }
    }


    
    
    OldPrimitive = Primitive;
    OldPrimValue = PrimValue;
    
  } // if connected
}    
