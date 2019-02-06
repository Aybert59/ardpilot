

//////////////////////////////////////////
//
// main
//
//////////////////////////////////////////


//#define DEBUG_ARDPILOT true     // in this case use SoftwareSerial to communucate with Wifly, and let HWserial for debug. 
                                // Nice but interferes with servos. TX jumper to D3, RX jumer to D7

// nothing                      // Normal operation mode. TX jumper to D0, RX jumer to D1
                                // appuyer sur bouton reset de la carte WIFI pendant le televersement !!!

// include the library code:

#include <Wire.h> //I2C Arduino Library
#include <Servo.h>
#include "grammar.h"
#include <SoftwareSerial.h>
#include <NewPing.h>

// Wifi Shield

#include <WiFly.h>
#include <WiFlyClient.h>

#ifdef DEBUG_ARDPILOT
#define DBGPRT(message)    Serial.print(message)
#define DBGPRTLN(message)  Serial.println(message)
#define SERIALBEGIN(toto)  Serial.begin(toto)
#define WIFLY_TX 3
#define WIFLY_RX 7
#define UARTSPEED 115200
#define UART uart
#define DECLARE_UART      SoftwareSerial uart(WIFLY_TX, WIFLY_RX)
#else
#define DBGPRT(message)
#define DBGPRTLN(message)
#define SERIALBEGIN(toto)
#define WIFLY_TX 0
#define WIFLY_RX 1
#define UARTSPEED 9600
#define UART Serial
#define DECLARE_UART
#endif

 // WIFLY reference : http://www.seeedstudio.com/wiki/Wifi_Shield_V2.0
DECLARE_UART; // create a serial connection to the WiFi shield TX and RX pins.
WiFly wifly(&UART); // create a WiFly library object using the serial connection to the WiFi shield we created above.




#define host "192.168.1.3"
const int port = 8002;

const int Compass_address = 0x1E; //0011110b, I2C 7bit address of HMC5883 (compass)
const int Lidar_address = 0x62; 

const int LED = 13;

const int TRIG_U = 2;   // capteur ultrason
const int ENABLE_L = 3; // capteur lidar
const int ECHO_L = 4;   // capteur lidar
const int ECHO_U = 5;   // capteur ultrason

NewPing SonarAV (TRIG_U, ECHO_U);

char ibuffer[64];
char obuffer[64];
bool WifiConnected = false;
long ipaddr = 0;

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

const long DIST_SEUIL = 1500; 
int DistanceMin = 5;  // Nb de cm en deça duquel on interrompt une primitive
int AjustementMoteur = 0;
float FacteurAlignement = 1.0;

// parametres de config

char listAP[5][20];

const char *ssid[] = {"TP-LINK_9692C8", "Livebox-94C0"};
const char *pass[] = { "mntifwrd5", "Mntifwrd5"};
byte CurrentAP;


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
    
  obuffer[0] = C_PING;
  obuffer[1] = '\0';
  wifi_write();

  delay (200);
  memoryFree(0x00);
  delay (200);
  batteryLevel();
  delay (200);
  get_angle ('*');
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
  
  pinMode (LED, OUTPUT);
  
  pinMode (TRIG_U, OUTPUT);
  pinMode (ENABLE_L, OUTPUT);
  pinMode (ECHO_L, INPUT);
  pinMode (ECHO_U, INPUT);
  digitalWrite (TRIG_U, LOW);
  digitalWrite (ENABLE_L, HIGH);

  UART.begin(9600); // start the serial connection to the shield

  // for debug only
  SERIALBEGIN(UARTSPEED);  // start the Arduino serial monitor window connection
  DBGPRTLN(""); // start a new line

  delay(3000); // wait 3 second to allow the serial/uart object to start

  wifly.reset(); // reset the shield 

  DBGPRT("Going to join Access Point... ");

  CurrentAP = wifi_FindBestAP (2); // passer le nb d'entrées dans le tableau des ssid
  while (!wifly.join(ssid[CurrentAP], pass[CurrentAP],WIFLY_AUTH_WPA2_PSK)) {
      delay (100);
  }
  DBGPRTLN("Done.");
  
}

void terminate_setup () {

  
  //Put the HMC5883 IC (compass) into the correct operating mode
  Wire.begin();
  
  Wire.beginTransmission(Compass_address); //open communication with HMC5883 compass
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

//  delay(20);

//  Wire.beginTransmission(Lidar_address); //open communication with Lidar lite V3
//  Wire.write((int)0x00); // sets register pointer to  (0x00)  
//  Wire.write((int)0x04); // sets register pointer to  (0x00)  
//  Wire.endTransmission(); // stop transmitting
  
  ServoLeft.attach (10);
  ServoRight.attach (12);
 
  ServoLeft.writeMicroseconds(1500);
  ServoRight.writeMicroseconds(1500);
  
  servo1.attach(11);
  servo2.attach(9);


  callibrate_dist_sensors ();
  servo1.write (posRepos1);   // vertical vers le bas
  servo2.write (80);          // debug - lui évite de vibrer
  delay (50);
  servo2.write (posRepos2);   // horizontal

  Primitive = P_NULL;
  OldPrimitive = P_NULL;

  PrimValue = 0;
  OldPrimValue = 0;

  // initialisation des parametres

  listAP[0][0] = '-';
  listAP[1][0] = '-';
  listAP[2][0] = '-';
  listAP[3][0] = '-';
  listAP[4][0] = '-';


}


void loop() {
  long dl;
  char cmd;
  int len;
  static long loop_cnt;


  if (!WifiConnected) {
      DBGPRT("Going to connect throught TCP... ");
      if (!wifly.connect (host, port)) {
        led_blink (2, 100, 100);          
        return;  
      }
      
      DBGPRTLN("Succeed.");
      WifiConnected = true;
      obuffer[0] = 'a'; //dummy
      obuffer[1] = '\0';
      wifi_write(); // vider le "*HELLO*" qui traine
         

      terminate_setup ();
      wifi_display_IP ();
            
      send_robot_status ();

      loop_cnt = 0;
      Primitive = P_NULL;
      OldPrimitive = P_NULL;

//wifly.sendCommand("get everything\r");          // à placer sur un bouton Diag

      reload_config_parameters(); // en dernier de ce bloc d'init
  } 

  if (WifiConnected) {

    // Check for any data has come to Wifly  
    len = 0;
    len = wifi_read (); 
    if (len > 0) {
      DBGPRT("Received ");
      DBGPRT(len);
      DBGPRT(" Bytes : ");
      DBGPRTLN(".");
      
      manage_command (len);
    } 
      
    // switch primitive interpret sensor values and send adequate values to engines

    //////revoir tout ce qui suit
    if (Primitive != P_NULL)
    {
      dl = measure_distance (3);

      
      switch (Primitive) {
        
      case P_SPOT_TURN :
        primitive_turn (-15, 15, dl, (char *)"Sturn");
      break;
      case P_TURN :
        primitive_turn (0, 20, dl, (char *)"Turn");
      break;
      case P_LARGE_TURN :
        primitive_turn (15, 30, dl, (char *)"Lturn");
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
        wifly.reboot();
        resetFunc();  
      }
    }
    OldPrimitive = Primitive;
    OldPrimValue = PrimValue;

// ---- below : debug only - comment otherwise
// debug : permet de visualiser la durée d'une boucle

//    if ((loop_cnt & 1) == 0) {
//      digitalWrite (LED, HIGH);
//    } else {
//      digitalWrite (LED, LOW);
//    }
    
  } // if connected
}    

