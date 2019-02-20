// grammar for control sockets

// subjects

// message level device

#define MSG_INFO    1
#define MSG_WARNING 2
#define MSG_ERROR   3

// actions for Arduino

#define ACTION_NULL  0x00




// LED management
#define LED_STATE    0x01

// controls
#define C_PING        0x02

#define C_LED         0x04
#define C_SR1         0x05
#define C_SR2         0x06
#define C_CMP         0x07
#define C_DISTL       0x08
#define C_DISTF       0x08 // only one in Front, previously the Left one
#define C_DISTR       0x09 // unused today
#define C_PARM        0x0a
#define C_SCANH       0x0b // mesure d'obstacles horizontale (capteur ultrasons)
#define C_SCANV       0x0c // mesure d'obstacles verticale (capteur ultrasons)
#define DO_NOT_USE    0x0d
#define C_MTR         0x10
#define C_MEM         0x11
#define C_LOG         0x12
#define C_BAT         0x13
#define C_PRI         0x14 // primitive de mouvement
#define C_IDLE        0x15
#define C_SHTDN       0x16
#define C_TOPWIFI     0x17 // top 10 des SSID WIFI
#define C_LIDAR       0x18


#define C_COLOR       0xfe // not a control. Pass directly a color change for an object in the web interface

// separating commands

#define ACTION_END    0xff

// engine primitives

#define P_NULL        0xff

#define P_SPOT_TURN   'S'
#define P_TURN        'T'
#define P_LARGE_TURN  'L'

#define P_TEST_LEFT   'G'
#define P_TEST_RIGHT  'D'

#define P_AVANT       'A'
#define P_AVANT_CAP   'C'
#define P_SUIVI_D     '>'
#define P_SUIVI_G     '<'

// config parameters

#define F_INIT        0xff
#define F_AP1         0x01
#define F_AP2         0x02
#define F_AP3         0x03
#define F_AP4         0x04
#define F_AP5         0x05

#define F_SRV1        0x10
#define F_PNGNUM      0x11
#define F_COVSEUIL    0x12
#define F_LINEW       0x13
#define F_PATHW       0x14
#define F_PATHD       0x15

#define F_DMIN        0x20
#define F_AJUST       0x21
#define F_ALIGN       0x22

// end of grammar
