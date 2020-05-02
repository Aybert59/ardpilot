// grammar for control sockets

// subjects

// message level device

#define MSG_INFO    '1'
#define MSG_WARNING '2'
#define MSG_ERROR   '3'
#define MSG_DEBUG   '4'

// actions for Arduino

#define ACTION_NULL  0x00



// LED management
#define LED_STATE    0x01

// controls
#define C_PING        0x02
#define C_STATUS      0x03
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
#define C_CALCMP      0x19  // compass callibration
#define C_I2CSCAN     0x1a 
#define C_DEBUG       0x1b
#define C_TEST        0x1c
#define C_TONE        0x1d


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

#define F_DEBUG       0x01
#define F_LEARN       0x02

#define F_SRV1        0x10
#define F_PNGNUM      0x11
#define F_COVSEUIL    0x12
#define F_LINEW       0x13
#define F_PATHW       0x14
#define F_PATHD       0x15

#define F_DMIN        0x20
#define F_AJUST       0x21
#define F_ALIGN       0x22

#define F_XMMIN       0x30
#define F_XMMAX       0x31
#define F_YMMIN       0x32
#define F_YMMAX       0x33
#define F_ZMMIN       0x34
#define F_ZMMAX       0x35

#define F_XAMIN       0x36
#define F_XAMAX       0x37  
#define F_YAMIN       0x38
#define F_YAMAX       0x39
#define F_ZAMIN       0x3a
#define F_ZAMAX       0x3b

#define F_LOC_SPREAD  0x40
#define F_LOC_PCENT   0x41

#define F_END         0xfe


/*************************************************
 * Tones
 *************************************************/

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

// end of grammar
