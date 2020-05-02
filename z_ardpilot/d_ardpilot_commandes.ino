void close_and_shutdown() {

  // park front tower
  servo2.write (90);
  delay(100);
  servo1.write (0);
  delay(100);
  
  // idle engines
  ServoLeft.writeMicroseconds (1500);
  delay(100);
  ServoRight.writeMicroseconds (1500);
  delay(100);

  // close socket and disconnect WIFI
  wifi_close ();

  // show ready
  digitalWrite (LED, HIGH);
  delay (1000);
}

bool terminate_current_primitive (char newPrim) {
  
  // pour le moment on interdit de démarrer une nouvelle primitive différente s'il y en a déjà une en cours
  // par contre si c'est la même on laisse passer, on ne fait rien, permet ainsi de changer la consigne en cours de route

  if (newPrim == Primitive) {
    return true;
  } else {
    return false;
  }
  
}

void set_servo (char servo, char sequence, int angle) {
    int b1;

 
    b1 = constrain (angle, 0, 179);
  
    switch (servo) {
      case C_SR1:
          servo1.write (b1);
          break;

      case C_SR2:
          servo2.write (b1);
          break;
    }

    obuffer[0] = servo;
    obuffer[1] = sequence;
    itoa (b1, &(obuffer[2]), 10);
    wifi_write();
    delay(400);
}

void manage_command (byte len) {
  unsigned long t;
  char cmd, v;
  byte i;
  bool res;
  int b1, cap;
  unsigned char newPrimitive, parameter;
  char sequence;
  
  cmd = ibuffer[0];
  sequence = ibuffer[1];
  
  switch (cmd) {    // voir si on peut créer un tableau de fonctions ?
  
  case C_PING:
  case C_CMP:
    
    obuffer[0] = cmd;
    obuffer[1] = '\0';
    get_compas (obuffer, 64); 
  
    wifi_write();
    break;
    
  case C_LED:
  
    if (sequence == 1)
      digitalWrite (LED, LOW);
    else
      digitalWrite (LED, HIGH);
    break;

  case C_DEBUG:
  
    if (ibuffer[1] == 1)
      DebugMode = false;
    else
      DebugMode = true;
    break;


  case C_SR1:
  case C_SR2:
  
    cap = atoi (&(ibuffer[1]));
    set_servo (cmd, sequence, cap);
     
    break;
    
  case C_I2CSCAN:

    I2Cscan();
    break;
  
  case C_PARM:

    parameter = (unsigned char) ibuffer[1]; // get the parameter
    obuffer[0] = C_LOG;
    switch (parameter) {
      
      case F_INIT :
        reload_config_parameters();
      break;
      case F_DEBUG :
        if (!(strcmp (&(ibuffer[2]), "true")))
          DebugMode = true; 
        else
          DebugMode = false;
      break;
 /*      case F_XMMIN :
        xmmin = atoi(&(ibuffer[2])); 
      break;
 
      case F_ZAMAX :
        zamax = atoi(&(ibuffer[2])); 
      break;
*/      case F_SRV1 :
        ServoDelay = atoi(&(ibuffer[2])); 
      break;
      case F_DMIN :
        DistanceMin = atoi(&(ibuffer[2])); 
      break;
      case F_AJUST :
        AjustementMoteur = atoi(&(ibuffer[2])); 
      break;
      case F_ALIGN :
        FacteurAlignement = atof(&(ibuffer[2])); 
      break;
      case F_PNGNUM :
        PngNum = atoi(&(ibuffer[2])); 
      break;
      case F_END :
        info_print ("Initialization complete...\n");  // keep this one as kind of prrof of read
      break;
      default :
//        strcpy (&(obuffer[1]), "Unknown Parameter in config file");
//        wifi_write();
      break;
      }
    
    break;
    
 
  case C_TOPWIFI:
  

    wifi_getTopAP (1);
    
    break;
    
  case C_DISTL:
  
    get_distance (cmd);
    break;

  case C_SCANH:
  case C_SCANV:
  
    scan_distance (cmd, sequence);
    break;
    
  case C_MEM:
  
    memoryFree();
    break;
 
  case C_BAT:
  
    batteryLevel();
    break;

  case C_SHTDN:
  
    close_and_shutdown();
    break;

  case C_TEST:
  
    play_melody (2, 1000);
    break;
    
    
  case C_PRI:

    res = true;
    newPrimitive = (unsigned char) ibuffer[2]; // get the primitive
 
 // debug
 // obuffer[0] = C_LOG;
  // strcpy (&(obuffer[1]), &(ibuffer[2]));
  // wifi_write();
  // delay (200);
 
    if (Primitive != P_NULL) {
      // une primitive est en cours, il faut la terminer
      // renvoiee true s'il a réussi à la terminer, false sinon
      res = terminate_current_primitive (newPrimitive);
    } 
    
    if (res == true)
    {
      if (Primitive != P_NULL) {
        obuffer[0] = C_LOG;
        strcpy (&(obuffer[1]), "Overriding prim ");
        itoa ((unsigned int) Primitive, &(obuffer[19]), 16);
        wifi_write ();
      }
      
      // nouvelle demande, alors que le robot est au repos
      Primitive = newPrimitive; 
      SequencePrimitive = sequence;
      
      // on va juste se préparer à l'exécution (positionner les capteurs réflexe)
      // ensuite la consigne de vitesse sera calculée dans la boucle principale, fonction de 
      //    - mesure capteur réflexe
      //    - la primitive
      //    - la primitive précédente
      //    - la vitesse précédente
      //    - l'atteinte de l'objectif
      
      switch (Primitive) {
      
      case P_SPOT_TURN :
      case P_TURN :
      case P_LARGE_TURN :

        // initialisation de la primitive;
 
        PrimValue = atoi(&(ibuffer[3])); // consigne d'angle
        cap = get_compas (NULL, 0); // angle actuel

        // determine which way to turn - global var so no need to calculate each time
        b1 = dif_angle (PrimValue, cap);

        servo1.write (posRepos1);
        if (b1 < 0)
          servo2.write (0);
        else
          servo2.write (180);
        delay (200); // wait a bit for the servo to be in position, otherwise distance measure can be false. Is 200 enough ?
      break;
      
      case P_TEST_LEFT :
      case P_TEST_RIGHT :
      case P_AVANT :
      case P_AVANT_CAP :
      case P_SUIVI_D :
      case P_SUIVI_G :
     
        if (Primitive == P_AVANT_CAP)
          CapASuivre = get_compas (NULL, 0);
        else
          CapASuivre = -1;
                   
        PrimValue = atoi(&(ibuffer[3])); // consigne de vitesse
        for (i=4; i<64; i++)
        {
          if ((ibuffer[i] == '\0') || (ibuffer[i] == ' '))
            break;
        }
        if ((ibuffer[i] == ' '))
        {
          Decompte = (unsigned long) atoi(&(ibuffer[i])); // consigne de durée (en ms)
          Decompte += millis();
        }
        else
          Decompte = 0xffffffff;
          
        if (Primitive == P_SUIVI_D)
        {
          servo1.write (20);  // a bit in front to detect obstacles (corners)
          servo2.write (170);
          delay (500); // wait a bit for the servo to be in position, otherwise distance measure can be false. Is 200 enough ?
          DistASuivre = measure_distance (3);
        } else if (Primitive == P_SUIVI_G) {
          servo1.write (20);  // a bit in front to detect obstacles (corners)
          servo2.write (10);
          delay (500); // wait a bit for the servo to be in position, otherwise distance measure can be false. Is 200 enough ?
          DistASuivre = measure_distance (3);
        } else {
          servo1.write (90);
          servo2.write (posRepos2);
          DistASuivre = -1;
        }
        
      break;
      }
    } else {
      obuffer[0] = C_LOG;
      strcpy (&(obuffer[1]), "Can't override ");
      itoa ((unsigned int) Primitive, &(obuffer[23]), 16);
      wifi_write ();
      
    }
    
    break;
  }
}
