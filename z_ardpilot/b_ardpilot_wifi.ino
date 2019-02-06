
// lit ce qui arrive, le place dans ibuffer et retourne le nb d'octets lus
//
// cf le cpp
// par defaut le device est en data mode 
// boolean sendCommand(const char *cmd, const char *ack = NULL, int timeout = DEFAULT_WAIT_RESPONSE_TIME); pour envoyer une commande
// 
// a embedder dans des fonctions plus haut niveau
//
// sinon !! wifly est une sous classe de stream --> utiliser les fonctions héritées !

void wifi_clear_input()
{
  while (wifly.available()) {
    wifly.read();
  }
}

/* TUTO comment passer une commande au module WIFI et afficher son résultat dans la console
 *  
  obuffer[0] = C_LOG;
  wifly.sendCommand("show connection\r");
  strcpy (&(obuffer[1]), "Show connection : ");
  for (i=19; i<64;i++)
    obuffer[i] = '\0';
  wifly.readBytes(&(obuffer[19]), 32);

  wifly.dataMode();
  wifi_clear_input();
  
  wifi_write();
  */

bool wifi_check_TCP_connection ()
{

 /*
  * cette fonction merde
  * le retour de "show connection" est imprévisible ?
  * ou tout au moins non maitrisé
  */
  if (wifly.available() > 0)
  {
    // quelque chose est en attente en input : on ne perturbe pas
    // et ça prouve en plus que la connection est bonne :-)

    return (true);
    
  }

  wifly.sendCommand("show connection\r");
  wifly.flush();
  while (wifly.available() == 0);

// 1e ligne vide (juste '\r' : longueur lue = 0
// 3e ligne code par exemple \n8531 quand connection OK
// 4e ligne version software "<4.41>"

//  wifly.readBytesUntil ('\r', obuffer, 62); // 1e ligne vide
//  wifly.readBytesUntil ('\r', obuffer, 62); // 2e ligne vide
//  wifly.readBytesUntil ('\r', obuffer, 62);

  wifly.readBytesUntil ('\r', obuffer, 62);
  while (obuffer[1] != '8')
    wifly.readBytesUntil ('\r', obuffer, 62);
    
  wifly.dataMode();
  wifi_clear_input();

//obuffer[0] = C_LOG;
//obuffer[5] = '\0';
//wifi_write();

  if (obuffer[4] == '1')
    return (true);
  else
    return (false);  
}
  
void wifi_display_IP () // and complete some setup
{
  long i;

  obuffer[0] = C_LOG;
  strcpy (&(obuffer[1]), "Connected to ");
  strcpy (&(obuffer[14]), ssid[CurrentAP]);
  wifi_write();
  delay (1000);
  
  wifly.sendCommand("set ip flags 0x6\r","AOK");  // close TCP connection upon loss of AP pairing 
  // (by default allows 60 seconds to associate again : useful when unique SSID across various AP

// test
  wifi_check_TCP_connection();
  
  wifly.sendCommand("set sys printlvl 0x4000\r","AOK");
  wifly.sendCommand("get ip\r");
  wifly.find((char *)"192.168.1.");
  i = wifly.parseInt();
  
  DBGPRT("IP - 192.168.1.");
  DBGPRTLN(i);

  
  wifly.dataMode();
  wifi_clear_input();
  
  obuffer[0] = C_LOG;
  strcpy (&(obuffer[1]), "IP - 192.168.1.");
  itoa (i,&(obuffer[16]),10);
  wifi_write();
  
}

int wifi_read ()
{
  int len = 0;
  
  if (wifly.available()) {
    len = wifly.readBytesUntil('\0', ibuffer, 64);
    ibuffer[len] = '\0';
  }
  
  return len;
}

void wifi_write ()
{
  unsigned char len;

  len = (unsigned char) strlen (obuffer);

  wifly.send(&len, 1, DEFAULT_WAIT_RESPONSE_TIME);
  wifly.send(obuffer, DEFAULT_WAIT_RESPONSE_TIME);
  wifly.flush();
  delay(20);  // timeout by default of the RN-171 is 10ms, before it actually sends the TCP packet. a bit more is safer (15 gives yet some hickups)

}

void wifi_close () {
  
  //  ("AT+CIPCLOSE"); --> se déconnecter du serveur (fermer la socket)
  wifly.leave();
  
}


void wifi_getAccessPoint (char sequence)
{
  // renvoyer la sequence au serveur pour qu'il s'y retrouve
  byte i, j;
  bool found;
  char resultat[24];
  

//wifly.sendCommand("scan 1\r", "END");           // trouvé sur le web : do one 1ms scan, because in APmode the first scan gives always an empty result 


  wifly.sendCommand("scan\r");
  while (wifly.available() == 0);
  strcpy(resultat, "xx -99 -99 -99 -99 -99");

// 1e ligne vide (juste '\r' : longueur lue = 0
// 2e ligne "scan"
// 4e ligne version software "<4.41>"
// 7e ligne "SCAN:Found 10" (par exemple)
// 8e ligne data "01,01,-60,04,1115,1c,40,48:83:c7:39:94:c0,Livebox-94C0" par exemple
// dernière ligne doit être "END:" selon la doc

  wifly.readBytesUntil ('\r', obuffer, 62);
  
  while (!strstr (obuffer, "END:") )
  {
    for (i = 0; i < 5; i++)
    {
      if ((listAP[i][0] != '-') && (strstr (obuffer, listAP[i]))) 
      {
        // la puissance du signal est le 3e champ, entier négatif sur 2 caractères (cf exemple ci-dessus)
        j = (4*(i+1));
        resultat[j] = obuffer[7];
        resultat[j+1] = obuffer[8];
      }
    }
    wifly.readBytesUntil ('\n', obuffer, 62);
  }
   
  wifi_clear_input();
  wifly.dataMode();

  strcpy (obuffer, resultat);
  obuffer[0] = C_LAP;
  obuffer[1] = sequence;
  wifi_write();
  
}

void wifi_getTopAP (char sequence)
{
  byte i, j;
  char resultat[10][24];
  char pd, pu; // dizaines, unites
 
  wifly.sendCommand("scan\r");
  while (wifly.available() == 0);
 
  for (i = 0; i < 10; i++)
  {
      resultat[i][0] = '9';
      resultat[i][1] = '9';
      resultat[i][2] = ' ';
      resultat[i][3] = '\0';
  }
 
  j = wifly.readBytesUntil ('\n', obuffer, 62);
  obuffer[j] = '\0';
  obuffer[63] = '\0'; // securite
 
  while (!strstr (obuffer, "END:") )
  {
    pd = obuffer[7];
    pu = obuffer[8];   
    
    if ((pd >= '0') && (pd <= '9'))
    {
      for (i = 0; i < 10; i++)
      {
        if ((pd < resultat[i][0]) || ((pd == resultat[i][0]) && (pu < resultat[i][1])))
        {
          for (j = 9; j > i; j--)
          {
            if (j>0)
              strcpy (resultat[j],resultat[j-1]);
          }
          resultat[i][0] = pd;
          resultat[i][1] = pu;
          strcpy (&(resultat[i][3]), &(obuffer[42]));
          break;
        }
      }
    }
    
    j= wifly.readBytesUntil ('\n', obuffer, 62);
    obuffer[j] = '\0';
    obuffer[63] = '\0'; // securite
  }
  
  wifi_clear_input();
  wifly.dataMode();
 
  for (i = 0; i < 10; i++)
  {
      strcpy (&(obuffer[2]), resultat[i]);
      obuffer[0] = C_TOPWIFI;
      obuffer[1] = sequence;
      wifi_write();
      delay(120);
  }
}
void wifi_getTopAPold (char sequence)
{
  // renvoyer la sequence au serveur pour qu'il s'y retrouve
  byte i, j;
  bool found;
  char resultat[10][24];
  char pd, pu; // dizaines, unites

//wifly.sendCommand("scan 1\r", "END");           // trouvé sur le web : do one 1ms scan, because in APmode the first scan gives always an empty result 


  wifly.sendCommand("scan\r");
  while (wifly.available() == 0);
  
  for (i = 0; i < 10; i++)
  {
      resultat[i][0] = '9';
      resultat[i][1] = '9';
      for (j=2; j<24; j++)
        resultat[i][j] = '\0';
  }
  
// 1e ligne vide (juste '\r' : longueur lue = 0
// 2e ligne "scan"
// 4e ligne version software "<4.41>"
// 7e ligne "SCAN:Found 10" (par exemple)
// 8e ligne data "01,01,-60,04,1115,1c,40,48:83:c7:39:94:c0,Livebox-94C0" par exemple
// dernière ligne doit être "END:" selon la doc

  for (j = 0; j < 64; j++)
    obuffer[j]='\0';
  wifly.readBytesUntil ('\r', obuffer, 62);
  
  while (!strstr (obuffer, "END:") )
  {
    pd = obuffer[7];
    pu = obuffer[8];    
    
    for (i = 0; i < 10; i++)
    {
       if (pd < resultat[i][0])
       {
         resultat[i][0] = pd;
         resultat[i][1] = pu;
         strncpy (&(resultat[i][2]), &(obuffer[42]), 20);
         break;
       }
       else if ((pd == resultat[i][0]) && (pu < resultat[i][1]))
       {
         resultat[i][0] = pd;
         resultat[i][1] = pu;
         strncpy (&(resultat[i][2]), &(obuffer[42]), 20);
         break;
       }
    }
    
    for (j = 0; j < 64; j++)
      obuffer[j]='\0';
    wifly.readBytesUntil ('\n', obuffer, 62);
  }
   
  wifi_clear_input();
  wifly.dataMode();

 
  for (i = 0; i < 10; i++)
  {
      strcpy (&(obuffer[2]), resultat[i]);
      obuffer[0] = C_TOPWIFI;
      obuffer[1] = sequence;
      wifi_write();
  }
    
}

int wifi_FindBestAP (byte numAP)
{
  // renvoyer la sequence au serveur pour qu'il s'y retrouve
  byte i, j;
  byte minVal, minAP;
  

  wifly.sendCommand("scan 1\r", "END");           // trouvé sur le web : do one 1ms scan, because in APmode the first scan gives always an empty result 

  minVal = 99;
  minAP = -1;
  
  wifly.sendCommand("scan\r");
  while (wifly.available() == 0);

// 1e ligne vide (juste '\r' : longueur lue = 0
// 2e ligne "scan"
// 4e ligne version software "<4.41>"
// 7e ligne "SCAN:Found 10" (par exemple)
// 8e ligne data "01,01,-60,04,1115,1c,40,48:83:c7:39:94:c0,Livebox-94C0" par exemple
// dernière ligne doit être "END:" selon la doc

  wifly.readBytesUntil ('\r', obuffer, 62);
  
  while (!strstr (obuffer, "END:") )
  {
    for (i = 0; i < numAP; i++)
    {
      if (strstr (obuffer, ssid[i])) 
      {
        // la puissance du signal est le 3e champ, entier négatif sur 2 caractères (cf exemple ci-dessus)

        j = (obuffer[7] - 48) * 10;
        j += obuffer[8] - 48;

        if (j < minVal)
        {
          minVal = j;
          minAP = i;
        }
      }
    }
    wifly.readBytesUntil ('\n', obuffer, 62);
  }
   
  wifi_clear_input();
  wifly.dataMode();

  return (minAP);
  
}
