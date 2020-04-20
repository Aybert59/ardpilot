

/*
 * command reference https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+RST
 * Exemples ici https://github.com/espressif/ESP8266_AT/wiki/at_example_0020000903
 * 
 */
 
void wifi_clear_input()
{
 while (Serial.available()) {
    Serial.read();
  }
}


bool wifi_check_TCP_connection ()
{
   return (true);
//   return WiClient.connected();
}

bool wifi_connect (char *address, int portnum) {
  Serial.print ("AT+CIPSTART=\"TCP\",\"");
  Serial.print (address);
  Serial.print ("\",");
  Serial.println (portnum);
  return (Serial.find ("CONNECT"));
}

void wifi_display_IP () // and complete some setup
{
//  IPAddress ip;
  String LocalIP;

 return;
/*  
 *   
 obuffer[0] = C_LOG;
  strcpy (&(obuffer[1]), "Connected to ");
  strcpy (&(obuffer[14]), ssid[CurrentAP].c_str());
  wifi_write();
  delay (1000);
  
  ip = WiFi.localIP();
  
  obuffer[0] = C_LOG;

  LocalIP = String() + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3];
  strcpy (&(obuffer[1]), LocalIP.c_str());
  wifi_write();
  */
}

byte wifi_read ()
{
  int len = 0;
  char longueur[8];
  int i;
  
  if (Serial.available())
  {
     if (Serial.find ("IPD,"))
    {
      for (i=0; i<8; i++)
        longueur[i] = '\0';
        
      Serial.readBytesUntil(':',longueur, 8);
    
      len = atoi(longueur);
      Serial.readBytes(ibuffer, len);
      ibuffer[len] = '\0';
    }
  }
  return len;
}

int wifi_write ()
{
  unsigned char len;

  len = (unsigned char) strlen (obuffer);
  wifi_write_binary (len);
}

void wifi_write_binary (unsigned char len)
{
  Serial.print ("AT+CIPSEND=");
  Serial.println (len+1);
  Serial.flush();
  Serial.find (">");
  Serial.write(len);
  Serial.write((const uint8_t*)obuffer, len);
  Serial.flush();
  Serial.find ("SEND OK");
 
}

void wifi_close () {

  Serial.println ("AT+CWQAP");
  digitalWrite (WIFI_RST, LOW); // disable
}

int wifi_start () {
  int ret=0;
  bool found = false;
  
  ecran.println("Start WIFI");
  
  Serial.begin (115200);
  Serial.setTimeout (2000);
  
  digitalWrite (WIFI_ON, HIGH); // enable power
  delay (500);

  digitalWrite (WIFI_RST, HIGH); // enable chip @
  delay (500);

  while (Serial.available())
    Serial.read();
    

  Serial.print ("AT+CWMODE=3\r\n");     // station mode
  if (Serial.find ("OK"))
  {
    Serial.println ("AT+RST");          // take CWMODE into account
    if (Serial.find ("OK"))
    {
      ecran.println("resetting");
      if (Serial.find ("ready"))
      {
        Serial.println ("AT+CWDHCP=1,1");   // Start DHCP client
        if (Serial.find ("OK"))
        {
          ecran.println("WIFI init'd");
          ret = 1;
        } else {
          ecran.println("CWDHCP fails");
        }
      } else {
        ecran.println("ready fails");
      }
    } else {
      ecran.println("RST fails");
    }
  } else {
    ecran.println("CWMODE fails");
  }

  return (ret);
}

bool wifi_begin (char *ssid, char *pass) {
  Serial.print ("AT+CWJAP=\"");
  Serial.print (ssid);
  Serial.print ("\",\"");
  Serial.print (pass);
  Serial.println ("\"");
  return (Serial.find ("OK"));
}

void wifi_getTopAP (int rst)
{
  byte i, j;
  int count = 0;
  int c,d;
  int Force;
  char resultat[10][32];
  int rssi[10];
  char SSIDname[32];
  int len;
  
  for (i = 0; i < 10; i++)
  {
      resultat[i][0] = '9';
      resultat[i][1] = '9';
      resultat[i][2] = ' ';
      resultat[i][3] = '\0';
      rssi[i] = 99;
  }

  ecran.println ("get top AP");  
i=0;

  wifi_clear_input();
  Serial.println ("AT+CWLAP");

  // exemple : +CWLAP:(4,"SFR-a0c8",-79,"08:3e:5d:f4:a0:ce",1)
  Serial.setTimeout (5000); 
  while (Serial.find ("+CWLAP:("))
  {
    len=Serial.readBytesUntil(')', obuffer, 63);
    obuffer[len+1] = '\0';


    if (len > 8)
    {
      count++;
      
      c=3;
      d=4;
      while (obuffer[d] != '"')
        d++;

      obuffer[d] = '\0';
      strcpy (SSIDname, &(obuffer[c]));


      c=d+3; // first RSSI digit
      d=c;
      while (obuffer[d] != ',')
        d++;
      obuffer[d] = '\0';
      
      Force = abs (atoi(&(obuffer[c])));
             
             
      //debug
      /*
      if ((count == 4) or (count == 9) or (count == 13))
      {
        ecran.print (SSIDname); 
        ecran.print (" "); 
        ecran.println (Force); 
      }
 */
      for (i = 0; i < 10; i++)
      {
        if ((Force > 0) && (Force < rssi[i]))
        {
          for (j = 9; j > i; j--)
          {
            if (j>0)
            {
              strcpy (resultat[j],resultat[j-1]);
              rssi[j] = rssi[j-1];
            }
          }
          resultat[i][0] = (char) (48 + (Force / 10));    // 48 is ascii code for '0'
          resultat[i][1] = (char) (48 + (Force % 10));    
          strcpy (&(resultat[i][3]), SSIDname);
          rssi[i]=Force;
          break;
        }
      
      }
         
    }
  }
  
  ecran.print ("found ");
  ecran.print (count);
  ecran.println (" SSID");
  Serial.setTimeout (2000);
  
  obuffer[0] = C_TOPWIFI;
  obuffer[1] = '*';
  for (i = 0; i < 10; i++)
  {
      strcpy (&(obuffer[2]), resultat[i]);
      j = wifi_write();                               
  }

}

  
byte wifi_FindBestAP (byte numAP)
{
  int numSsid;
  int thisNet;
  byte minAP, i; 
  long minVal, j;
  char *token;
  int len;

return (0);

  minVal = -99;
  minAP = -1;

  ecran.println ("scanning WIFI");
  Serial.println ("AT+CWLAP");

  // exemple : +CWLAP:(4,"SFR-a0c8",-79,"08:3e:5d:f4:a0:ce",1)

  while (Serial.available())
  {
    len = Serial.readBytesUntil('\n', obuffer, 64);
    obuffer[len] = '\0';

    if (len > 8)
    {
      for (i = 0; i < numAP; i++)
      {
        if (strstr (obuffer, ssid[i].c_str()))
        {
          token = strtok(obuffer, ",");   // +CWLAP
          token = strtok(NULL, ",");  // SSID name
          token = strtok(NULL, ",");  // Signal Strengh
        
          j = atoi(token);
          if (j > minVal)
          {
            minVal = j;
            minAP = i;
          }
        }
      }
    }
  }

  return (minAP);
}
