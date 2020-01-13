
void wifi_clear_input()
{
  while (WiClient.available()) {
    WiClient.read();
  }
}


bool wifi_check_TCP_connection ()
{
  return WiClient.connected();
}
  
void wifi_display_IP () // and complete some setup
{
  IPAddress ip;
  String LocalIP;

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
  
}

byte wifi_read ()
{
  byte len = 0;
  
  if (WiClient.available()) {
    len = WiClient.readBytesUntil('\0', ibuffer, 64);
    ibuffer[len] = '\0';
  }
  
  return len;
}

// when buffer size is unknown
// write until it meets '\0'
// not suitable for raw data which may contain zeros
// see wifi_write_binary() in that case

void wifi_write ()
{
  unsigned char len;

  len = (unsigned char) strlen (obuffer);

  WiClient.write(&len, 1);
  WiClient.write((const uint8_t*)obuffer, len);
  delay(20);  // timeout by default of the RN-171 is 10ms, before it actually sends the TCP packet. a bit more is safer (15 gives yet some hickups)

}

void wifi_write_binary (unsigned char len)
{
  WiClient.write(&len, 1);
  WiClient.write((const uint8_t*)obuffer, len);
  delay(20);  // timeout by default of the RN-171 is 10ms, before it actually sends the TCP packet. a bit more is safer (15 gives yet some hickups)

}

void wifi_close () {
  
  WiClient.stop();
  
}


void wifi_getTopAP (char sequence)
{
  byte i, j;
  int numSsid;
  char resultat[10][24];
  int rssi[10];

  numSsid = WiFi.scanNetworks();
 
  for (i = 0; i < 10; i++)
  {
      resultat[i][0] = '9';
      resultat[i][1] = '9';
      resultat[i][2] = ' ';
      resultat[i][3] = '\0';
      rssi[i] = 99;
  }
 
  for (int thisNet = 0; thisNet < numSsid; thisNet++)
  {  
    for (i = 0; i < 10; i++)
      {
        if (WiFi.RSSI(thisNet) < 99)
        {
          for (j = 9; j > i; j--)
          {
            if (j>0)
            {
              strcpy (resultat[j],resultat[j-1]);
              rssi[j] = rssi[j-1];
            }
          }
          resultat[i][0] = (char) WiFi.RSSI(thisNet) / 10;
          resultat[i][1] = (char) WiFi.RSSI(thisNet) % 10;
          strcpy (&(resultat[i][3]), WiFi.SSID(thisNet));
          rssi[i]=WiFi.RSSI(thisNet);
          break;
        }
    }
  }
  
    for (i = 0; i < 10; i++)
  {
      strcpy (&(obuffer[2]), resultat[i]);
      obuffer[0] = C_TOPWIFI;
      obuffer[1] = sequence;
      wifi_write();
      delay(120);
  }
}




byte wifi_FindBestAP (byte numAP)
{
  int numSsid;
  int thisNet;
  byte minAP, i; 
  long minVal, j;

  minVal = -99;
  minAP = -1;

  ecran.print ("scanning WIFI\n");
  
  numSsid = WiFi.scanNetworks();
 
  for (thisNet = 0; thisNet < numSsid; thisNet++)
  {
    for (i = 0; i < numAP; i++)
    {
      if (strstr (WiFi.SSID(thisNet) , ssid[i].c_str()))
      {
        j = WiFi.RSSI(thisNet);
        if (j > minVal)
        {
          minVal = j;
          minAP = i;
        }
      }
    }
  }

  return (minAP);
}
