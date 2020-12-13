//////////////////////////////////////////
//
// Sensors reading
//
//////////////////////////////////////////

#include <MemoryFree.h>

// addresses for the magneto/accel/gyro 
#define CMPS12_ADDRESS 0x60


// docs et refs du compas https://robot-electronics.co.uk/files/cmps12.pdf and https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/downloads 
// docs du lidar : http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf
// attention contrairement à documentation, le fil jaune doit être relié à la masse !

void I2Cscan()
{
  byte error, address;
  int nDevices;
 
  ecran.clear();
  ecran.setPowerSave(0);
  ecran.print("scanning i2c...\n");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      ecran.print(" ");
      ecran.print(address,HEX);
        
      nDevices++;
    }
    else if (error==4)
    {
      ecran.print(" *");
      ecran.print(address,HEX);
      ecran.print("*");
    }    

    delay(20);
  }
  if (nDevices == 0)
    ecran.print(" No I2C devices found\n");
  else
    ecran.print(" done\n");
  
}

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  uint8_t index=0;
  byte error;
  
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  error = Wire.endTransmission();
  if (error != 0)
  {
      ecran.print("No I2C device\n");
      ecran.print("read at ");
      ecran.print(Address);
      ecran.print("\n");
      ecran.print("error ");
      ecran.print(error);
      ecran.print("\n");
      delay (5000);
  }
  delay(20); // Wait 20ms for transmit
  
  // Read Nbytes
  Wire.requestFrom((int)Address, (int)Nbytes); 
  while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  byte error;
  
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);

  delay(20); // Wait 20ms for transmit
  
  Wire.write(Data);
  error = Wire.endTransmission();
    if (error != 0)
  {
      ecran.print("No I2C device\n");
      ecran.print("write at ");
      ecran.print(Address);
      ecran.print("\n");
      ecran.print("error ");
      ecran.print(error);
      ecran.print("\n");
      delay (5000);
  }
}

void initialize_lidar() {
  
  I2CwriteByte(Lidar_address, 0x00, 0x00);
  delay (200);
  // Default mode, balanced performance
  I2CwriteByte(Lidar_address, 0x02, 0x80); 
  I2CwriteByte(Lidar_address, 0x04, 0x08); 
  I2CwriteByte(Lidar_address, 0x1c, 0x00); 
  
}

  
// Ultrasonic sensors reading

long measure_distance_us (int n) {
  long d;

  //d = SonarAV.ping_median(n);

   digitalWrite(TRIG_U, HIGH);
   delayMicroseconds(10);
   digitalWrite(TRIG_U, LOW);
   d = pulseIn(ECHO_U, HIGH);
 
  return d;
}

int measure_distance_lidar_i2c () {
  int val = -1;
  
  Wire.beginTransmission((int)Lidar_address); // transmit to LIDAR-Lite
  Wire.write((int)0x00); // sets register pointer to  (0x00)  
  Wire.write((int)0x04); // write 4  (0x04)  
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.beginTransmission((int)Lidar_address); // transmit to LIDAR-Lite
  Wire.write((int)0x8f); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit
  
  Wire.requestFrom((int)Lidar_address, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    val = Wire.read(); // receive high byte (overwrites previous reading)
    val = val << 8; // shift high byte to be high 8 bits
    val |= Wire.read(); // receive low byte as lower 8 bits
  }
  
  return val;
  
}


long measure_distance (int n) {
  long li,us, d;
  long dr = 0;
  int i;
  
  for (i=0; i<n; i++)
  {
    li = (long) measure_distance_lidar_i2c ();
    us = measure_distance_us (n);

    // à affiner

    if (li > 40)
      dr += li - 10.0;
    else 
      dr += (us / 58.0) - 2 ; // 58 pour vitesse du son ; 2 cm longueur de l'autre capeur qui dépasse
  }
  
  return (dr/i);
}

long measure_distance_stable () {
  long d1, d2;
  float error = 1, delta;
  
  d1 = measure_distance (1);
  while (error > 0.05)
  {
    d2 = measure_distance (1);
    delta = (d2 - d1) * 1.0;
    error = abs (delta) / d2;

    d1 = d2;
  }
     
  return d2;
}


void get_distance (byte side) {
 // long d;
    
 // d = measure_distance (3);
  
  obuffer[0] = side;
  ltoa (measure_distance (3), &(obuffer[1]), 10);
  wifi_write ();


}

void scan_distance (byte side, char sequence) {
int angle, d;
byte segment;

  if (side == C_SCANH) {
    servo2.write (0);
  } else {
    servo2.write (90);
  }

  // lecture en 6 segments de 30 points (int) chacuns
  // tq on remplisse un buffer de 60 octets + header + '\0'

  obuffer[0] = side;
  obuffer[1] = sequence;
  
  for (segment=0; segment<6; segment++)
  {
    obuffer[2] = segment + 1;
    for (angle=0; angle<30; angle++)
    {
      servo1.write (angle + 30*segment);
      delay (ServoDelay);

      d = (unsigned int) measure_distance (3); // (limite à 5m50 environ, suffisant car capteur ultrason va jusqu'a 1m80 !
       
      obuffer[3+(angle*2)] = highByte (d);
      obuffer[4+(angle*2)] = lowByte (d);  // si 0 alors on ajoute 1
    }
    wifi_write_binary (63);
  }
  

}


void callibrate_dist_sensors () {
  unsigned long h1, h2;
  int angle = 0;
  int pse = 100;


// en fait la méthode ne fonctionne pas car le capteur est trop aléatoire.
// il faudrait idéalement prendre plusieurs échantillons puis moyenner à chaque pas d'angle (minimum 5 mesures espacées de 200 ms au moins
// --> créer une fonction "mesure_distance_précise" qui fasse une boucle de 5 mesures et renvoie la moyenne des 5

// pour le moment faisons rapide
posRepos1 = 2;
posRepos2 = 90;
return;


  servo2.write (90); 
  servo1.write (angle); //mise vers le bas absolu
  
  h1 = measure_distance (3);
  angle += 1;
  servo1.write (angle);
  delay(pse);
  h2 = measure_distance (3);
  
  while (h2 < h1) {
    h1 = h2;
    angle += 1;
    servo1.write (angle);
    delay(pse);
    h2 = measure_distance (3);
  }
  posRepos1 = angle - 1;
  obuffer[0] = C_SR1;
  obuffer[1] = '\1';
  itoa (posRepos1, &(obuffer[2]), 10);
  wifi_write();
  delay(200);

  
  angle = 70;
  servo2.write (angle);
  delay(pse);
  h1 = measure_distance (3);
  
  angle += 1;
  servo2.write (angle);
  delay(pse);
  h2 = measure_distance (3);
  
  while (h2 <= h1) {
    h1 = h2;
    angle += 1;
    servo2.write (angle);
    delay(pse);
    h2 = measure_distance (3);

  }
  
  posRepos2 = angle - 1;

    obuffer[0] = C_SR2;
    obuffer[1] = '\1';
    itoa (posRepos2, &(obuffer[2]), 10);
    wifi_write();
    delay(200);
    
}


// compass reading

void initialize_bno055() {
  uint8_t unitsel;

//this procedure for real BNO055. Not used for CMPS12

// nevertheless should turn 360° to initialize compass


}

int bin_get_compas (char outstr[])  // outstr supposed to have 6 Bytes available
{  
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(2);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 4 bytes from the CMPS12
  // this will give us both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 4);       
  while(Wire.available() < 4);        // Wait for all bytes to come back
  
  outstr[0] = Wire.read(); // head_high
  outstr[1] = Wire.read(); // head_low
  outstr[2] = Wire.read(); // pitch
  outstr[3] = Wire.read(); // roll

  
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0x18);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  Wire.requestFrom(CMPS12_ADDRESS, 2);       
  while(Wire.available() < 2);        // Wait for all bytes to come back
  
  outstr[4] = Wire.read(); // temp_high
  outstr[5] = Wire.read(); // temp_low
   
  return (6); // number of bytes added
}

int get_compas (char *outstr, int outlen) {
  int8_t temp_high=0, temp_low=0, head_high=0, head_low=0, pitch=0, roll=0;
  int16_t angle16=0, temp=0;
  int i;
  
//debug
//  ecran.clear();
//  ecran.setPowerSave(0);
//  ecran.print("CMPS12\n");
  
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(2);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 4 bytes from the CMPS12
  // this will give us both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 4);       
  
  while(Wire.available() < 4);        // Wait for all bytes to come back
  
  head_high = Wire.read();
  head_low = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();
  
  angle16 = head_high;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += head_low;

  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0x18);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  Wire.requestFrom(CMPS12_ADDRESS, 2);       
  
  while(Wire.available() < 2);        // Wait for all bytes to come back
  
  temp_high = Wire.read();
  temp_low = Wire.read();
   
  temp = temp_high;                 // Calculate 16 bit angle
  temp <<= 8;
  temp += temp_low;
  
// afficher les valeurs sur l'écran
/*
  ecran.clear();
  ecran.setPowerSave(0);
  
  ecran.setCursor(0, 1);
  ecran.print("head");
  ecran.setCursor(6, 1);
  ecran.print(angle16/10);
  
  ecran.setCursor(0, 2);
  ecran.print("roll");
  ecran.setCursor(6, 2);
  ecran.print(roll);
  
  ecran.setCursor(0, 3);
  ecran.print("pitch");
  ecran.setCursor(6, 3);
  ecran.print(pitch);
  
  ecran.setCursor(0, 5);
  ecran.print("temp");
  ecran.setCursor(6,5);
  ecran.print(temp);
*/
  if (outstr != NULL)
  {
      i=0;
      while (outstr[i] != '\0')
        i++;
      if (i < (outlen - 8))
      {
        outstr[i++] = ' ';
        itoa ((int)(angle16/10), &(outstr[i]), 10);
      }
      
      while (outstr[i] != '\0')
        i++;
      if (i < (outlen - 8))
      {
        outstr[i++] = ' ';
        itoa (roll, &(outstr[i]), 10);
      }

      while (outstr[i] != '\0')
        i++;
      if (i < (outlen - 8))
      {
        outstr[i++] = ' ';
        itoa (pitch, &(outstr[i]), 10);
      }

      while (outstr[i] != '\0')
        i++;
      if (i < (outlen - 8))
      {
        outstr[i++] = ' ';
        itoa (temp, &(outstr[i]), 10);
      }
  }
  return (angle16/10);
}

void get_angle()
{
  
    obuffer[0] = C_CMP;
    obuffer[1] = '\0';
    get_compas (obuffer, 64); 
  
    wifi_write();
}

int fast_get_compas () {
  int8_t head_high=0, head_low=0;
  int16_t angle16=0;
  int i;
  

  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(2);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 4 bytes from the CMPS12
  // this will give us both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 2);       
  while(Wire.available() < 2);        // Wait for all bytes to come back
  
  head_high = Wire.read();
  head_low = Wire.read();
   
  angle16 = head_high;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += head_low;

  return (angle16/10);
}

unsigned int cmps_get_calibration_state () {
  int8_t CalState=0;
  
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0x1E);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 1 bytes from the CMPS12
  Wire.requestFrom(CMPS12_ADDRESS, 1);       
  while(Wire.available() < 1);        // Wait for all bytes to come back
  
  CalState = Wire.read();
  
  return (CalState);
}

int bin_get_calibration_state (char outstr[])  // outstr supposed to have 6 Bytes available
{     
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0x1E);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  Wire.requestFrom(CMPS12_ADDRESS, 1);       
  while(Wire.available() < 1);        // Wait for all bytes to come back
 
  outstr[0] = Wire.read(); 
   
  return (1); // number of bytes added
}

void reset_calibration ()
{     
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0xE0);                    
  delay(20); // Wait 20ms for transmit
  Wire.write(0xE5); 
  delay(20); // Wait 20ms for transmit
  Wire.write(0xE2); 
  Wire.endTransmission(); // stop transmitting
}

int get_lidar_val_i2c (byte *s)
{
  int val = -1;
  int busyFlag = 1; 
  byte StatusReg;
  int busyCounter = 0;
//

  Wire.beginTransmission(Lidar_address); //open communication with Lidar lite V3
  Wire.write((int)0x00); // sets register pointer to  (0x00)  
  Wire.write((int)0x04); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting

  delay(20);

  while(busyFlag != 0) // Loop until device is not busy
  {
    // Read status register to check busy flag
    Wire.beginTransmission((int)Lidar_address);
    Wire.write(0x01); // Set the status register to be read
    Wire.endTransmission();
    
    Wire.requestFrom(Lidar_address,1); // Read register 0x01

    StatusReg = Wire.read();
    busyFlag = bitRead(StatusReg,0); // Assign the LSB of the status register to busyFlag

    busyCounter++; // Increment busyCounter for timeout

    if (busyCounter > 9999)
    {
      return -1;
    }
  }

  delay(20);
    
  Wire.beginTransmission(Lidar_address); // transmit to LIDAR-Lite
  Wire.write((int)0x8f); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(20);
  
  Wire.requestFrom((int)Lidar_address, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    val = Wire.read() << 8 ; // receive high byte (overwrites previous reading)
    val |= Wire.read(); // receive low byte as lower 8 bits
  }

  *s = StatusReg;
  return val;

}

int get_lidar_val_i2c_2 (byte *s)
{
  int val = -1;
  int busyFlag = 1; 
  byte StatusReg;
  int busyCounter = 0;

  while(busyFlag != 0) // Loop until device is not busy
  {
    // Read status register to check busy flag
    Wire.beginTransmission((int)Lidar_address);
    Wire.write(0x01); // Set the status register to be read
    Wire.endTransmission();
    
    Wire.requestFrom(Lidar_address,1); // Read register 0x01

    StatusReg = Wire.read();
    busyFlag = bitRead(StatusReg,0); // Assign the LSB of the status register to busyFlag

    busyCounter++; // Increment busyCounter for timeout

    if (busyCounter > 9999)
    {
      return -1;
    }
  }

  delay(20);
    
  Wire.beginTransmission(Lidar_address); // transmit to LIDAR-Lite
  Wire.write((int)0x8f); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(20);
  
  Wire.requestFrom((int)Lidar_address, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    val = Wire.read() << 8 ; // receive high byte (overwrites previous reading)
    val |= Wire.read(); // receive low byte as lower 8 bits
  }

  *s = StatusReg;
  return val;

}
/*
unsigned long get_lidar_val_pwm ()
{
  unsigned long val = 0;

 
  val = pulseIn(ECHO_L, HIGH); // Count how long the pulse is high in microseconds

  // If we get a reading that isn't zero, let's print it
  if(val != 0)
  {
    val = val / 10; // 10usec = 1 cm of distance
  } else
  { // We read a zero which means we're locking up. 
    digitalWrite(ENABLE_L,LOW); // Turn off the sensor
    delay(1);// Wait 1ms
    digitalWrite(ENABLE_L,HIGH); //Turn on te sensor
    delay(1);//Wait 1ms for it to turn on.
    val = pulseIn(ECHO_L, HIGH);
    if(val != 0)
    {
      val = val / 10; // 10usec = 1 cm of distance
    }
  }

  return val;
}
*/
/*
void get_lidar(char sequence) {
  int a;
  byte s = 255;
  
//  a = get_lidar_val_i2c (&s);
  a = (int) get_lidar_val_pwm();
  
  obuffer[0] = C_LIDAR;
  obuffer[1] = sequence;
  obuffer[2] = s;
  itoa (a, &(obuffer[3]), 10);

  wifi_write();

//  delay (100);

// l'accès en i2c fait planter le lidar
//  a = get_lidar_val_i2c_2 (&s); 
  
//  obuffer[0] = C_LIDAR;
//  obuffer[1] = sequence;
//  obuffer[2] = s;
//  itoa (a, &(obuffer[3]), 10);

//  wifi_write();
}
*/

// variables created by the build process when compiling the sketch


extern int __bss_end;
extern void *__brkval;

// function to return the amount of free RAM
 
void memoryFree()
{
   int freeValue;
   
/*   
   if((int)__brkval == 0)
      freeValue = ((int)&freeValue) - ((int)&__bss_end);
   else
     freeValue = ((int)&freeValue) - ((int)__brkval);
*/

  freeValue = freeMemory();
  
  obuffer[0] = C_MEM;
  obuffer[1] = '*';
  itoa (freeValue, &(obuffer[2]), 10);

  delay (200);   // don't be too fast, the WIFI interface cannot handle
  wifi_write ();

}

int bin_get_memory (char outstr[])  // outstr supposed to have 2 Bytes available
{  
  int freeValue;

  freeValue = freeMemory();
  outstr[0] = highByte (freeValue); 
  outstr[1] = lowByte (freeValue);
  
  return (2); // number of bytes added
}

void batteryLevel()
{
  unsigned int raw_bat;
 
  analogReference (DEFAULT);
  raw_bat = analogRead(A2);
  
  obuffer[0] = C_BAT;
  itoa (raw_bat, &(obuffer[1]), 10);

  wifi_write ();
}

int bin_get_battery (char outstr[])  // outstr supposed to have 2 Bytes available
{  
  unsigned int raw_bat;
 
  analogReference (DEFAULT);
  raw_bat = analogRead(A2);
  
  outstr[0] = highByte (raw_bat); 
  outstr[1] = lowByte (raw_bat);
  
  return (2); // number of bytes added
}
