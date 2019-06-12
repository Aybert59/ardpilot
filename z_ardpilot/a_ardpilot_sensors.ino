//////////////////////////////////////////
//
// Sensors reading
//
//////////////////////////////////////////

#include <MemoryFree.h>

// addresses for the magneto/accel/gyro 
#define BNO055_ADDRESS 0x28



// docs et refs du compas https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/downloads 

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
  Wire.requestFrom(Address, Nbytes); 
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

// Ultrasonic sensors reading

long measure_distance_us (int n) {
  long d;

  d = SonarAV.ping_median(n);
    
  return d;
}

int measure_distance_lidar_i2c () {
  int val = -1;
  
  Wire.beginTransmission((int)Lidar_address); // transmit to LIDAR-Lite
  Wire.write((int)0x00); // sets register pointer to  (0x00)  
  Wire.write((int)0x04); // sets register pointer to  (0x00)  
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
/*
long measure_distance_lidar_pwm (int n) {
  long d, s, moy, ecart = 0, somme = 0;
  int i, j;
  long val[7];

  if (n > 7)
    n = 7;
    
  for (i=0;i<n;i++)
  {
    val[i] = get_lidar_val_pwm();
    somme += val[i];
  }
  moy = somme / n; 
 
  for (i=0;i<n;i++)
  {
    s += (val[i] - moy) * (val[i] - moy);
  }
  ecart = sqrt(s/n);
  
  j = n;
  for (i=0;i<n;i++)
  {
    if (abs(val[i] - moy) > ecart)
    {
      val[i] = 0;
      j--;
    }
  }
  somme = 0;
  for (i=0;i<n;i++)
  {
    somme += val[i];
  }
  
  moy = somme / j;
  
  return moy;
}
*/

long measure_distance (int n) {
  long li,us, d;
  
  li = (long) measure_distance_lidar_i2c ();
  us = measure_distance_us (n);

  // à affiner

  if (li > 40)
    d = li - 10.0;
  else 
    d = (us / 58.0) - 2 ; // 58 pour vitesse du son ; 2 cm longueur de l'autre capeur qui dépasse

  return d;
  
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
return;

  // Set normal power mode
  I2CwriteByte (BNO055_ADDRESS,0x3e,0x00);
  delay(10);

  // select register data units
  unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  I2CwriteByte (BNO055_ADDRESS, 0x3b, unitsel);
  delay(10);

  // remap the axes
  I2CwriteByte (BNO055_ADDRESS,0x41,0x06);
  delay(10);
  I2CwriteByte (BNO055_ADDRESS,0x42,0x07);
  delay (10);

  // Set operation mode (full fusion)
  I2CwriteByte (BNO055_ADDRESS,0x3d,0x0c);
  delay(10);
  
  ecran.clear();
  ecran.setPowerSave(0);
  ecran.print("Setup OK\n");
  delay(2000);
}

  
/*
int get_compas (bool callibration) {
  int x,y,z; //triple axis data
  double angle;

  Wire.beginTransmission(Compass_address); //open communication with HMC5883 compass
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  delay(20);

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(Compass_address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(Compass_address, 6); 
  if(6<=Wire.available()){
     x = Wire.read()<<8; //X msb
     x |= Wire.read(); //X lsb
     z = Wire.read()<<8; //Z msb
     z |= Wire.read(); //Z lsb
     y = Wire.read()<<8; //Y msb
     y |= Wire.read(); //Y lsb
  }

  if (callibration == false)
  {
    angle = atan2((double)y,(double)x) * 57.2957796 + 180.0;
    return ((int) angle);
  } else {
    if (x < xmmin) xmmin = x;
    if (x > xmmax) xmmax = x;
    if (y < ymmin) ymmin = y;
    if (y > ymmax) ymmax = y;
    if (z < zmmin) zmmin = z;
    if (z > zmmax) zmmax = z;
    delay(50);
    return 0;
  }
}
*/

int get_compas (bool callibration) {
  int8_t temp;
  uint8_t euler[6];
  int16_t heading=0, pitch=0, roll=0;
return (0);
//debug
  ecran.clear();
  ecran.setPowerSave(0);
  ecran.print("BNO 055\n");
  
  I2Cread (BNO055_ADDRESS, 0x34, 1, &temp);
  I2Cread (BNO055_ADDRESS, 0x1a, 6, euler);

  heading = (((uint16_t)euler[1]) << 8) | ((uint16_t)euler[0]);
  roll = (((uint16_t)euler[3]) << 8) | ((uint16_t)euler[2]);
  pitch = (((uint16_t)euler[5]) << 8) | ((uint16_t)euler[4]);


// afficher les valeurs sur l'écran

  ecran.clear();
  ecran.setPowerSave(0);
  
  ecran.setCursor(0, 1);
  ecran.print("head");
  ecran.setCursor(6, 1);
  ecran.print(heading);
  
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
    
  return (heading);
}

void get_angle(char sequence) {
  int a;
  
  a = get_compas (false);
  obuffer[0] = C_CMP;
  obuffer[1] = sequence;
  itoa (a, &(obuffer[2]), 10);

  wifi_write();
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


// variables created by the build process when compiling the sketch


extern int __bss_end;
extern void *__brkval;

// function to return the amount of free RAM
 
void memoryFree(unsigned char sequence)
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
  obuffer[1] = sequence;
  itoa (freeValue, &(obuffer[2]), 10);

  wifi_write ();
}

void batteryLevel()
{
  unsigned int raw_bat;
 
  analogReference (INTERNAL);
  raw_bat = analogRead(A0);
  
  obuffer[0] = C_BAT;
  itoa (raw_bat, &(obuffer[1]), 10);

  wifi_write ();
}
