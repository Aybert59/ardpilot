#include <math.h>

//////////////////////////////////////////
//
// movement primitives
//
//////////////////////////////////////////

void log_obstacle (char cote, int val)
{
      obuffer[0] = C_LOG;
      strcpy (&(obuffer[1]), "Obstacle X ");
      obuffer[10] = cote;
      itoa (val, &(obuffer[12]), 10);
      wifi_write ();

}

int dif_angle(int x, int y)
{
  int arg;

  arg = (y-x) % 360;
  if (arg < -180 )  arg  = arg + 360;
  if (arg > 180) arg  = arg - 360;

  return (-arg);
}


void primitive_turn(int SpeedInt, int SpeedExt, long dl, char *ButtonID) {
long cap, delta;
static long previousDelta = 999;

  // determine which way to turn
  cap = (long) get_compas (NULL, 0);
  delta = (long) dif_angle (PrimValue, cap);
  if (previousDelta == 999) {
    previousDelta = delta;
  }
  
// debug pour le moment
//obuffer[0] = C_LOG;
//strcpy (&(obuffer[1]), "correction ");
//itoa (delta, &(obuffer[12]), 10);
//wifi_write ();


 
        if ((dl >= DistanceMin) // no obstacle
          && ((delta * previousDelta) > 0)) // target in the same direction as previously
        { 
            if (delta < 0) {
              ServoLeft.writeMicroseconds (1500 + SpeedInt);
              ServoRight.writeMicroseconds (1500 - SpeedExt);
            } else {
              ServoLeft.writeMicroseconds (1500 + SpeedExt);
              ServoRight.writeMicroseconds (1500 - SpeedInt);
            }
            
            previousDelta = delta;
        } else { // we have turn past the objective or detected an obstacle ==> stop turning 

          ServoLeft.writeMicroseconds (1500);
          ServoRight.writeMicroseconds (1500);

          // return true if we have reached the requested angle ; flase if stopped by an obstacle
          obuffer[0] = C_COLOR;
          obuffer[1] = SequencePrimitive;
          obuffer[2] = Primitive;
                    
          if (dl >= DistanceMin)
            obuffer[3] = 'T';  
          else
            obuffer[3] = 'F';
            
          strcpy (&(obuffer[4]), ButtonID);
  
          wifi_write ();
          
          Primitive = P_NULL;
          previousDelta = 999;
        }

 }


void primitive_test_engine (char engine) {

delay(200);
obuffer[0] = C_LOG;
strcpy (&(obuffer[1]), "consigne : ");
itoa ((unsigned int) PrimValue, &(obuffer[12]), 10);
wifi_write ();
        
  if (engine == P_TEST_LEFT) {
    ServoLeft.writeMicroseconds(1500 + PrimValue);
  } else {
    ServoRight.writeMicroseconds(1500 - PrimValue);
  }

  if (PrimValue == 0) {
    Primitive = P_NULL;
  }
}

void primitive_avant (long dl) 
{
  int cap;
  int delta = AjustementMoteur; 
  
  // attention penser à ajuster le seuil d'obstacle, et la correction d'angle, en fonction de la vitesse        

  if (CapASuivre >= 0) 
  {
    cap = get_compas (NULL, 0);
    delta += (int) round (dif_angle (CapASuivre, cap) * FacteurAlignement); // ajoute le delta d'angle à la correction. Si instable proposer un facteur multiplicatif dans fichier config

// debug pour le moment
//obuffer[0] = C_LOG;
//strcpy (&(obuffer[1]), "correction ");
//itoa (delta, &(obuffer[12]), 10);
//wifi_write ();

  }
  
    if ((dl >= DistanceMin) && (PrimValue !=0) && (millis() < Decompte))
    { // objective still to reach : continue
        ServoLeft.writeMicroseconds (1500 + PrimValue + delta);
        ServoRight.writeMicroseconds (1500 - PrimValue + delta);
    } else { // we have etected an obstacle ==> stop engines 

      ServoLeft.writeMicroseconds(1500);
      ServoRight.writeMicroseconds(1500);
    
      obuffer[0] = C_PRI;
      obuffer[1] = SequencePrimitive;
      obuffer[2] = Primitive;
      if (dl < DistanceMin) {
        obuffer[3] = 'F';
      } else {
        obuffer[3] = 'T';
      }
      obuffer[4] = '\0';
      wifi_write ();
          
      Primitive = P_NULL;

    }
}


void primitive_suivi (long dl) 
{
  float delta = AjustementMoteur; 
  float reduction = 25.0;
  int maxcorrection = 15;
  int correction;
  
  // attention penser à ajuster le seuil d'obstacle, et la correction d'angle, en fonction de la vitesse      

    // reduction = 20 un peu juste pour vitesse 20, essayer 25 ?
    // si détecte un obstacle (genre coin) : il devient instable : caper la correction ? genre à 20 max ?
  
  if (Primitive == P_SUIVI_D)
    delta += round (((dl - DistASuivre) / reduction) * FacteurAlignement); 
  else
    delta += round (((DistASuivre - dl) / reduction) * FacteurAlignement); 

  delta = delta / 5.8; // distance en mm

  if (delta > 0.0)
  {
    if (delta < 2.0) 
    {
      delta = 2.0;
    }  else if (delta < 10.0)
    {
      delta = delta;
    } else if (delta < 20.0) {
      delta = 10.0 + (delta - 10.0) / 2.0;
    } else if (delta < 40.0) {
      delta = 15.0 + (delta - 20.0) / 3.0;
    } else
      delta = 28.0;
  } else {
    if (delta > -2.0) 
    {
      delta = -2.0;
    }  else if (delta > -10.0)
    {
      delta = delta;
    } else if (delta > -20.0) {
      delta = -10.0 + (delta + 10.0) / 2.0;
    } else if (delta > -40.0) {
      delta = -15.0 + (delta + 20.0) / 3.0;
    } else
      delta = -28.0;
  }
    
// debug pour le moment
obuffer[0] = C_LOG;
strcpy (&(obuffer[1]), "delta ");
ltoa (delta, &(obuffer[12]), 10);
wifi_write ();

  correction = (int) round (delta);
  
    if ((PrimValue !=0) && (millis() < Decompte))
    { // objective still to reach : continue
        ServoLeft.writeMicroseconds (1500 + PrimValue + correction);
        ServoRight.writeMicroseconds (1500 - PrimValue + correction);
    } else { 

      ServoLeft.writeMicroseconds(1500);
      ServoRight.writeMicroseconds(1500);
    
      obuffer[0] = C_PRI;
      obuffer[1] = SequencePrimitive;
      obuffer[2] = Primitive;
      obuffer[3] = 'T';
      obuffer[4] = '\0';
      wifi_write ();
          
      Primitive = P_NULL;

    }
}
