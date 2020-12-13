#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ardpilot.h"

#include "ardpilot_grammar.h"
#define PIsur180 0.0174532925
#define AJUSTEMENT_ANGLE_SCAN 2
#define LONGUEUR_LIDAR 7.5

extern int DebugMode;

extern struct cellule Appartement[];

void draw_segment (char *buffer, char segment, char orientation)
{
    char message[384];
    char temp[16];
    int i;
    float angle;
    float dist;
    float fx, fy;
    int x, y;
    int msb, lsb;
    
    if (segment == 1)
    {
        if (orientation == C_SCANH)
            sprintf (message,"CLEARH");
        else
            sprintf (message,"CLEARV");
    }
    control_message(MSG_INFO, message, 10);
    
    if (orientation == C_SCANH)
        strcpy (message,"PLOTH");
    else
        strcpy (message,"PLOTV");

    
    for (i=0; i<30; i++)
    {
        
        angle = ((((int) segment - 1) * 30) + i - AJUSTEMENT_ANGLE_SCAN) * PIsur180; // -2 ajustement pour "redresser" l'image obtenue

        msb = (int) (buffer[(i*2)+3]);
        lsb = (int) (buffer[(i*2)+4]);
        dist = ((msb * 256) + lsb) + LONGUEUR_LIDAR;
               
        // puis calculer x et y en fonction distance et angle
        if (orientation == C_SCANH)
        {
            fx = - (dist * cos (angle));
            fy = dist * sin (angle);
            x = (int) round (fx);
            y = (int) round (fy);
        }
        else
        {
            fy = - (dist * cos (angle));
            fx = dist * sin (angle);
            x = (int) round (fx);
            y = (int) round (fy);
        }

        sprintf (temp, " %d %d", x, y);
        strcat (message, temp);
    }
    control_message(MSG_INFO, message, 10);
}

void consolidate_points (char *buffer, char segment, char orientation, double X[], double Y[], double mesures[])
{
    int i;
    double fx, fy;
    double angle, dist;
    int msb, lsb;
    int degre;
    
    for (i=0; i<30; i++)
    {
        degre = (((int) segment - 1) * 30) + i - AJUSTEMENT_ANGLE_SCAN;
        angle = degre * PIsur180;
        msb = (int) (buffer[(i*2)+3]);
        lsb = (int) (buffer[(i*2)+4]);
        dist = ((msb * 256) + lsb) + LONGUEUR_LIDAR;
        
        // puis calculer x et y en fonction distance et angle
        if (orientation == C_SCANH)
        {
            fx = - (dist * cos (angle));
            fy = dist * sin (angle);
        }
        else
        {
            fy = - (dist * cos (angle));
            fx = dist * sin (angle);
        }
        
        X[degre] = fx;
        Y[degre] = fy;
        mesures[degre] = dist;
    }

}

void draw_line (int xs, int ys, int xe, int ye)
{
    char message[64];
    
    sprintf (message, "DRAWLINE %d %d %d %d", xs, ys, xe, ye);
    if (DebugMode ==1)
        printf ("drawing line %s\n", message);
    control_message(MSG_INFO, message, 10);  // semble ne pas fonctionner, rien n'apparait chez node.js ?
}

void clear_plan ()
{
    control_message(MSG_INFO, "CLEARPLAN", 10);
}

void draw_plan ()
{
    char message[1024];
    char temp[10];
    int i,j;

    // murs : envoyer en plusieurs fois


    for (i=0;i<APPT_L;i++)
    {
        strcpy (message,"DRAWMURS  ");
        for (j=0;j<APPT_W;j++)
        {
            if (Appartement[i * APPT_W + j].piece == 0x0F)
            {
                sprintf (temp, "%d,%d ", j,i);
                strcat (message, temp);
            }
        }
        control_message(MSG_INFO, message, 0);

    }
    
    // lieu des matrices de référence WIFI
    // attention 1 point = 8 octets (donc au max 256 points)
    
/*    sleep(1); // necessaire sinon le javascript n'a pas le temps de digérer  l'envoi précédent
    strcpy (message,"DRAWREFS");
    for (i=0;i<193;i++)
    {
        for (j=0;j<93;j++)
        {
            if (Appartement[i * 93 + j].WifiMatrix != 0)
            {
                sprintf (temp, "%d,%d ", j,i);
                strcat (message, temp);
            }
        }
    }

    control_message(MSG_INFO, message);
*/
}

void draw_matched_scan (double x[], double y[], int taille, int posx, int posy)
{
    char message[400];
    char temp[10];
    int i, j, s;
    int point;
    
    s = taille / 6;
    for (j=0; j<7; j++)
    {
        point = 0;
        strcpy (message,"DRAWMAPSCAN Crimson  ");
        for (i=0; i<s; i++)
        {
            if ((j*s +i) < taille)
            {
                point = 1;
                sprintf (temp, "%.0f,%.0f ", posx + (x[j*s +i] / 10), posy - (y[j*s +i] / 10));
                strcat (message, temp);
            }
        }
        if (point == 1)
        {
            control_message(MSG_INFO, message, 10);
            if (DebugMode == 1)
                printf ("%s-\n",message);
        }
    }
}

void draw_location (int posx, int posy)
{
    char message[32];
 
    sprintf (message,"DRAWLOCATION Red  %d,%d", posx, posy);
    control_message(MSG_INFO, message, 10);
//    if (DebugMode == 1)
            printf ("%s\n",message);
}




void draw_path (int x[], int y[], int taille)
{
    char message[400];
    char temp[10];
    int i, j, s;
    int point;
    
    if (DebugMode == 1)
    {
        for (i=0;i<taille-1;i++)
            printf ("%d,%d\n",x[i],y[i]);
    }
    
    s = taille / 6;
    for (j=0; j<7; j++)
    {
        point = 0;
        strcpy (message,"DRAWMAPPATH Green  ");
        for (i=0; i<s; i++)
        {
            if ((j*s +i) < taille-1)
            {
                point = 1;
                sprintf (temp, "%d,%d ", x[j*s +i], y[j*s +i]);
                strcat (message, temp);
            }
        }
        if (point == 1)
        {
            control_message(MSG_INFO, message, 10);
        }
    }
    
}

void display_room_from_matrix (unsigned int zone, char *color)
{
    char message[1024];
    char temp[10];
    int i,j;
    
    clear_plan();
    draw_plan ();
    
    printf ("drawing zone %2X\n",zone);
    for (i=0;i<193;i++)
    {
        sprintf (message,"DRAWCOLOR %s  ", color);
        for (j=0;j<93;j++)
        {
            if (Appartement[i * 93 + j].piece == zone)
            {
                sprintf (temp, "%d,%d ", j,i);
                strcat (message, temp);
            }
        }
        control_message(MSG_INFO, message, 0);
        
    }
}


void display_robot_status (unsigned char *buffer)
{
    // format of buffer :
    // from 0 to 5 | headH | headL | pitch | roll | tempH | tempL |
    
    int head, pitch, roll, temp;
    int Compas;
    int memory;
    float volts;
    unsigned int CalibrationState, calibration;
    char message[64];
    extern int compas_correction[];
    
    head = ((int)(buffer[0]) * 256 + (int)(buffer[1])) / 10; // CMP12 gives heading at .1 degree precision
    pitch = (int)(buffer[2]);
    roll = (int)(buffer[3]);
    temp = (int)(buffer[4]) * 256 + (int)(buffer[5]);
    
    
    if ((head >= 0) && (head < 360))
        Compas = compas_correction[head];
    else
    {
        printf ("    ARD-->Server STATUS %d weird head value\n", head);
        Compas = 0;
    }
    
    memory = (int)(buffer[6]) * 256 + (int)(buffer[7]);
    volts = ((int)(buffer[8]) * 256 + (int)(buffer[9])) * 10.0 / 1023;
    CalibrationState = (unsigned int) buffer[10];
    // seuls les bits 7 et 8 semblent significatifs
    calibration = CalibrationState >> 6;
    
 /*   if (DebugMode == 1) // in case of emergency only (too noisy)
    {
      printf ("    ARD-->Server STATUS head : %d (%d) - temp : %d°\n", head, Compas, temp);
        printf ("    ARD-->Server STATUS memory : %d - voltage : %.2f°\n", memory, volts);

        printf ("Calibration : %X %d\n", CalibrationState, calibration);
    }
 */
    
    
    sprintf (message, "STATUS %d %d %d %.2f %d", Compas, temp, memory, volts, calibration);
    control_message(MSG_INFO, message, 10);
}
