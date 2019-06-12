#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ardpilot.h"

#include "ardpilot_grammar.h"
#define PIsur180 0.0174532925
#define AJUSTEMENT_ANGLE_SCAN 2

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
    
    if (orientation == C_SCANH)
        strcpy (message,"PLOTH");
    else
        strcpy (message,"PLOTV");

    for (i=0; i<30; i++)
    {
        
        angle = ((((int) segment - 1) * 30) + i - AJUSTEMENT_ANGLE_SCAN) * PIsur180; // -2 ajustement pour "redresser" l'image obtenue
//printf ("%d,",(((int) segment - 1) * 30) + i);
//printf ("-- %f - ", angle);
//-        msb = (int) (buffer[(i*2)+3]) - 128;
        msb = (int) (buffer[(i*2)+3]);
        lsb = (int) (buffer[(i*2)+4]);
        //       dist = ((msb * 256) + lsb) / 58.0 ; // Ultrasonic : The speed of sound is 340 m/s or 29 microseconds per centimeter.
                                             // The ping travels out and back so divide by 2
        dist = ((msb * 256) + lsb);
        
//printf ("%f,",dist);
//printf ("%d+%d %f - ",msb,lsb,dist);
        
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
//printf ("%1f,%1f\n",fx,fy);
        
        sprintf (temp, " %d %d", x, y);
        strcat (message, temp);
    }
    control_message(MSG_INFO, message);
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
        dist = ((msb * 256) + lsb);
        
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
    control_message(MSG_INFO, message);  // semble ne pas fonctionner, rien n'apparait chez node.js ?
}

void clear_plan ()
{
    control_message(MSG_INFO, "CLEARPLAN");
}

void draw_plan ()
{
    char message[1024];
    char temp[10];
    int i,j;

    // murs : envoyer en plusieurs fois


    for (i=0;i<193;i++)
    {
        strcpy (message,"DRAWMURS  ");
        for (j=0;j<93;j++)
        {
            if (Appartement[i * 93 + j].piece == 0x0F)
            {
                sprintf (temp, "%d,%d ", j,i);
                strcat (message, temp);
            }
        }
        control_message(MSG_INFO, message);

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
            control_message(MSG_INFO, message);
            if (DebugMode == 1)
                printf ("%s-\n",message);
        }
    }
}

void display_room_from_matrix (unsigned int zone)
{
    char message[1024];
    char temp[10];
    int i,j;
    
    clear_plan();
    draw_plan ();
    
    printf ("drawing zone %2X\n",zone);
    for (i=0;i<193;i++)
    {
        strcpy (message,"DRAWCOLOR Aquamarine  ");
        for (j=0;j<93;j++)
        {
            if (Appartement[i * 93 + j].piece == zone)
            {
                sprintf (temp, "%d,%d ", j,i);
                strcat (message, temp);
            }
        }
        control_message(MSG_INFO, message);
        
    }
}
