#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ardpilot.h"

#include "ardpilot_grammar.h"
#define PIsur180 0.0174532925


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
    {
        strcpy (message,"PLOTH");
    }
    else
    {
        strcpy (message,"PLOTV");
    }

    for (i=0; i<30; i++)
    {
        // il faudrait aussi le garder dans un seul tableau (sous forme d'angle)
        // afin de pouvoir ensuite faire les calculs
        
        angle = ((((int) segment - 1) * 30) + i) * PIsur180;
//printf ("%d,",(((int) segment - 1) * 30) + i);
//printf ("-- %f - ", angle);
        msb = (int) (buffer[(i*2)+3]) - 128;
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
        degre = (((int) segment - 1) * 30) + i;
        angle = degre * PIsur180;
        msb = (int) (buffer[(i*2)+3]) - 128;
        lsb = (int) (buffer[(i*2)+4]);
        dist = ((msb * 256) + lsb) / 58.0 ; // The speed of sound is 340 m/s or 29 microseconds per centimeter.
        
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
