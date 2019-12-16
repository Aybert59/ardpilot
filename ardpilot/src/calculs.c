#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "ardpilot_grammar.h"
#include "ardpilot.h"
#define PIsur180 0.0174532925

extern int DebugMode;
extern struct cellule Appartement[APPT_L][APPT_W];
extern float ApptDistances[APPT_L+BORDER*2][APPT_W+BORDER*2]; // appartment size + 1 meter all around

// from http://www.tangentex.com/RegLin.htm
void RegLineaire (double x[], double y[], int n, double *a, double *b)
{
    int i;
    double xsomme, ysomme, xysomme, xxsomme;
    double ai,bi;
    
    xsomme = 0.0;
    ysomme = 0.0;
    xysomme = 0.0;
    xxsomme = 0.0;
    
    for (i=0;i<n;i++)
    {
        xsomme = xsomme + x[i]; ysomme = ysomme + y[i];
        xysomme = xysomme + x[i]*y[i];
        xxsomme = xxsomme + x[i]*x[i];
    }
    
    ai = (n*xysomme - xsomme*ysomme)/(n*xxsomme - xsomme*xsomme);
    bi = (ysomme - ai*xsomme)/n;
    
    *a = ai;
    *b = bi;
    
    return;
}

// from http://www.commentcamarche.net/forum/affich-15253091-fonction-coefficient-de-correlation-en-c

double correlationPearson (double tableauX[], double tableauY[], int tailleTableau)
{
    double sommeXY=0; //somme des Xi*yi
    double sommeX=0;//somme des Xi
    double sommeY=0;//somme des Yi
    double sommeXiCarre=0;//somme des xi carré
    double sommeYiCarre=0;
    double r;
    int i=0;
    

    for( i=0; i<tailleTableau; i++)
    {
        sommeX+=tableauX[i];
        sommeY+=tableauY[i];
        sommeXiCarre+=pow(tableauX[i], 2.0);
        sommeYiCarre+=pow(tableauY[i], 2.0);
        sommeXY=sommeXY+(tableauX[i]*tableauY[i]);
    }
    
    //formule coefficient de correlation de pearson
    // ((n*sommeXY)-sommeX*sommeY)/Racine((n*sommeXiCarre-(sommeX)^2)*(n*sommeYiCarre-(sommeY)^2))
    r=((tailleTableau*sommeXY)-sommeX*sommeY)/sqrt((tailleTableau*sommeXiCarre-pow(sommeX,2.0))*(tailleTableau*sommeYiCarre-pow(sommeY,2.0)));
    return r; 
}

double covariance (double tableauX[], double tableauY[], int tailleTableau)
{
    double sommeX=0;//somme des Xi
    double sommeY=0;//somme des Yi
    double moyX=0;//moyenne des Xi
    double moyY=0;//moyenne des Yi
    double somme=0;
    int i=0;
    
    
    for( i=0; i<tailleTableau; i++)
    {
        sommeX+=tableauX[i];
        sommeY+=tableauY[i];
    }
    moyX = sommeX / tailleTableau;
    moyY = sommeY / tailleTableau;
    
    for( i=0; i<tailleTableau; i++)
    {
        somme += (tableauX[i] - moyX) * (tableauY[i] - moyY);
    }
    
    return (somme / tailleTableau);
}

double calculelargeur (double x1, double x2, double y1, double y2)
{
    return sqrt (((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)));
}

// deteclines et detectpath renvoient des segments caractérisés par :
// - angle du point median du segment, par rapport au capteur du robot (!! 0° à gauche, 180 à droite)
// - distance de ce point median par rapport au robot
// - largeur de l'objet en degrés d'angle
// - orientation de l'objet par rapport au robot
//          (angle  par rapport au cap du robot, tq si le robot tourne de cet angle il y fait face)

void detectlines (double x[], double y[], double points[], int taille, int ecart, float precision)
{
    // taille : des tableaux (180 en principe)
    // ecart : longeur minimale de segment à détecter (en nb de points)
    // precision : seuil de covariance maximale
    
    // a voir quels parametres on retourne, et comment...
    // pour le moment on les imprime
    
    int i = 0;
    int largeur = ecart;
    double c;
    double a, b;
    int med;
    double angle;
    double mesure;

    if (DebugMode == 1)
        printf ("DetectLines on %d points, width %d, precision %0.4f\n",taille, ecart, precision);
    draw_line (-10, 0, 10, 0); // bug ? le premier segment n'est pas dessiné ?
    sleep(1);
    while (i+largeur < taille)
    {
        
        c = fabs (correlationPearson (&(x[i]), &(y[i]), largeur));
        if (DebugMode == 1)
            printf ("\tTesting i=%d (x: %0.2f y: %0.2f), getting correlation %0.4f\n",i,x[i],y[i], c);
        if (c > precision)
        {
            while ((c > precision) && ((i + largeur) < taille))
            {
                largeur++;
                c = fabsf (correlationPearson (&(x[i + largeur - ecart]), &(y[i+ largeur - ecart]), largeur));
            }
            largeur--;
            c = fabs (correlationPearson (&(x[i]), &(y[i]), largeur));
            RegLineaire (&(x[i]), &(y[i]), largeur, &a, &b);
            if (DebugMode == 1)
                printf ("found segment (%f) starting at %d, angle width %d\n", c, i, largeur);
            med = i + largeur/2;
            if (DebugMode == 1)
            {
                printf ("\ty = %f*x + %f - median angle %d\n",a,b,med);
                printf ("\tFrom x=%.0f y=%.0f to x=%.0f y=%.0f\n",x[i],y[i],x[i+largeur],y[i+largeur]);
            }
            draw_line ((int) round(x[i]), (int) round(y[i]), (int) round(x[i+largeur]), (int) round(y[i+largeur]));
            sleep(1);
            // affichage normalisé
            angle = - atan(a) / PIsur180; // angle à ajouter au cap actuel pour faire face au segment.
            mesure = calculelargeur (x[i], x[i+largeur], y[i], y[i+largeur]);
            
            if (DebugMode == 1)
            {
                printf ("segment normalisé : \n");
                printf ("\tangle median de detection : %d\n",med);
                printf ("\tdistance point median : %.1f\n",points[med]);
                printf ("\tlargeur de segment : %.1f\n",mesure);
                printf ("\torientation par rapport cap robot (orthogonale) : %.0f\n",angle);
            }
            i += largeur;
            largeur = ecart;
        }
        else
        {
            i++;
        }
    }
}

void detectpath (double x[], double y[], double points[], int taille, int ecart, int seuil)
{
    // taille : des tableaux (180 en principe)
    // largeur : largeur du passage en cm
    // seuil : distance minimale disponible en cm
    
    // a voir quels parametres on retourne, et comment...
    // pour le moment on les imprime
    
    // détecte un passage de lareur minimale, avec une profondeur minimale.
    // on pourra boucler sur cette fonction pour tester différentes distances par exemple
    
    int i = 0;
    int start, med;
    double angle;
    double depth;
    double mesure, alpha;
    double a, z;
    
    angle = (float) round ( (asin ((ecart / 2.0) / (seuil * 1.0)) * 2) / PIsur180);
printf ("values %d %d, %.1f\n", ecart, seuil, asin ((ecart / 2.0) / (seuil * 1.0)));
printf ("Angle de recherche %.1f\n", angle);

    // en fonction de la largeur de passage souhaitée et de la distanc eminimale disponible :
    // nombre de points devant être au dela du seuil.
    
    while (i < (taille - (int) angle))
    {
        // chercher le premier point au dela du seuil
        while ((i < (taille - (int) angle)) && (points[i] < seuil))
            i++;
        start = i;
        depth = 0;
printf ("début de passage : %d ?\n", start);
        
        // chercher le dernier point
        while ((i < taille) && (points[i] >= seuil))
        {
            depth += points[i];
            i++;
        }
        
printf ("fin de passage : %d ?\n", i);
        if ((i - start) > (int) angle)
        {
            // on a trouvé !
            i = i-1;
            depth = depth / (i - start);
            med = (i + start) / 2;
            printf ("Found path of width %d at angle %d, average depth %.0f cm\n", (i - start), med, depth);
            printf ("\tFrom x=%.0f y=%.0f to x=%.0f y=%.0f\n",x[start],y[start],x[i],y[i]);
            
            mesure = calculelargeur (x[start], x[i], y[start], y[i]);
            a = (y[i] - y[start]) / (x[i] - x[start]);
            alpha = - atan(a) / PIsur180; // angle à ajouter au cap actuel pour faire face au segment.
            z = calculelargeur ((x[i] + x[start]) / 2.0 , 0, (y[i] + y[start]) / 2.0 , 0);
            
            printf ("segment normalisé : \n");
            printf ("\tangle median de detection : %d\n",med);
            printf ("\tdistance point median : %.1f\n",z);
            printf ("\tlargeur de segment : %.1f\n",mesure);
            printf ("\torientation par rapport cap robot (orthogonale) : %.0f\n",alpha);
            

        }
    }
}

float distance_matrix (struct wifiref *CurrentWifi, struct wifiref *RefWifi)
{
    int i,j,k;
    float dist = 0.0;
    float somme = 0.0;
    float val;
    
    for (i=0; i<4; i++)
    {
        for (j=0; j<10; j++)
        {
            val = 99.0 * 99.0;
            if (strlen ((*CurrentWifi).ssid[i][j].label) > 0)
            {
                for (k=0; k<10; k++)
                {
                    if (!strcmp ((*CurrentWifi).ssid[i][j].label, (*RefWifi).ssid[i][k].label))
                    {
                        val = ((*CurrentWifi).ssid[i][j].val - (*RefWifi).ssid[i][k].val) *
                                ((*CurrentWifi).ssid[i][j].val - (*RefWifi).ssid[i][k].val) * 1.0;
                        break;
                    }
                }
            }
            somme += val;
        }
    }
    
    dist = sqrtf (somme);
    return (dist);
}

void oriente_nord (double points[], int taille, int orientation, double xnorm[], double ynorm[])
{
    // points contains the detected distances left (-90°) to right (+90°)
    // knowing that the robot heads to "orientation", calculate x & y coordinates north oriented
    
    int i;
    int degre;
    float angle;
    
    for (i=0; i<taille; i++)
    {
        degre = i + orientation;
        if (degre >= 360)
            degre -= 360;
        angle = degre * PIsur180;
        
        xnorm[i] = - (points[i] * cos (angle));
        ynorm[i] = points[i] * sin (angle);
     }
}

double find_best_match (double cap, double spread, double step, double mesures[], int taille, int *minx, int *miny, double *minAngle, double minNormX[], double minNormY[], unsigned char piece) // finds a best map matching allowing a tolerance on the robot orientation
{
    double normX[180];
    double normY[180];
    int posx, posy, i;
    double val, minval;
    double angle;
    
    
    minval = 99999;
                          
    for (angle = cap - spread; angle <= cap + spread; angle += step)
    {
        if (DebugMode == 1)
            printf("orientation : %.0f°\n", angle);
                                                  
            oriente_nord (mesures, 180, angle, normX, normY);
            val = map_match (normX, normY, 180, piece, &posx, &posy); // finds the best match for this orientation
                              
            if (val < minval)
            {
                // this match is better than the ones before
                if (DebugMode == 1)
                    printf("best match found %.0f°\n", angle);
                                  
                minval = val;
                *minx = posx;
                *miny = posy;
                *minAngle = angle;      // may be used to ajust the compas value ?
                                  
                for (i=0; i<180; i++)
                {
                    minNormX[i] = normX[i];
                    minNormY[i] = normY[i];
                }
            }
    }

    return minval;
}

double map_match (double x[], double y[], int taille, unsigned char piece, int *posx, int *posy)
{
    // points have been oriented north
    // checks only in the zone matching piece - FF checks the whole appartment
    // returns position x, y and the match value (the lower the closer)
    
    int i, j;
    int xd, yd, p;
    double distance = 20000.0 * 180; /// max value for a whole scan
    double value;
    
//    if (DebugMode == 1)
//        for (p=0; p<taille; p++)
//            printf ("point %.0f,%.0f\n", x[p], y[p]);
    
    for (i=0; i<APPT_L; i++)
    {
//        if (DebugMode == 1)
//            printf ("testing line %d\n", i);
        
        for (j=0; j<APPT_W; j++)
        {
            if ((piece == 0xFF) || (piece == Appartement[i][j].piece))
            {
                // this is an appartment cell againts which we test the points pattern
                // calculate the distance from a wall of each point
                
                value = 0;
                for (p=0; p<taille; p++)
                {
                    xd = j + (int) round(x[p] / 10);  // attention les axes ne sont pas dans le meme sens
                    yd = i - (int) round(y[p] / 10);
                    
                    if ((xd >= -BORDER) && (xd < APPT_W+BORDER) &&
                            (yd >= -BORDER) && (yd < APPT_L+BORDER))
                    {
                        // point is within the appt_distance range
                        //value += ((ApptDistances[xd + BORDER][yd + BORDER]) * (ApptDistances[xd + BORDER][yd + BORDER]));
                        value += ((ApptDistances[yd + BORDER][xd + BORDER]) * (ApptDistances[yd + BORDER][xd + BORDER]));
                    }
                    else
                        value += 20000.0;
                }
            
            
//            if (DebugMode == 1)
//                printf ("Found %.0f at %d,%d\n", value,i,j);
            
                if (value < distance)
                {
                    distance = value;
                
                    *posx = j;
                    *posy = i;
                }
            }
        }
    }
    
    return (sqrt(distance));

}

int find_path_to (int x[], int y[], int TailleMax, int CurX, int CurY, int ToX, int ToY)
{
    int PathLength = 0; // should not exceed TailleMax (truncate ?)
    
    if (Appartement[ToX][ToY].piece == 0x0F)    // can't get into a wall !
        return -1;
    
    PathLength = get_path (x, y, TailleMax, CurX, CurY, ToX, ToY);
    
    if (DebugMode ==1)
        printf ("Raw path length : %d\n", PathLength);
    
    return PathLength;
}


int lee_expansion (int ApptPath[][APPT_W], int level, int CurX, int CurY, int ToX, int ToY)
{
    int i,j;
    int k,l;
    int found = 0;
    
    // check all the appt cells. if they are close to level then mark them as level +1
    // for efficiency limit the search to a square limited to "level" distance around the dest point
    
    for (i = ToY-level; i <= ToY+level; i++)
        for (j = ToX-level; j <= ToX+level; j++)
            if ((i >= 0) && (i < APPT_L) && (j >= 0) && (j < APPT_W)) // stay within boundaries
                if (ApptPath[i][j] == 0)                // no wall and not yet tagged
                {
                    // then search a cell of value "level" in the boundaries
                    for (k = i-1; k <= i+1; k++)
                        for (l = j-1; l <= j+1; l++)
                            if ((k >= 0) && (k < APPT_L) && (l >= 0) && (l < APPT_W))
                                if (ApptPath[k][l] == level)
                                {
                                    ApptPath[i][j] = level + 1;
                                    if ((i == CurY) && (j == CurX))
                                        found = 1;
                                }
                }
    
    return found;
}

int get_path (int x[], int y[], int TailleMax, int CurX, int CurY, int ToX, int ToY)
{
    extern float ApptDistances [APPT_L+BORDER*2][APPT_W+BORDER*2];
    int ApptPath [APPT_L][APPT_W];
    int i,j;
    int k;
    int level=1;
    int Tolerance = 30; // distance under which we should not go, close to a wall
FILE *fd;
    
    // initiate ApptPath by blacking walls + cells too close (< Tolerance)
    
    for (i=0; i<APPT_L; i++)
        for (j=0; j<APPT_W; j++)
            if (ApptDistances[i+BORDER][j+BORDER] < 20)
                ApptPath[i][j] = -1;
            else
                ApptPath[i][j] = 0;
    
    ApptPath[ToY][ToX] = 1; // we'll start the expansion with the detination point
    while (lee_expansion (ApptPath, level, CurX, CurY, ToX, ToY) == 0)
        level++;
    
    
    // si level > 300 : problème de longeur !
    // opportunité de faire un malloc ?
    
    
/*
    //debug
    printf ("writing lee expanded file\n");
    fd = fopen ("/home/olivier/projets/ardpilot/lee_test", "w");
    if (fd != NULL)
    {
        for (i=0; i<(APPT_L); i++)
        {
            for (j=0; j<(APPT_W); j++)
            {
                fprintf (fd, "%d ", ApptPath[i][j]);
            }
            fprintf (fd, "\n");
        }
        fclose (fd);
    }
*/
    
    // now trace back the proper path
    
    k = 0;
    
    i = CurY;
    j = CurX;
    
    while (level >= 0)
    {
        if (ApptPath[i-1][j] == level)
        {
            x[k]=j;
            y[k]=i-1;
        } else if (ApptPath[i+1][j] == level)
        {
            x[k]=j;
            y[k]=i+1;
        } else if (ApptPath[i][j-1] == level)
        {
            x[k]=j-1;
            y[k]=i;
        } else if (ApptPath[i][j+1] == level)
        {
            x[k]=j+1;
            y[k]=i;
        } else if (ApptPath[i-1][j-1] == level)
        {
            x[k]=j-1;
            y[k]=i-1;
        } else if (ApptPath[i+1][j+1] == level)
        {
            x[k]=j+1;
            y[k]=i+1;
        } else if (ApptPath[i+1][j-1] == level)
        {
            x[k]=j-1;
            y[k]=i+1;
        } else if (ApptPath[i-1][j+1] == level)
        {
            x[k]=j+1;
            y[k]=i-1;
        }
        
        i=y[k];
        j=x[k];
        level--;
        k++;
    }
        
/* debug
 
    for (level=0;level<=k;level++)
        printf ("step %d - %d,%d\n", level, x[level], y[level]);
 
*/
    return k; // path length
        
}

