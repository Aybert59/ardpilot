#include <stdio.h>
#include <string.h>
#include <math.h>

#include "ardpilot_grammar.h"
#include "ardpilot.h"
#define PIsur180 0.0174532925

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
    
    // a noter recherche covariance proche de 1 ne marche pas bien
    // utilisons covariance proche de zero
    
    // a voir quels parametres on retourne, et comment...
    // pour le moment on les imprime
    
    int i = 0;
    int largeur = ecart;
    double c;
    double a, b;
    int med;
    double angle;
    double mesure;

    while (i+largeur < taille)
    {
        c = fabsf (covariance (&(x[i]), &(y[i]), largeur));
        if (c < precision)
        {
            while ((c < precision) && ((i + largeur) < taille))
            {
                largeur++;
                c = fabsf (covariance (&(x[i]), &(y[i]), largeur));
            }
            largeur--;
            c = fabsf (covariance (&(x[i]), &(y[i]), largeur));
            RegLineaire (&(x[i]), &(y[i]), largeur, &a, &b);
            printf ("found segment (%f) starting at %d, angle width %d\n", c, i, largeur);
            med = i + largeur/2;
            printf ("\ty = %f*x + %f - median angle %d\n",a,b,med);
            printf ("\tFrom x=%.0f y=%.0f to x=%.0f y=%.0f\n",x[i],y[i],x[i+largeur],y[i+largeur]);

            // affichage normalisé
            angle = - atan(a) / PIsur180; // angle à ajouter au cap actuel pour faire face au segment.
            mesure = calculelargeur (x[i], x[i+largeur], y[i], y[i+largeur]);
            
            printf ("segment normalisé : \n");
            printf ("\tangle median de detection : %d\n",med);
            printf ("\tdistance point median : %.1f\n",points[med]);
            printf ("\tlargeur de segment : %.1f\n",mesure);
            printf ("\torientation par rapport cap robot (orthogonale) : %.0f\n",angle);
            
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
