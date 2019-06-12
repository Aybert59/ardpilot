#define _GNU_SOURCE

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "ardpilot_grammar.h"
#include "ardpilot.h"

extern int ardfd;

#define CONFDIR "/home/olivier/projets/ardpilot"
#define CONFIG_FILE "ardpilot.conf"
#define COMPAS_FILE "compas2.csv"
#define PLAN_FILE "plan.csv"
#define DIST_FILE "plan_distances.csv"
#define MATRIX_FILE "matrices_wifi.txt"

int compas_correction[360]; // retourne cap affiché en fonction cap du robot - 0 = boulevard, 180 = jardin
int cap_correction[360];    // retourne cap du robot en fonction cap affiché - 0 = Nord

// cf fichier de référence XLS (sur OneDrive). chaque case correspond à un pas de 5 dans la commande de vitesse, le dernier (25e = 120)
float carto_vitesse[] = {0.00,
    2.77,
    11.40,
    23.27,
    34.00,
    44.67,
    61.53,
    66.67,
    76.50,
    86.83,
    95.30,
    104.90,
    110.50,
    120.30,
    126.00,
    131.00,
    135.00,
    139.20,
    142.33,
    144.67,
    146.67,
    148.53,
    149.93,
    151.20,
    152.87};


struct cellule Appartement [APPT_L][APPT_W];
float ApptDistances [APPT_L+BORDER*2][APPT_W+BORDER*2]; // appartment size + 1 meter all around
struct wifiref WifiMatrix[64]; // 64 matrcies should be enough ...
int MaxMatrices;

float CovSeuil = 50.0;
int LineW = 30;
int PathW = 25;
int PathD = 80;

void read_compas_correction()
{
    FILE *fd;
    char filename[256];
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    int angleArd, angleReel;
    int i;
    char buffer[4];
    
    // initialisation
    for (i=0; i<360; i++)
    {
        compas_correction[i] = i;
    }
    
    strcpy (filename, CONFDIR);
    strcat (filename, "/");
    strcat (filename, COMPAS_FILE);
    fd = fopen (filename, "r");
    i=0;
    if (fd != NULL)
    {
        while ((read = getline(&line, &len, fd)) > 0)
        {
         
            // le fichier doit contenir une ligne par valeur de chaque coté
            sscanf (line, "%d;%d", &angleArd, &angleReel);
            if ((angleArd >= 0) && (angleArd < 360) && (angleReel >= 0) && (angleReel < 360))
            {
                i++;
                compas_correction[angleArd] = angleReel;
                cap_correction[angleReel] = angleArd;
            }
        }
        free (line);
        fclose (fd);
        printf ("Read %d lines from Compas adjustment file\n",i);
    } else {
        printf ("Compas adjustment file unreadable !\n");
    }
    
    buffer[0] = C_CMP;
    buffer[1] = '\0';
    write_ard (ardfd, buffer);

}


void read_and_send_config ()
{
    FILE *fd;
    char filename[256];
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    char message[32];
    static int first_time = 1;
    int count = 0;
    
    if (first_time == 1)
    {
        sleep (2);  // attendre que le robot soit pret a lire
        first_time = 0;
    }
    
    strcpy (filename, CONFDIR);
    strcat (filename, "/");
    strcat (filename, CONFIG_FILE);
    fd = fopen (filename, "r");
    if (fd != NULL)
    {
        while ((read = getline(&line, &len, fd)) != -1) {
            if ((read > 1) && (line[0] != '#'))
            {
                message[0] = C_PARM;
                message[1] = '\0';
                sscanf (line, "%*s %s", &(message[2]));
                count++;
                
                if (!strncmp(line, "SRV1", 4))
                    message[1] = F_SRV1;
                else if (!strncmp(line, "DEBUG", 5))
                    message[1] = F_DEBUG;
                else if (!strncmp(line, "DMIN", 4))
                    message[1] = F_DMIN;
                else if (!strncmp(line, "AJUST", 5))
                    message[1] = F_AJUST;
                else if (!strncmp(line, "ALIGN", 5))
                    message[1] = F_ALIGN;
                else if (!strncmp(line, "PNGNUM", 6))
                    message[1] = F_PNGNUM;
                else if (!strncmp(line, "COVSEUIL", 8))
                    sscanf (line, "%*s %f", &CovSeuil);
                else if (!strncmp(line, "LINEW", 5))
                    sscanf (line, "%*s %d", &LineW);
                else if (!strncmp(line, "PATHW", 5))
                    sscanf (line, "%*s %d", &PathW);
                else if (!strncmp(line, "PATHD", 5))
                    sscanf (line, "%*s %d", &PathD);
                
                if (message[1] != '\0')
                    write_ard (ardfd, message);
            }
        }
        free (line);
        fclose (fd);
        printf ("Read %d parameters in Config file\n", count);
    } else {
        printf ("Config file unreadable !\n");
    }
    
    read_compas_correction();
}

void init_appt_distances ()
{
    int i,j;
    int x,y;
    int c;
    float d;
    FILE *fd;
    char filename[256];

    
    // initiates a table ApptDistances which represents the appartment map plus 1 meter all around
    // each cell contains the distance in cm to the nearest wall
    // to be used by the map matching algorithm
 
    ///////////////////////////////////////////////////////////////////////////////////
    // en fait vérifier si ix un fichier distances plus récent que fichier appart.
    // si oui juste le lire ; si non le créer
    ///////////////////////////////////////////////////////////////////////////////////
    
    printf ("Entering init_appt_distances\n");
    
    strcpy (filename, CONFDIR);
    strcat (filename, "/");
    strcat (filename, DIST_FILE);
    fd = fopen (filename, "r");
    if (fd != NULL)
    {
        for (i=0; i<(APPT_L+BORDER*2); i++)
        {
            for (j=0; j<(APPT_W+BORDER*2); j++)
            {
                fscanf (fd, "%f ", &(ApptDistances[i][j]));
            }
        }
        fclose (fd);
    }
    else // file not found, creating one
    {
    
        printf ("no file found, initializing\n");
    
        for (i=0; i<(APPT_L+BORDER*2); i++)
            for (j=0; j<(APPT_W+BORDER*2); j++)
            {

                ApptDistances[i][j] = 300.0;
                for (c=0;c<20;c++) // search for a wall within a square of C size (unit = 10 cm)
                    // start small and grow until we reach a wall
                {
                    for (x=i-c; x<=i+c; x++)
                        for (y=j-c; y<=j+c; y++)
                        {
                            if ((x >= BORDER) && (x < APPT_L+BORDER) && (y >= BORDER) && (y < APPT_W+BORDER))
                                // we are within the appartment boundaries
                            {
                                if (Appartement[x-BORDER][y-BORDER].piece == 0x0F)
                                    // we found a wall
                                {
                                    d = sqrtf ((float)((i-x)*(i-x) + (j-y)*(j-y))) * 10.0; // one unit for 10cm
                                    if (d < ApptDistances[i][j])
                                        ApptDistances[i][j] = d;
                                }
                            }
                        }
                }
            }
        
        printf ("writing distance file\n");
        fd = fopen (filename, "w");
        if (fd != NULL)
        {
            for (i=0; i<(APPT_L+BORDER*2); i++)
            {
                for (j=0; j<(APPT_W+BORDER*2); j++)
                {
                    fprintf (fd, "%f ", ApptDistances[i][j]);
                }
                fprintf (fd, "\n");
            }
            fclose (fd);
        }
    }
    
    
    
    printf ("exiting init_appt_distances\n");
/*
    // print 20 first lines as debug
    for (i=0; i<(APPT_L+BORDER*2); i++)
    {
        printf("**** ");
        for (j=0; j<(APPT_W+BORDER*2); j++)
        {
            printf("%.0f ", ApptDistances[i][j]);
        }
        printf("\n");
    }
 */
}

void read_plan()
{
    FILE *fd;
    char filename[256];
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    int lines;
    int cellule;
    int start, end;
    int piece, zone;
    int wifi_ref;
    int i,j;
    
    // initialisation
    lines=0;
    
    // format du fichier
    // une case = 10*10cm
    // dans chaque case une séquence de valeurs séparées par des '-'
    // 1. (1 char) décrit la pièce, '*' si c'est un mur
    // 2. (1 char) décrit la zone dans la pièce
    // obsolete 3. (id sur plusieurs char) nom de la matrice WIFI de référence, ou 0 si pas de matrice de référence à cet endroit
    
    strcpy (filename, CONFDIR);
    strcat (filename, "/");
    strcat (filename, PLAN_FILE);
    fd = fopen (filename, "r");
    if (fd != NULL)
    {
        while ((read = getline(&line, &len, fd)) != -1)
        {
            start = 0;
            end = 0;
            cellule = 0;
            
            while (end < read)
            {
                while ((line[end] != ';') && (end < read) && (line[end] != '\n') && (line[end] != '\r'))
                    end++;
                
                // on a un token entre start et end
                
                switch (end - start) {
                    case 0:
                        // cellule vide
                        Appartement[lines][cellule].piece = 0x00;
                        break;
                        
                    case 1:
                        if (line[start] == '*')
                        {
                            Appartement[lines][cellule].piece = 0x0F;
                        }
                        else
                            printf ("bad cell %d of size 1 at line %d\n", cellule, lines);
                        break;
                        
                    default:

                        sscanf (&(line[start]), "%x-%x-%d", &piece, &zone, &wifi_ref);
                        Appartement[lines][cellule].piece = (unsigned char) (piece * 16 + zone);

                        break;
                }
                
                start = end + 1;
                end = start;
                cellule++;
            }
            lines++;
        }
        free (line);
        fclose (fd);
        if (lines == APPT_L)
            printf ("Read %d lines in plan : OK\n", lines);
        else
            printf ("WARNING read %d lines in plan : NOT OK\n", lines);
    } else {
        printf ("Plan file unreadable !\n");
    }
    
    //init_appt_distances ();
    
/* debug
    for (i=0;i<APPT_L;i++)
    {
        for (j=0;j<APPT_W;j++)
        {
            if (Appartement[i][j].piece == 0x00)
                printf(";");
            else if (Appartement[i][j].piece == 0x0F)
                printf("*;");
            else
            {
                printf("%X-%d;", Appartement[i][j].piece, Appartement[i][j].WifiMatrix);
            }

        }
        printf ("\n");
    }
    printf ("\n");

*/
}

void write_plan()
{
    FILE *fd;
    char filename[256];
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    int lines;
    
    // initialisation
    
    strcpy (filename, CONFDIR);
    strcat (filename, "/new-");
    strcat (filename, PLAN_FILE);
    fd = fopen (filename, "w");
    if (fd != NULL)
    {
        for (lines=0; lines<APPT_L; lines++)
        {
// WIP
            printf ("Write plan not yet available\n");
        }
        fclose (fd);
    } else {
        printf ("New Plan file unwriteable !\n");
    }
}

void read_wifi_matrixes()
{
    FILE *fd;
    char filename[256];
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    int lines;
    int rang = -1;
    int i,j;
    int val;
    char label[32];
    
    // initialisation
    lines=0;
    MaxMatrices = 0;
    
    strcpy (filename, CONFDIR);
    strcat (filename, "/");
    strcat (filename, MATRIX_FILE);
    fd = fopen (filename, "r");
    
    if (fd != NULL)
    {
        while ((read = getline(&line, &len, fd)) != -1)
        {
            if (!strncmp (line, "Matrix", 6) ) // the first 6 chars are equal to 'Matrix'
            {
                lines++;
                sscanf (line, "%*s %d", &rang);
                
                WifiMatrix[rang].i = 0; // non renseigné pour le moment
                WifiMatrix[rang].j = 0;
            } else if (!strncmp (line, "Zone", 4) )
            {
                if (rang >= 0)
                    sscanf (line, "%*s %x", &(WifiMatrix[rang].zone));
            } else if (!strncmp (line, "Name", 4) )
            {
                if (rang >= 0)
                    sscanf (line, "%*s %s", WifiMatrix[rang].name);
            }  else if (!strncmp (line, "Releves", 7) )
            {
                if (rang >= 0)
                    sscanf (line, "%*s %d", &(WifiMatrix[rang].Releves));
            } else if (!strncmp (line, "ssid", 4) )
            {
                if (rang >= 0)
                {
                    strcpy (label, "");
                    sscanf (line, "%*s %d %d %d %s", &i, &j, &val, label);
                    WifiMatrix[rang].ssid[i][j].val = val;
                    strcpy (WifiMatrix[rang].ssid[i][j].label, label);
                }
            }
            
        }
        free (line);
        fclose (fd);

        MaxMatrices = lines;
        printf ("Uploaded %d WIFI matrixes\n", lines);
        
// vérification
/*
        for (lines=0; lines<MaxMatrices; lines++)
        {
            printf ("***** Matrix %d, zone %c%c, %s *****\n", lines, WifiMatrix[lines].zone[0], WifiMatrix[lines].zone[1], WifiMatrix[lines].name);
            for (i=0; i<4; i++)
                for (j=0; j<10; j++)
                {
                    printf ("ssid %d %d %d %s\n", i, j, WifiMatrix[lines].ssid[i][j].val, WifiMatrix[lines].ssid[i][j].label);
                }
        }
  */
    } else {
        printf ("WIFI matrix file unreadable !\n");
    }
}

void write_wifi_matrixes()
{
    FILE *fd;
    char filename[256];
    int lines;
    int rang = -1;
    int i,j;
    int val;
    char label[32];
    
    // initialisation
    lines=0;
    
    strcpy (filename, CONFDIR);
    strcat (filename, "/");
    strcat (filename, MATRIX_FILE);
    fd = fopen (filename, "w");
    
 
    if (fd != NULL)
    {
        // file header
        
        fprintf (fd, "# matrices de WIFI référence\n");
        fprintf (fd, "#\n");
        fprintf (fd, "# chaque matrice contient une mesure de référence des 10 hotspots les plus visibles\n");
        fprintf (fd, "# mesurées dans chacune des 4 directions cardinales \n");
        fprintf (fd, "#\n");
        fprintf (fd, "# pour la zone dans le plan cf le plan excel et le format dans read_config.c\n");
        fprintf (fd, "#\n");
        fprintf (fd, "# Maximum 64 matrices\n");
        fprintf (fd, "#\n");
        fprintf (fd, "###################################################################################\n");
        fprintf (fd, "\n\n\n");
        
        for (lines=0; lines<MaxMatrices; lines++)
        {
            fprintf (fd, "#\n");
            fprintf (fd, "Matrix %d\n", lines);
            fprintf (fd, "Zone %2X\n", WifiMatrix[lines].zone);
            fprintf (fd, "Name %s\n", WifiMatrix[lines].name);
            fprintf (fd, "Releves %d\n", WifiMatrix[lines].Releves);
            for (i=0; i<4; i++)
                for (j=0; j<10; j++)
                {
                    fprintf (fd, "ssid %d %d %d %s\n", i, j, WifiMatrix[lines].ssid[i][j].val, WifiMatrix[lines].ssid[i][j].label);
                }
            fprintf (fd, "\n");
        }
        
        fclose (fd);
        
        printf ("Written %d WIFI matrixes\n", MaxMatrices);
    } else {
        printf ("WIFI matrix file unwriteable !\n");
    }
}

FILE *open_wifi_matrix_file (int MatRef)
{
    FILE *fd;
    char filename[256];
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    int lines = 0;
    int matrixNumber;
    int i,j;
    
    strcpy (filename, CONFDIR);
    strcat (filename, "/");
    strcat (filename, MATRIX_FILE);
    
    // initialisation compter le nombre de matrices existantes
    
    fd = fopen (filename, "a+"); // pointer read en début de fichier, write à la fin
    if (fd != NULL)
    {
        while ((read = getline(&line, &len, fd)) != -1)
        {
            if ((read > 1) && (line[0] != '#'))
            {
                if (!strncmp (line, "Matrix", 6) ) // the first 6 chars are equal to 'Matrix'
                {
                    lines++;
                }
            }
        }
        
        free (line);
        printf ("Found %d WIFI matrixes\n", lines);
        
        
        if (MatRef == -1) // request to create a new Matrix
        {
            matrixNumber = lines + 1;
        
            fprintf (fd, "\n");
            fprintf (fd, "# insert any comment here\n");
            fprintf (fd, "Matrix %d\n", matrixNumber);
            fprintf (fd, "Zone ##\n");
            fprintf (fd, "Name xxxx\n");
            fprintf (fd, "Releves 1\n");
        }
        return (fd);
        
    } else {
        printf ("WIFI matrix file unreadable !\n");
        return (NULL);
    }
}

void close_wifi_matrix_file (FILE *fd)
{
    fprintf (fd, "\n");
    fclose (fd);
}
