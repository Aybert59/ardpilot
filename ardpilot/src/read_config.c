#define _GNU_SOURCE

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ardpilot_grammar.h"
#include "ardpilot.h"

extern int ardfd;

#define CONFDIR "/home/olivier/projets/ardpilot"
#define CONFIG_FILE "ardpilot.conf"
#define COMPAS_FILE "compas2.csv"
#define PLAN_FILE "plan.csv"
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


struct cellule Appartement [193][93];
struct wifiref WifiMatrix[64]; // avec 64 matrices on va s'en sortir ...
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
    if (fd != NULL)
    {
        while ((read = getline(&line, &len, fd)) > 0)
        {
         
            // le fichier doit contenir une ligne par valeur de chaque coté
            sscanf (line, "%d;%d", &angleArd, &angleReel);
            if ((angleArd >= 0) && (angleArd < 360) && (angleReel >= 0) && (angleReel < 360))
            {
                compas_correction[angleArd] = angleReel;
                cap_correction[angleReel] = angleArd;
            }
        }
        free (line);
        fclose (fd);
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
             
                if (!strncmp(line, "SRV1", 4))
                {
                    message[0] = C_PARM;
                    message[1] = F_SRV1;
                    sscanf (line, "%*s %s", &(message[2]));
                    write_ard (ardfd, message);
                }
                else if (!strncmp(line, "DMIN", 4))
                {
                    message[0] = C_PARM;
                    message[1] = F_DMIN;
                    sscanf (line, "%*s %s", &(message[2]));
                    write_ard (ardfd, message);
                }
                else if (!strncmp(line, "AJUST", 5))
                {
                    message[0] = C_PARM;
                    message[1] = F_AJUST;
                    sscanf (line, "%*s %s", &(message[2]));
                    write_ard (ardfd, message);
                }
                else if (!strncmp(line, "ALIGN", 5))
                {
                    message[0] = C_PARM;
                    message[1] = F_ALIGN;
                    sscanf (line, "%*s %s", &(message[2]));
                    write_ard (ardfd, message);
                }
                else if (!strncmp(line, "PNGNUM", 6))
                {
                    message[0] = C_PARM;
                    message[1] = F_PNGNUM;
                    sscanf (line, "%*s %s", &(message[2]));
                    write_ard (ardfd, message);
                }
                else if (!strncmp(line, "COVSEUIL", 8))
                {
                    sscanf (line, "%*s %f", &CovSeuil);
                    printf ("Param CovSeuil %f\n", CovSeuil);
                }
                else if (!strncmp(line, "LINEW", 5))
                {
                    sscanf (line, "%*s %d", &LineW);
                    printf ("Param LineW %d\n", LineW);
                }
                else if (!strncmp(line, "PATHW", 5))
                {
                    sscanf (line, "%*s %d", &PathW);
                    printf ("Param PathW %d\n", PathW);
                }
                else if (!strncmp(line, "PATHD", 5))
                {
                    sscanf (line, "%*s %d", &PathD);
                    printf ("Param PathD %d\n", PathD);
                }
            }
        }
        free (line);
        fclose (fd);
    } else {
        printf ("Config file unreadable !\n");
    }
    
    read_compas_correction();
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
        if (lines == 193)
            printf ("Read %d lines in plan : OK\n", lines);
        else
            printf ("WARNING read %d lines in plan : NOT OK\n", lines);
    } else {
        printf ("Plan file unreadable !\n");
    }
    
/* debug
    for (i=0;i<193;i++)
    {
        for (j=0;j<93;j++)
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
        for (lines=0; lines<193; lines++)
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
                    sscanf (line, "%*s %c%c", &(WifiMatrix[rang].zone[0]), &(WifiMatrix[rang].zone[1]));
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
            fprintf (fd, "Zone %c%c\n", WifiMatrix[lines].zone[0], WifiMatrix[lines].zone[1]);
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
