#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>

#include "ardpilot.h"
#include "ardpilot_grammar.h"

extern int NoInterrupt;
int TopWIFICount = 0;

float distance_matrix (struct wifiref *CurrentWifi, struct wifiref *RefWifi);
extern unsigned char Sequence[];



void get_top_wifi (char sequence)
{
    extern int ardfd;
    char message[4];
    
    if (TopWIFICount == 0) // si déjà en cours on ne fait rien :-)
    {
        message[0] = C_TOPWIFI;
        message[1] = sequence;
        message[2] = '\0';
        write_ard (ardfd, message);
    }
}

int analyze_environment ()
{
    int n;

    // récupérer vitesse des moteurs
    
    // récupérer WIFI
    
    // récupérer orientation boussole
    
    // mesurer sol : plat devant ? plat à 20° ? obstacles à dr ou à gauche / ou trous ?
    // demander une mesure de ligne et analyser ici
    
    // mesurer hsp

    return 0;
}

void check_free_mem (unsigned char sequence)
{
    char message[4];
    extern int ardfd;
    
    message[0] = C_MEM;
    message[1] = sequence;
    message[2] = '\0';
    write_ard (ardfd, message);
}

void block_check_free_mem (unsigned char sequence)
{
    char message[4];
    extern int ardfd;
    extern int DebugMode;
    char buffer[64];
    int n;
    
    ard_block_mode ();
    check_free_mem (sequence);
    
    buffer[0] = '\0';
    while (buffer[0] != C_MEM)
    {
        n = read(ardfd, buffer, 64);
        if (n > 0)
        {
            interpret_ard (buffer, DebugMode);
        }
    }
    ard_async_mode ();
}

void check_compas (unsigned char sequence)
{
    char message[4];
    extern int ardfd;
    
    message[0] = C_CMP;
    message[1] = sequence;
    message[2] = '\0';
    write_ard (ardfd, message);
}

void check_voltage ()
{
    char message[4];
    extern int ardfd;
    
    message[0] = C_BAT;
    message[1] = '\0';
    write_ard (ardfd, message);
}

int get_health ()
{
    int n; // une fois que le ping récupère tout, juste envoyer un ping.
    // récupérer adresse IP
    
    // récuperer free mem
    
    check_free_mem ('*');
    pause();
    
    // ne mache pas : block_check_free_mem ('*');
 
    sleep (1);
    // récupérer vitesse des moteurs
    
    // récupérer charge batterie
    check_voltage ();
    pause();
    
    // récupérer orientation boussole
    
    // mesurer durée boucle vide / et boucle moyenne
    
     return 0;
}

int mesure_wifi (int MatRef)
{
    int i,j, n, p;
    int k, val;
    int found;
    int codeRet = 0;
    extern int ardfd;
    char buffer[64];
    char label[32];
    unsigned char len;
    FILE *fd;
    extern struct wifiref WifiMatrix[];
    extern int MaxMatrices;
 
// not yet implemented
printf ("Matrix to update %d\n", MatRef);
    
    NoInterrupt = 1;
    fd = open_wifi_matrix_file (MatRef);
    if (fd == NULL)
        return (0);
 
    // change ARDFD en mode bloquant, sans envoyer de SIGIO
    ard_block_mode ();
    
    control_message(MSG_WARNING, "\nBug #1 : may return empty result : run \"TopWifi\" once first\n");
    control_message(MSG_WARNING, "Bug #2 : if blocked if the middle of the process click on the compass to release\n\n");
    

    for (i=0; i<4; i++)
    {
        codeRet = oriente_robot (i*90, -1);
        if (codeRet == 0)
        {
            printf("Problème d'orientation\n");
            break;
        }
        
        for (k=0; k< 10; k++)
        {
            WifiMatrix[MatRef].ssid[i][k].updated = 0;
        }
            
            
        get_top_wifi (Sequence[C_TOPWIFI]);

        for (j=0; j< 10; j++)
        {
            buffer[0] = ACTION_NULL;
            while (buffer[0] != C_TOPWIFI)
            {
                n = read(ardfd, &len, 1);
                if (n > 0)
                {
                    for (p=0; p<len; p++)
                        read (ardfd, &(buffer[p]), 1);
                    buffer[p] = '\0';

                    if (buffer[0] == C_TOPWIFI)
                    {
                        printf ("received %d %s\n",j,&(buffer[2]));
                        if (MatRef >= 0)
                        {
                            // rechercher un ssid dans la matrice dans l'orientation i
                            // si trouvé : faire moyenne pondérée par "Releves"
                            // sinon s'il existe valeur à 99 (un slot libre) faire moyenne pondérée par "Releves"
                            
                            sscanf (&(buffer[2]), "%d %s", &val, label);
                            found = 0;
                            
                            for (k=0; k< 10; k++)
                            {
                                if (!strcmp (WifiMatrix[MatRef].ssid[i][k].label, label) )
                                {
                                    WifiMatrix[MatRef].ssid[i][k].val =
                                        ((WifiMatrix[MatRef].ssid[i][k].val * WifiMatrix[MatRef].Releves) + val) /
                                            (WifiMatrix[MatRef].Releves + 1);
                                    WifiMatrix[MatRef].ssid[i][k].updated = 1;
                                    found = 1;
                                    break;
                                }
                            }
                            if (found == 0)
                            {
                                for (k=0; k< 10; k++)
                                {
                                    if (WifiMatrix[MatRef].ssid[i][k].val == 99)
                                    {
                                        WifiMatrix[MatRef].ssid[i][k].val =
                                            ((99 * WifiMatrix[MatRef].Releves) + val) /
                                            (WifiMatrix[MatRef].Releves + 1);
                                        strcpy (WifiMatrix[MatRef].ssid[i][k].label, label);
                                        WifiMatrix[MatRef].ssid[i][k].updated = 1;
                                        break;
                                    }
                                }
                            } // si j = 10 j'ai recu toutes les mesures. rechercher les ssid non mis à jour et faire moyenne à 99
                        } else {
                            fprintf (fd, "ssid %d %d %s\n",i,j,&(buffer[2]));
                        }
                    } else {
                        interpret_ard (buffer, 1);
                    }
                }
            }
        }
        
        for (k=0; k< 10; k++)
        {
            if (WifiMatrix[MatRef].ssid[i][k].updated == 0)
            {
                // this SSID of the reference matrix has not been detected within the sample
                WifiMatrix[MatRef].ssid[i][k].val =
                    ((WifiMatrix[MatRef].ssid[i][k].val * WifiMatrix[MatRef].Releves) + 99) /
                        (WifiMatrix[MatRef].Releves + 1);
            }
        }
        
        sleep (2);
    }
    
    if (MatRef >= 0)
    {
        WifiMatrix[MatRef].Releves += 1;
    }
    
    // remise ARDFD en mode non bloquant, avec SIGIO
    ard_async_mode ();
    close_wifi_matrix_file (fd);
    
    if (MatRef >= 0)
    {
        write_wifi_matrixes ();
    } else {
        control_message(MSG_WARNING, "Please update Matrix file with name, zone, and other data\n\n");
    }

    
    NoInterrupt = 0;
    return codeRet; // 1 = ok, 0 = pb de positionnement
}

int record_wifi_reference (int n)
{
    int i,j;
    int codeRet = 1;
    
printf ("recording wifi\n");
    codeRet = mesure_wifi (n);
    
    return codeRet; // 1 = ok, 0 = pb de positionnement
}

int locate_myself ()
{
    int i,j,n,p;
    int codeRet;
    int rang;
    struct wifiref CurrentWifi;
    float d, dmin;
    extern struct wifiref WifiMatrix[];
    extern int MaxMatrices;
    extern int ardfd;
    char buffer[64];
    unsigned char len;
    extern int DebugMode;
    
    NoInterrupt = 1;

    
    // change ARDFD en mode bloquant, sans envoyer de SIGIO
    ard_block_mode ();
    
    control_message(MSG_WARNING, "\nBug #1 : may return empty result : run \"TopWifi\" once first\n");
    control_message(MSG_WARNING, "Bug #2 : if blocked if the middle of the process click on the compass to release\n\n");
    
    // step 0 cleanup & prepare

    for (i=0; i<4; i++)
    {
        for (j=0; j< 10; j++)
        {
            CurrentWifi.ssid[i][j].val = -1;
            strcpy (CurrentWifi.ssid[i][j].label, "");
        }
    }
            
    // step 1 acquire current wifi environment
    
    for (i=0; i<4; i++)
    {
        printf ("Orientation %d\n",i*90);
        
        codeRet = oriente_robot (i*90, -1);
        if (codeRet == 0)
        {
            printf("Problème d'orientation\n");
            break;
        }
        
        get_top_wifi (Sequence[C_TOPWIFI]);
        
        for (j=0; j< 10; j++)
        {
            buffer[0] = ACTION_NULL;
            while (buffer[0] != C_TOPWIFI)
            {
                n = read(ardfd, &len, 1);
                if (n > 0)
                {
                    for (p=0; p<len; p++)
                        read (ardfd, &(buffer[p]), 1);
                    buffer[p] = '\0';
                    
                    if (buffer[0] == C_TOPWIFI)
                    {
                        sscanf (&(buffer[2]), "%d %s", &(CurrentWifi.ssid[i][j].val), CurrentWifi.ssid[i][j].label);
                    } else {
                        interpret_ard (buffer, 1);
                    }
                }
            }
        }
        
        sleep (2);
    }
    
    // remise ARDFD en mode non bloquant, avec SIGIO
    ard_async_mode ();
    NoInterrupt = 0;

    // step 1bis verify
    
    if (DebugMode == 1)
    {
        printf ("current wifi environment :\n");
        for (i=0; i<4; i++)
        {
            for (j=0; j< 10; j++)
            {
                printf ("ssid %d %d %d %s\n", i, j, CurrentWifi.ssid[i][j].val, CurrentWifi.ssid[i][j].label);
            }
        }
        printf ("\n");
    }

    // step 2 find closest match in global matrices table
    
    dmin = 999999.0;
    rang = -1;
    for (i=0; i<MaxMatrices; i++)
    {
        d = distance_matrix (&CurrentWifi, &(WifiMatrix[i]));
        WifiMatrix[i].distance = d;
        if (d < dmin)
        {
            dmin = d;
            rang = i;
        }
        
        if (DebugMode == 1)
        {
            printf ("Distance with matrix %d : %f\n",i,d);
        }
    }
    
    // step 3 display results
    
    if (DebugMode == 1)
    {
        printf ("*** Closest matrix is %d ***\n\n",rang);
    }
    display_room_from_matrix (WifiMatrix[rang].zone, "Aquamarine");
    
    for (i=0; i<MaxMatrices; i++)
    {
        if (WifiMatrix[i].distance / dmin < 1.05)  // let's display matrices where distance is close by less than 5%
        {
            if (DebugMode == 1)
            {
                printf ("* Near matrix found %d *\n",i);
            }
            display_room_from_matrix (WifiMatrix[i].zone, "BurlyWood");
        }
    }
    
    return codeRet; // 1 = ok, 0 = pb de positionnement
}

void average_and_save_wifi (int x, int y)
{
    printf ("real location %d %d\n",x,y);
}
