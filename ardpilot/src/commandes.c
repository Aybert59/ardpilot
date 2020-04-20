#define _GNU_SOURCE

#include <stdio.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <poll.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/poll.h>

#include "ardpilot_grammar.h"
#include "ardpilot.h"

// ce fichier implémente les commandes (bloquantes et non bloquantes) qui seront utilisées
// depuis le raspberry pour piloter le robot

extern int ardfd, cmdfd, scriptfd, scriptWritefd;
extern int cap_correction[];
unsigned char Sequence[256];
pid_t RunningScript = (pid_t) NULL;

// recupération des résultats
int RSSI[5];
int MemoryFree = 2048;
char PrimitiveResult = 'T';
int Compas = 0;

extern int NoInterrupt;

void init_command_mode ()
{
    int i;
    
    for (i=0; i<256; i++)
        Sequence[i] = 0x00;
    
    for (i=0; i<5; i++)
        RSSI[i] = 0;
}


int boucle_attente (unsigned char parametre, int attente, int retries)
// loops until parameter is received.
// if parameter == ACTION_NULL then will never exits - as nobody sends this parameter
{
    char buffer[4];
    int ret;
    fd_set rfds;
    unsigned char val = ACTION_NULL;
    struct timeval timeout;
    int retcode = 0;
    int count = retries;

    timeout.tv_sec = 0;
    timeout.tv_usec = attente * 1000; // attente in milliseconds

    while ((parametre == ACTION_NULL) || (val != parametre))
    {
        // attente d'avoir reçu le paramètre demandé

        // initialize file descriptors table
        FD_ZERO(&rfds);
        if (ardfd > 0)
            FD_SET(ardfd, &rfds);
        if (cmdfd > 0)
            FD_SET(cmdfd, &rfds);
        if (scriptfd > 0)
            FD_SET(scriptfd, &rfds);

        if (attente == 0)
            ret = select(FD_SETSIZE, &rfds, NULL, NULL, NULL); // wait forever
        else
            ret = select(FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        
        if (ret == -1)
        {
            printf ("error on select()");
            // let's loop infinitely, user will shutdown the system himself (something weird happened)
            retcode = 1;
        }
        else if (ret)
        {
            // printf("Data is available now.\n");
            // FD_ISSET(0, &rfds) will be true.
            
            if (FD_ISSET(cmdfd, &rfds))
                val = read_cmd (cmdfd, '\0');
            if (FD_ISSET(ardfd, &rfds))
                val = read_ard (ardfd); // in this case val equals the command received from the robot
            if (FD_ISSET(scriptfd, &rfds))
                val = read_cmd (scriptfd, '\n');
            
            retcode = 0;
        }
        else if (ret == 0)
        {
            printf("No data within timeout period, remaining %d tries\n", count);
            if (count == 0)
            {
                // number of retries has expired
                retcode = 1;
                break;
            }
            else
                count--;
        }
        
    }
    
    return retcode;
}

int bloc_get_top_wifi ()
{
    do
        get_top_wifi ();
    while (boucle_attente (C_TOPWIFI, 10000, 0)); // attention faux, devrait appeler bloucle attente 10 fois avant de revenir car le robot renvoie 10 messages
    
    
    // attention si boucle attente sort en timeout alors ret = 1 ==> le while va tourner en rond
    // mais à un moment il devrait trouver ce qu'il attend ????
    
    return 1;
}

int bloc_get_memory_free ()
{
    do
        check_free_mem ();
    while (boucle_attente (C_MEM, 2000, 0));
    
    return MemoryFree;
}

int bloc_get_compas ()
{
    do
        check_compas ();
    while (boucle_attente (C_CMP, 2000, 0));
           
    return Compas;
}

int bloc_check_voltage ()
{
    do
        check_voltage ();
    while (boucle_attente (C_BAT, 2000, 0));
}

int bloc_check_ping ()
{
    do
        check_ping ();
    while (boucle_attente (C_PING, 2000, 0));
}

char bloc_primitive_avant (int vitesse, int distance) // distance en mm
{
    unsigned char sequence;
    char buffer[64];
    int i;
    unsigned int duree;
    
    // necessité de calibrer distance parcourue en fonction de la vitesse et de la durée
    
    sequence = Sequence[C_PRI] + 1;
    if ((sequence == '*') || (sequence == 0x00))
        sequence++;
    
    //duree = distance_to_millis (vitesse,distance);
duree = distance;

    buffer[0] = C_PRI;
    buffer[1] = sequence;
    buffer[2] = P_AVANT;
    sprintf (&(buffer[3]), "%d %u",vitesse,duree);
    
    write_ard (ardfd, buffer);
    boucle_attente (C_PRI, 0, 0);
    
    // afficher les parametres relevés
    
    if (PrimitiveResult == 'T')
        printf ("Move completed\n");
    else
        printf ("Move blocked\n");
    
    return PrimitiveResult;
}

char bloc_primitive_spot_turn (int cap_demande) // cap corrigé en ° (0 = boulevard, 180 = jardin)
{
    unsigned char sequence;
    char buffer[64];
    int cap;
    int ret = 1, retries = 5;
    
    extern char PrimitiveResult;
    
    sequence = Sequence[C_PRI] + 1;
    if ((sequence == '*') || (sequence == 0x00))
        sequence++;
    
    buffer[0] = C_PRI;
    buffer[1] = sequence;
    buffer[2] = P_SPOT_TURN;
    cap = cap_correction[cap_demande];
    sprintf (&(buffer[3]), "%d",cap);
    
    PrimitiveResult = 'T';
    while ((ret == 1) && (retries > 0))
    {
        write_ard (ardfd, buffer);
        ret = boucle_attente (C_COLOR, 5000, 0); // wait 5 seconds
        
        retries--;
    }
    
    // afficher les parametres relevés
    
    if (PrimitiveResult == 'T')
        printf ("Move completed\n");
    else
        printf ("Move blocked\n");
    
    return PrimitiveResult;
}

int dif_angle(int x, int y)
{
    int arg;
    
    arg = (y-x) % 360;
    if (arg < 0 )  arg  = arg + 360;
    if (arg > 180) arg  = arg - 360;
    
    return (-arg);
}


int oriente_robot (int cap_souhaite, int tolerance)
{
    int cap, cap_demande;
    char result;
    int nb_blocs = 0;
    int code_retour = 1;
    
    cap_demande = cap_souhaite % 360;

printf ("=== oriente_robot vers %d\n", cap_demande);
    
    cap = bloc_get_compas ();
printf ("=== cap courant %d\n", cap);
    
    if (tolerance == -1)
    {
        result = 'F';
        while (result == 'F')
        {
            printf("demande %d\n",cap_demande);
            result = bloc_primitive_spot_turn (cap_demande);
            printf("success %c\n",result);
            if (result == 'F')
                nb_blocs++;
            if (nb_blocs > 2)
            {
                code_retour = 0;
                break;
            }
            cap = bloc_get_compas ();
        }
    } else
    {
        while (abs (dif_angle (cap, cap_demande)) > tolerance)
        {
            printf("demande %d\n",cap_demande);
            result = bloc_primitive_spot_turn (cap_demande);
            printf("success %c\n",result);
            if (result == 'F')
                nb_blocs++;
            if (nb_blocs > 2)
            {
                code_retour = 0;
                break;
            }
            cap = bloc_get_compas ();
        }
    }
    return (code_retour);
}

int run_command_script(char *ScriptName)
{
    char fileName[32];
    char commandName[64];
    int i, flags;
    int pc[2]; /* Parent to child pipe */
    int cp[2]; /* Child to parent pipe */
    char *name[] = {
        "/bin/bash",
        NULL,
        NULL
    };
    
    if (!strcmp (ScriptName, "Load Script")) // the default string
    {
        control_message(MSG_INFO, "ERROR : no script specified", 0);
    }
    else
    {
        sprintf(commandName, "Scripts/%s", ScriptName);
        /* Make pipes */
        if( pipe(pc) < 0)
        {
            control_message(MSG_INFO, "ERROR : Can't make pipe", 0);
            return(0);
        }
        if( pipe(cp) < 0)
        {
            control_message(MSG_INFO, "ERROR : Can't make pipe", 0);
            return(0);
        }
        
        /* Create a child to run command. */
        switch( RunningScript = fork() )
        {
            case -1:
                control_message(MSG_INFO, "ERROR : Can't fork script", 0);
                return(0);
            case 0:
                /* Child. */
                close(1); /* Close current stdout. */
                dup( cp[1]); /* Make stdout go to write end of pipe. */
                close(0); /* Close current stdin. */
                dup( pc[0]); /* Make stdin come from read end of pipe. */
                close( pc[1]);
                close( cp[0]);
                name[1] = commandName;
                execvp(name[0], name);
/*    printf ("errno : %d\n", errno); */
                control_message(MSG_INFO, "ERROR : Can't exec script", 0);
                printf("end\n");
                exit(1);
            default:
                /* Parent. */
                /* Close what we don't need. */
                close(pc[0]);
                close(cp[1]);
                scriptfd = cp[0];
                scriptWritefd = pc[1];

                    
/*                fcntl(scriptfd, F_SETOWN, getpid());
                fcntl(scriptfd, F_SETSIG, SIGIO);
                flags = fcntl(scriptfd, F_GETFL);
                if (fcntl(scriptfd, F_SETFL, flags | O_ASYNC | O_NONBLOCK) == -1)
                    exit_on_failure ("fcntl(F_SETFL) error on socket fd");
 */
        }
    }
    return(1);
}

int stop_command_script(char *ScriptName)
{
    printf("stopping script %s\n",ScriptName);
    
    if (RunningScript > 0)
    {
        kill(RunningScript, SIGKILL);
        RunningScript = (pid_t) NULL;
        close (scriptfd);
        close (scriptWritefd);
        scriptfd = 0;
        scriptWritefd = 0;
    }
    return (1);
}


