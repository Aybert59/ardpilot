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


void boucle_attente (unsigned char parametre)
// loops until parameter is received.
// if parameter == ACTION_NULL then will never exits - as nobody sends this parameter
{
    char buffer[4];
    int ret;
    fd_set rfds;
    unsigned char val = ACTION_NULL;
    

    while ((parametre == ACTION_NULL) || (val != parametre))
    {
        // attente soit du retour de la séquence demandée, soit ecrasée par une commande de l'interface graphique

        // initialize file descriptors table
        FD_ZERO(&rfds);
        if (ardfd > 0)
            FD_SET(ardfd, &rfds);
        if (cmdfd > 0)
            FD_SET(cmdfd, &rfds);
        if (scriptfd > 0)
            FD_SET(scriptfd, &rfds);

        
        ret = select(FD_SETSIZE, &rfds, NULL, NULL, NULL); // wait forever


        if (ret == -1)
             printf ("error on select()");
 //        else if (ret)
 //            printf("Data is available now.\n");
             /* FD_ISSET(0, &rfds) will be true. */
 //        else
 //            printf("No data within timeout period\n");
        

        if (FD_ISSET(cmdfd, &rfds))
            val = read_cmd (cmdfd, '\0');
        if (FD_ISSET(ardfd, &rfds))
            val = read_ard (ardfd); // in this case val equals the command received from the robot
        if (FD_ISSET(scriptfd, &rfds))
            val = read_cmd (scriptfd, '\n');
        
    }
}

char attente_bloquante (unsigned char parametre, unsigned char sequence) // a fusionner avec la boucle ppale, loopcnt en static
{
    char buffer[64];
    unsigned char cmd, len;
    int j, n;
    int flags;
    
     
    buffer[0] = ACTION_NULL;
printf ("wait ard to send me expected param\n");
    
    while (buffer[0] != parametre)
    {
        n = read(ardfd, &len, 1);
        if (n > 0)
        {
            for (j=0; j<len; j++)
                read (ardfd, &(buffer[j]), 1);
            buffer[j] = '\0';
            
            if (buffer[0] == parametre)
                break;
            else
                interpret_ard (buffer, 1);
         }
    }
printf ("got it %c!\n", buffer[3]);
    
    
    return (buffer[3]);  // renvoie T ou F mais seulement dans le cas C_COLOR
}


int bloc_get_top_wifi ()
{
printf("get top wifi\n");
         
    get_top_wifi ();
    boucle_attente (C_TOPWIFI);
    
    return 1;
}

int bloc_get_memory_free ()
{
    check_free_mem ();
    boucle_attente (C_MEM);
    
    // afficher les parametres relevés
    printf ("%d\n",MemoryFree);
    
    return MemoryFree;
}

int bloc_get_compas ()
{
    unsigned char sequence;
    int c, x, i, j, n;
    unsigned char len;
    char buffer[64];
    int flags;
    extern int compas_correction[];
    
    sequence = Sequence[C_CMP] + 1;
    if ((sequence == '*') || (sequence == 0x00))
        sequence++;
    
    check_compas (sequence);
    
    
    buffer[0] = ACTION_NULL;
    while (buffer[0] != C_CMP)
    {
        n = read(ardfd, &len, 1);
        if (n > 0) {
            for (j=0; j<len; j++)
                read (ardfd, &(buffer[j]), 1);
            buffer[j] = '\0';
            if (buffer[0] == C_CMP)
            {
                sscanf(&(buffer[2]), "%d", &x);
                c = compas_correction[x];
            } else {
                interpret_ard (buffer, 1);
            }
        } else if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
            printf ("Read timeout, resending\n");
            check_compas (sequence);
        }
    }
    
    
    printf("current orientation brute %d corrigée %d\n",x,c);
    return c;
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
    boucle_attente (C_PRI);
    
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
    char outcome;
    
    sequence = Sequence[C_PRI] + 1;
    if ((sequence == '*') || (sequence == 0x00))
        sequence++;
    
    buffer[0] = C_PRI;
    buffer[1] = sequence;
    buffer[2] = P_SPOT_TURN;
    cap = cap_correction[cap_demande];
    sprintf (&(buffer[3]), "%d",cap);
    
    write_ard (ardfd, buffer);
    outcome = attente_bloquante (C_COLOR, sequence);
    
    // afficher les parametres relevés
    
    if (outcome == 'T')
        printf ("Move completed\n");
    else
        printf ("Move blocked\n");
    
    return outcome;
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


