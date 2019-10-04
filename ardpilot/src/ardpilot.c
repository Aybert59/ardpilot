#define _GNU_SOURCE

#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h> 

#include "ardpilot.h"
#include "ardpilot_grammar.h"

int cmdfd = 0, ardfd = 0, scriptfd = 0, scriptWritefd =0;
FILE *logfd = 0;
int ArdBlocLevel = 0;

int DebugMode = 0;
char CompasCallibration = 'F';

pid_t process;

double pointsX[180];
double pointsY[180];
double mesures[180];

int PlanClickAction = 0;
// default 0 : indicate the place to go
//         1 : show correct location after self localization survey

extern float CovSeuil;
extern int LineW;
extern int PathW, PathD;

extern int compas_correction[];
extern int cap_correction[];
extern float carto_vitesse[];
extern unsigned char Sequence[];
extern int RSSI[];
extern int MemoryFree;
extern int Compas;
extern int TopWIFICount;

int NoInterrupt = 0;

extern void detectlines (double *, double *, double *, int, int, float);
extern void detectpath (double *, double *, double *, int, int, int);
extern void boucle_attente (unsigned char, unsigned char, int);

char logFileName[64];
char Expecting[16];
int (*ExpectFunction)();

#define PORT_NUMBER 8001

/* displays received message to the standard output */
void control_message (char action, char *message)
{
/* todo : manage color and other effects depending on subject and action */

    if (cmdfd > 0) {
        // control socket mode
        if (send (cmdfd, message, strlen (message), MSG_NOSIGNAL) < 0) {
          if (errno == EPIPE) {
            printf ("Pipe broken, resetting...\n");
            close (cmdfd);
            cmdfd = 0;
            cmdfd = open_cmd_socket (PORT_NUMBER);
          } else {
            printf ("Error %d while writing to node server\n", errno);
          }
        }
        
        //printf ("Message envoyé : %s\n", data);
    } else { // console mode
        printf ("%s\n", message);
    }
    
    if (logfd != 0)
        fprintf (logfd, "%s\n", message);
    if (scriptWritefd != 0)
    {
        write (scriptWritefd, message, strlen(message));
    }
}

void log_message (char action, char *message)
{
    /* todo : manage color and other effects depending on subject and action */
    static char data[1024];
    
    // add a \n
    sprintf (data, "LOG%c%s\n", action, message);
    control_message (action, data);

}

void
cleanup_fds ()
{
    control_message(MSG_WARNING, "\nCleaning up");
    
    close (cmdfd);
    close (ardfd);
    ardfd = 0;
    cmdfd = 0;
    
    fclose (logfd);
    logfd = NULL;
    
    if (scriptWritefd != 0)
        close (scriptWritefd);
    if (scriptfd != 0)
        close (scriptfd);
    scriptWritefd = 0;
    scriptfd = 0;
}

void
cleanup_on_exit (int signum)
{
    killpg( process, SIGKILL);
    cleanup_fds ();
    exit(0);
}

int match_expecting (char buffer[], char Expecting[])
{
    if (strlen (Expecting) == 0)
        return 0;
    
    if (buffer[0] != Expecting[0])
        return 0;
    
    // le code fonction est le bon
    // on délaisse la séquence en position 2
    
    // si la fonction est COLOR, alors on vérifie qu'on parle du bon id d'objet
    if (buffer[0] == C_COLOR)
    {
        if (!strcmp (&(buffer[4]), &(Expecting[4])))
            return 1;
    }
    
    // pour le moment on n'implémente pas l'attente pour d'autre type de résultat que COLOR
    return 0;
}

/* reads message received on command socket */
int read_ard (int fd)
{
    int n;
    unsigned char len;
    char buffer[64];
    
    // le premier char reçu est la longueur de ce qui arrive
    read(fd, &len, 1);
    
    if (DebugMode == 1)
        printf("%d bytes to read... ",len);
    n = read(fd, buffer, len);
    if (DebugMode == 1)
        printf(" got %d\n",n);
    
    if (n < 1)
        return 0;
    buffer[n] = '\0';
    
    if (match_expecting (buffer, Expecting) > 0)
    // si une chaine est attendue et on la reçoit, alors on execute la fonction prévue et c'est tout
    {
        n = (*ExpectFunction)(buffer);
        strcpy (Expecting, "");
        ExpectFunction = NULL;
        return n;
    }

    if (strlen (Expecting) == 0)
    // si rien n'est attendu en particulier, alors on traite le buffer.
    // sinon, on ne fait rien, on attend ce qui est attendu
    {
        interpret_ard (buffer, DebugMode);
    }

    return n;
}

void interpret_ard (char *buffer, int debug_mode)
{
    char message[128];
    int x, y, z;
    char *p;
    double angle;
    unsigned char cmd;
    char sequence, oldsequence;
    extern int RSSI[5];
    extern char PrimitiveResult;

    // for debug only
    double normX[180];
    double normY[180];
    int posx, posy;
    double val;
    
    // structure :
    // Buffer[0] : la commande
    // Buffer[1] : le numéro de séquence
    // Buffer[2] : si la commande est Primitive,  la primitive en question
    //           : si la commande est COLOR, la comande d'origine (Primitive par exemple)
    //           : sinon la valeur
    // Buffer[3] : si la commande est Primitive, la valeur
    //           : si la commande est COLOR, la valeur de color (T ou F en principe)
    // Buffer[4] : si la commande est COLOR, l'id de l'objet à colorer
    
        cmd = buffer[0];
        
        if (cmd == C_COLOR)
        {
            oldsequence = Sequence[buffer[2]];
            Sequence[buffer[2]] = buffer[1]; // dans le cas de COLOR la primitive initiale est dans buffer[2]
        } else {
            oldsequence = Sequence[cmd];
            Sequence[cmd] = buffer[1]; // sera la version finale, quand les sequences seront implémentées pour chaque commande
        }
        
        sequence = buffer[1];   // attention pas forcément implémenté sur chaque commande
        // pour le moment C_SCANH, C_SCANV, C_MEM, C_CMP, C_PRI
        switch (cmd) {
                
            case C_PING :
                if (debug_mode == 1)
                    printf ("    ARD-->Server PING\n");
                
                
                // todo :
                //
                // add other measures : volts and mem
                //
                // combine in one unique message
                //
                // and update gauges accordingly
                
                control_message(MSG_INFO, "COLORTPing");
                sleep(1);
                control_message(MSG_INFO, &(buffer[1]));
                break;
                
            case C_LOG :
                if (debug_mode == 1)
                    printf ("    ARD-->Server LOG %s\n",&(buffer[1]));
                
                control_message(MSG_INFO, &(buffer[1]));
                break;
                
            case C_CMP :
                sscanf(&(buffer[2]), "%d", &x);
                if ((x >= 0) && (x < 360))
                    Compas = compas_correction[x];
                else
                {
                    printf ("    ARD-->Server CMP %d weird value\n", x);
                    Compas = 0;
                }
                
                if (debug_mode == 1)
                    printf ("    ARD-->Server CMP %d (%d)\n", x, Compas);
                
                sprintf (message, "AXY %d", Compas);
                control_message(MSG_INFO, message);
                break;
            
            case C_LIDAR :
                sscanf(&(buffer[3]), "%d", &x);
                
                if (debug_mode == 1)
                    printf ("    ARD-->Server Lidar %d \n", x );
            
                sprintf (message, "Lidar %d ", x);
                control_message(MSG_INFO, message);
                break;
            
            case C_SR1 :
                sscanf(&(buffer[2]), "%d", &x);
                
                if (debug_mode == 1)
                    printf ("    ARD-->Server SR1 %d\n", x);
                
                sprintf (message, "SR1 %d", x);
                control_message(MSG_INFO, message);
                break;
            case C_SR2 :
                sscanf(&(buffer[2]), "%d", &x);
                
                if (debug_mode == 1)
                    printf ("    ARD-->Server SR2 %d\n", x);
                
                sprintf (message, "SR2 %d", x);
                control_message(MSG_INFO, message);
                break;
                
            case C_PARM :
                if (debug_mode == 1)
                    printf ("    ARD-->Server PARM\n", x);
                
                control_message(MSG_INFO, "Reloading config parameters");
                read_and_send_config ();
                break;
                
            case C_DISTL :
                if (debug_mode == 1)
                    printf ("    ARD-->Server DISTL %s\n", &(buffer[1]));
                
                sprintf (message, "Distance Gauche : %s", &(buffer[1]));
                control_message(MSG_INFO, message);
                break;
                
            case C_DISTR :
                if (debug_mode == 1)
                    printf ("    ARD-->Server DISTR %s\n", &(buffer[1]));
                
                sprintf (message, "Distance Droite : %s", &(buffer[1]));
                control_message(MSG_INFO, message);
                break;
                
            case C_MEM :
                sscanf(&(buffer[2]), "%d", &MemoryFree);
                
                if (debug_mode == 1)
                    printf ("    ARD-->Server MEM %d\n", MemoryFree);
                
                sprintf (message, "MEM %d", MemoryFree);
                control_message(MSG_INFO, message);
                break;
                
            case C_BAT :
                sscanf(&(buffer[1]), "%d", &x);
                
                if (debug_mode == 1)
                    printf ("    ARD-->Server BAT %d\n", x);
                
                sprintf (message, "VOLT %'.2f", ((float)x)*12.1/1023); // 560 = 100%
                control_message(MSG_INFO, message);
                break;
                

            case C_TOPWIFI :
                
                if (debug_mode == 1)
                    printf ("    ARD-->Server TOPWIFI %s\n", &(buffer[2]));
                
                // attention retenir oldsequnce les 9 premières fois
                TopWIFICount++;
                control_message(MSG_INFO, &(buffer[2]));
                if (TopWIFICount = 10)
                {
                    TopWIFICount = 0;
                    sleep (1);
                    control_message(MSG_INFO, "COLORTTopWifi");
                }
                break;
                
            case C_SCANH :
                if (debug_mode == 1)
                    printf ("    ARD-->Server SCANH\n");
                
                sprintf (message, "received scan segment %c.", buffer[2] + '0');
                control_message(MSG_INFO, message);
                draw_segment (buffer, buffer[2], C_SCANH);
                consolidate_points (buffer, buffer[2], C_SCANH, pointsX, pointsY, mesures);
                
                if ((int) buffer[2] == 6)  // we received all the points
                {
                    /*
                     printf ("X = [");
                     for (x=0; x<180; x++)
                     {
                     printf ("%f,",pointsX[x]);
                     }
                     printf ("]\n");
                     printf ("Y = [");
                     for (x=0; x<180; x++)
                     {
                     printf ("%f,",pointsY[x]);
                     }
                     printf ("]\n");
                     */
                    //for (z=0;z<180;z++)
                    //    printf("%f,%f\n",pointsX[z], pointsY[z]);
                    
                    printf("Checking for objects\n");
                    detectlines (pointsX, pointsY, mesures, 180, LineW, CovSeuil);
//                    printf("Checking for path\n");
//                    detectpath (pointsX, pointsY, mesures, 180, PathW, PathD);
                    
                    oriente_nord (mesures, 180, 315, normX, normY);             // Test at -45°
                    val = map_match (normX, normY, 180, 0xC0, &posx, &posy);    // test dans la chambre zone bureau (C0)
                    if (debug_mode == 1)
                        printf ("Located at %d,%d\n",posx,posy);
                    draw_matched_scan (normX, normY, 180, posx, posy);
                }
                break;
                
            case C_SCANV :
                if (debug_mode == 1)
                    printf ("    ARD-->Server SCANV\n");

                sprintf (message, "received scan segment %c.", buffer[2] + '0');
                control_message(MSG_INFO, message);
                draw_segment (buffer, buffer[2], C_SCANV);
                break;
                
            case C_PRI :
                if (debug_mode == 1)
                    printf ("    ARD-->Server PRI %c %c\n", buffer[2], buffer[3]);
                
                sprintf (message, "ENG%c%c", buffer[2], buffer[3]);
                PrimitiveResult = buffer[3];
                control_message(MSG_INFO, message);
                break;
                
            case C_COLOR :
                if (debug_mode == 1)
                    printf ("    ARD-->Server COLOR %s\n",&(buffer[3]));

                sprintf (message, "COLOR%s", &(buffer[3]));
                PrimitiveResult = buffer[3];
                control_message(MSG_INFO, message);
                break;
                
            default :
                if (debug_mode == 1)
                    printf ("    ARD-->Server unknown\n");
                
                sprintf(message, "Unknown message %d from Arduino (%d bytes : %s)", (int)buffer[0], strlen(buffer), &(buffer[1]));
                control_message(MSG_INFO, message);
        }
    
}



/* reads message received on command socket (from node.js) */
int exec_cmd (char *buffer, int debug_mode)
{
    int val, n, dist;
    char message[64];
    char cmd;
    char sequence;
    static int LedStatus = '\1';
    int x, y, m;
    float facteur;
    int duree;
    

        sequence = '*'; // par convention si demande vient de interface graphique
        
		if (!strncmp (buffer, "PING", 4) ) // the first 4 chars are equal to 'PING'
		{
            message[0] = C_PING;
            message[1] = '\0';
            write_ard (ardfd, message);
		}
        else if (!strncmp (buffer, "Test", 4) ) // the first 4 chars are equal to 'PING'
        {
            message[0] = C_LIDAR;
            message[1] = sequence;
            message[2] = '\0';
            write_ard (ardfd, message);
        }
		else if (!strncmp(buffer, "LED", 3))
		{
            message[0] = C_LED;
            if (LedStatus == '\1')
                LedStatus = '\2';
            else
                LedStatus = '\1';
            message[1] = LedStatus;
            message[2] = '\0';
            write_ard (ardfd, message);
            
            sprintf (message, "COLOR");
            if (LedStatus == '\1')
                strcat (message, "F");
            else
                strcat (message, "T");
            strcat (message, "Led");
            control_message(MSG_INFO, message);
        }
        else if (!strncmp(buffer, "DEBUG", 5))
        {
            if (DebugMode == 0)
                DebugMode = 1;
            else
                DebugMode = 0;
            
            sprintf (message, "COLOR");
            if (DebugMode == 0)
                strcat (message, "F");
            else
                strcat (message, "T");
            strcat (message, "DbgMode");
            control_message(MSG_INFO, message);
        }
        else if (!strncmp(buffer, "CALCMP", 6))
        {
            if (debug_mode == 1)
                printf ("    Server-->ARD CALCMP\n");
            
            if (CompasCallibration == 'F')
                CompasCallibration = 'T';
            else
                CompasCallibration = 'F';
        
            message[0] = C_CALCMP;
            message[1] = CompasCallibration;
            message[2] = '\0';
            write_ard (ardfd, message);
            
            sprintf (message, "COLORxCallibCompas");
            message[5] = CompasCallibration;
            control_message(MSG_INFO, message);
        }
        else if (!strncmp(buffer, "I2CSCAN", 7))
        {
            if (debug_mode == 1)
                printf ("    Server-->ARD I2CSCAN\n");
            
            message[0] = C_I2CSCAN;
            message[1] = '\0';
            write_ard (ardfd, message);
        }
        else if (!strncmp(buffer, "RSCPT", 5))
        {
            if (run_command_script(&(buffer[5])) == 1)
                control_message(MSG_INFO, "COLORTRunScript");
            else
                control_message(MSG_INFO, "COLORFRunScript");
        }
        else if (!strncmp(buffer, "SSCPT", 5))
        {
            stop_command_script(&(buffer[5]));
        }
        else if (!strncmp(buffer, "WREF", 4))
        {
            sprintf (message, "COLOR");
            sscanf (&(buffer[4]), "%d", &n);
            if (record_wifi_reference(n) == 1)
                strcat (message, "T");
            else
                strcat (message, "F");
            
            strcat (message, "wifiref");
            control_message(MSG_INFO, message);
        }
        else if (!strncmp(buffer, "LOCT", 4))
        {
            sprintf (message, "COLOR");
            
//            if (locate_myself() == 1)
                strcat (message, "T");
//            else
//               strcat (message, "F");
            
            strcat (message, "locate");
            control_message(MSG_INFO, message);
        }
        else if (!strncmp(buffer, "LOC_OK", 6))
        {
            printf ("Localization approved\n");
            average_and_save_wifi (-1,-1);
        }
        else if (!strncmp(buffer, "LOC_NOK", 7))
        {
            printf ("Localization rejected\n");
            PlanClickAction = 1;
        }
        else if (!strncmp(buffer, "TOPW", 4))
        {
            if (bloc_get_top_wifi() == 1)
            {
                sleep (2);
//conflit avec les messages venant du robot
                //                control_message(MSG_INFO, "COLORTTopWifi");
            }
            else
            {
                sleep(2);
//                control_message(MSG_INFO, "COLORFTopWifi");
            }
        }
        else if (!strncmp(buffer, "SR1", 3))
		{
            message[0] = C_SR1;
            sprintf (&message[1], "%s", buffer+3);
            write_ard (ardfd, message);
        }
        else if (!strncmp(buffer, "SR2", 3))
        {
            message[0] = C_SR2;
            sprintf (&message[1], "%s", buffer+3);
            write_ard (ardfd, message);
        }
		else if (!strncmp(buffer, "ENVT", 4))
		{
            analyze_environment ();
		}
        else if (!strncmp(buffer, "LOGF", 4))
        {
            printf ("Sending logs to %s\n", buffer+4);
            sprintf (message, "Logs/%s", buffer+4);
            rename (logFileName, message);
            strcpy (logFileName, message);
            sprintf (message, "LOGF %s", buffer+4);
            control_message(MSG_INFO, message);
        }
        else if (!strncmp(buffer, "SANTE", 5))
        {
            get_health ();
        }
        else if (!strncmp(buffer, "SHTDN", 4))
        {
            control_message(MSG_INFO, "Shutdown requested");
            message[0] = C_SHTDN;
            message[1] = '\0';
            write_ard (ardfd, message);
            sleep (2);
            cleanup_on_exit(0);
        }
        else if (!strncmp(buffer, "CMP", 3))
        {
            check_compas (sequence);
        }
        else if (!strncmp(buffer, "CFG", 3))
        {
            message[0] = C_PARM;
            message[1] = F_INIT;
            message[2] = '\0';
            write_ard (ardfd, message);
        }
        else if (!strncmp(buffer, "DST", 3))
        {
                message[0] = C_DISTL;
                message[1] = '\0';
                write_ard (ardfd, message);
        }
        else if (!strncmp(buffer, "ENG", 3))
        {
            sprintf(message, "Command Engine was received (%c) value : %s)", buffer[3], &(buffer[4]));
            control_message(MSG_INFO, message);
            
            message[0] = C_PRI;
            message[1] = sequence;

            if ((buffer[3] == 'S') || (buffer[3] == 'L') || (buffer[3] == 'T'))
            {
                sscanf(buffer+4, "%d", &val);
                n = cap_correction[val];
printf ("Cap demandé : %d, réel : %d\n",val,n);
//                sprintf (buffer+4, " %d", n);
                
                sprintf (&message[2], "%c %d", buffer[3], n);
            } else
            {
                sscanf(buffer+4, "%d %d", &val, &dist);
                if (val < -120)
                    val = -120;
                if (val > 120)
                    val = 120;
                
                if (dist < 0)
                    dist = 0;
                if (dist > 200)
                    dist = 200;

                x = (int) floor (val / 5.0);
                m = val % 5;
                if (m == 0)
                    facteur = carto_vitesse[x];
                else
                {
                    facteur = (carto_vitesse[x+1] - carto_vitesse[x]);
                    facteur = ((facteur / 5) * m) + carto_vitesse[x];
                }
                duree = (int) round((dist / facteur) * 10000);
                
                sprintf (&message[2], "%c %d %d", buffer[3], val, duree);
            }
            write_ard (ardfd, message);
		}
        else if (!strncmp(buffer, "SCN", 3))
        {
            if (buffer[3] == 'h') {
                control_message(MSG_INFO, "Command Scan Horizontal was received");
                message[0] = C_SCANH;
                message[1] = sequence;
                message[2] = '\0';
                write_ard (ardfd, message);
            } else {
                control_message(MSG_INFO, "Command Scan Vertical was received");
                message[0] = C_SCANV;
                message[1] = sequence;
                message[2] = '\0';
                write_ard (ardfd, message);
            }
        }
        else if (!strncmp(buffer, "END", 3))
        {
            close (scriptWritefd);
            close (scriptfd);
            scriptWritefd = 0;
            scriptfd = 0;
            if (buffer[3] == 'F')
            {
                control_message(MSG_INFO, "COLORFRunScript");
            } else {
                control_message(MSG_INFO, "COLORTRunScript");
            }
        }
        else if (!strncmp(buffer, "GOTO", 4)) // received a click on the plan
        {
            sscanf (&(buffer[4]), "%d %d", &x, &y);
            switch (PlanClickAction) {
                case 0 :
                    printf ("Received request to go to x=%d, y=%d\n", x, y);
                break;
                
                case 1 :
                    average_and_save_wifi (x,y);
                    PlanClickAction = 0;
                break;
            }
        }
        else
        {
            printf ("Can't interpret command %s of length %d\n", buffer, strlen(buffer));
        }

    val = 0;
    return val;
}

int read_cmd (int fd, char fin)
{
    int val, n;
    char buffer[256];

    
    for (n = 0; n < 256; n++)
        buffer[n] = '\0';
    
    n = 0;
    while( read(fd, &buffer[n++], 1) == 1)
    {
        if (buffer[n-1] == fin)
        {
            buffer[n-1] = '\0';
            break;
        }
    }
    
    if (n > 0)
    {
        val = exec_cmd (buffer, DebugMode);
    }

    return val;
}

/* an input is pending on one of the socket */
void read_handler (int signum, siginfo_t *si, ucontext_t *unused)
{
    int val;
    int fd;
    long event;
    

    if (si) {

        fd = si->si_fd;
        event = si->si_band;
        
        // check which event did actually trigger the I/O handler
        if ((event && POLLIN) == 0) {
            printf ("Other event\n");
            return;
        }
        
        if (fd == (int) NULL) {
            printf ("FD is null\n");
            control_message(MSG_WARNING, "FD is null");
        }
        else if (fd == ardfd) {
            read_ard (fd);
        }
        else if (fd == cmdfd){
            read_cmd (fd, '\0');
        }
        else if (fd == scriptfd){
            read_cmd (scriptfd, '\n');
        }
        else {
            printf ("unknown FD received on SIGIO\n");
            control_message(MSG_WARNING, "unknown FD received on SIGIO");
        }
    }
    else {
        printf ("siginfo is null\n");
        control_message(MSG_WARNING, "siginfo is null");
    }

}

void ard_block_mode ()
{
    int flags;
    struct timeval tv;

    tv.tv_sec = 3;  // timeout after 3 seconds
    tv.tv_usec = 0;
    
    
    ArdBlocLevel++;
    
    flags = fcntl(ardfd, F_GETFL);
    if (fcntl(ardfd, F_SETFL, flags & ~O_ASYNC & ~O_NONBLOCK) == -1)
        exit_on_failure ("fcntl(F_SETFL) error on socket fd");
    setsockopt(ardfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    
}

void ard_async_mode ()
{
    int flags;
    
    if (ArdBlocLevel >= 2)
        ArdBlocLevel--;
    else
    {
        ArdBlocLevel = 0;
        flags = fcntl(ardfd, F_GETFL);
        if (fcntl(ardfd, F_SETFL, flags | O_ASYNC | O_NONBLOCK) == -1)
            exit_on_failure ("fcntl(F_SETFL) error on socket fd");
    }
}

void exit_on_failure (char *reason)
{
    killpg( process, SIGKILL);
    cleanup_fds ();
    control_message(MSG_ERROR, reason);
    exit(1);
}


void pipe_broken (int signum)
{
    printf("pipe broken unmanaged\n");
    killpg( process, SIGKILL);
    cleanup_fds ();
    exit(0);
}

void reset_communication (int signum)
{
    printf("Reset communication request\n");

}

int open_cmd_socket (int portno)
{
    int fd1, fd2, flags, i=0;
    socklen_t clilen;
    int opt;
    struct sockaddr_in serv_addr, cli_addr;

    fd1 = socket(AF_INET, SOCK_STREAM, 0);
    if (fd1 < 0) {
        exit_on_failure ("ERROR opening socket");
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    
    opt = 1;
    if (setsockopt (fd1, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        exit_on_failure ("ERROR on setsockopt");
    }

    if (bind(fd1, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
                exit_on_failure ("ERROR on binding");
    }
    
    // wait and accept a client on the socket
    
    listen(fd1,5);
    clilen = sizeof(cli_addr);
    fd2 = accept(fd1, (struct sockaddr *) &cli_addr, &clilen);
    if (fd2 < 0) {
        exit_on_failure ("ERROR on accept");
    }
    control_message(MSG_INFO, "Command connection accepted");

    // enlever les 5 lignes suivantes tq le fd reste en synchrone
    // dans la boucle d'attente utiliser poll 
    
    //
//    fcntl(fd2, F_SETOWN, getpid());
//    fcntl(fd2, F_SETSIG, SIGIO);
//    flags = fcntl(fd2, F_GETFL);
//    if (fcntl(fd2, F_SETFL, flags | O_ASYNC | O_NONBLOCK) == -1)
//        exit_on_failure ("fcntl(F_SETFL) error on socket fd");

    close (fd1);
    return (fd2);
}


int open_ard_socket (int portno)
{
    int fd1, fd2, flags, i=0;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    int opt;
    
    fd1 = socket(AF_INET, SOCK_STREAM, 0);
    if (fd1 < 0) {
        exit_on_failure ("ERROR opening socket");
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    
    opt = 1;
    if (setsockopt (fd1, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        exit_on_failure ("ERROR on setsockopt");
    }

    if (bind(fd1, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        exit_on_failure ("ERROR on binding");
    }
    
    // wait and accept a client on the socket
    
    control_message(MSG_INFO, "Now waiting for Arduino to connect");
    listen(fd1,5);
    clilen = sizeof(cli_addr);
    fd2 = accept(fd1, (struct sockaddr *) &cli_addr, &clilen);
    if (fd2 < 0) {
        exit_on_failure ("ERROR on accept");
    }
    control_message(MSG_INFO, "Arduino connection accepted");
    
    fcntl(fd2, F_SETOWN, getpid());
    fcntl(fd2, F_SETSIG, SIGIO);
    flags = fcntl(fd2, F_GETFL);
    if (fcntl(fd2, F_SETFL, flags | O_ASYNC | O_NONBLOCK) == -1)
        exit_on_failure ("fcntl(F_SETFL) error on socket fd");
    
    close (fd1);
    return (fd2);
}

int write_ard (int fd, char *message)
{
    if (DebugMode == 1)
        printf ("    Server-->ARD %d bytes\n", strlen(message) + 1);
    
    if (send (fd, message, strlen(message) + 1, MSG_NOSIGNAL) < 0) {
        if (errno == EPIPE) {
            printf ("Pipe broken, resetting...\n");
            close (fd);
            ardfd = 0;
            ardfd = open_ard_socket (PORT_NUMBER + 1);
        }
    }
}


int main(int argc, char *argv[])
{
    int n, opt, optFileName;
    struct sigaction int_action, io_action, old_action, pipe_action, rst_action;
    pid_t pgid;
    char buffer[4];
    char message[64];


    
    process = vfork();
    if (process == 0)             // child
    {
        // Code only executed by child process
        sleep (1);
        execl ("/usr/bin/node", "/usr/bin/node", "/home/olivier/projets/node/index.js", (char *) 0);
    }
    else if (process < 0)            // failed to fork
    {
        exit_on_failure ("failed to fork");
    }
    else                         // parent
    {
      // Code only executed by parent process
      pgid = setpgid(process, process);
      if( pgid < 0)
      {
          printf("Failed to set process group ID : ");
         if(errno == EACCES)
             printf("setpgid: Attempted to change child proecess GID. Child process already performed an execve\n");
         else if(errno == EINVAL)
             printf("setpgid: pgid is less than 0\n");
         else if(errno == EPERM)
             printf("setpgid: Attempt to move proocess into process group failed. Can not move process to group in different session.\n");
         else if(errno == ESRCH)
             printf("setpgid: pid does not match any process\n");
          
     }
     
        
        printf("\nPlease launch web interface once Node is running\n\n");
        
        // parse arguments before forking
        
        strcpy (logFileName, "Logs/default.log");
        optFileName = 0;

        while ((opt = getopt(argc, argv, "o:")) != -1) {
            switch (opt) {
                case 'o':
                    strcpy (logFileName, "Logs/");
                    strcat (logFileName, optarg);
                    printf ("request to send logs to %s\n",logFileName);
                    optFileName = 1;
                    break;
                    
                default: /* '?' */
                    fprintf(stderr, "Usage: %s [-o logFileName] ... ignoring args\n",
                            argv[0]);
            }
        }
        
        // setup interruptions : quit
        
        logfd = fopen (logFileName,"w");
        if (logfd == NULL)
            printf("!! Error cannot open log file\n");
        
        int_action.sa_handler = cleanup_on_exit;
        sigemptyset (&int_action.sa_mask);
        int_action.sa_flags = 0;
        
        sigaction (SIGINT, &int_action, NULL);
        sigaction (SIGTERM, &int_action, NULL);
        // comment if core dump expected
//      sigaction (SIGQUIT, &int_action, NULL);
        
        // setup interruptions : rest communication (SIGUSR1)
        
        rst_action.sa_handler = reset_communication;
        sigemptyset (&rst_action.sa_mask);
        rst_action.sa_flags = 0;
        
        sigaction (SIGUSR1, &rst_action, NULL);
        
        // setup interruptions : pipe broken
    
        pipe_action.sa_handler = pipe_broken;
        sigemptyset (&pipe_action.sa_mask);
        pipe_action.sa_flags = 0;
    
        sigaction (SIGPIPE, &pipe_action, NULL);
    
        // setup interruptions : io pending
    
        io_action.sa_handler = (__sighandler_t)read_handler;
        sigemptyset (&io_action.sa_mask);
        io_action.sa_flags = SA_SIGINFO|SA_NODEFER;
    
        sigaction (SIGIO, &io_action, NULL);

        // initialize reference data
        read_plan ();
        read_wifi_matrixes ();
        
        // open command and Arduino sockets
    
        control_message(MSG_INFO, "Now opening command channel ... ");
        cmdfd = open_cmd_socket (PORT_NUMBER);
        
        // initialiser la couleur du ping à orange
        control_message(MSG_INFO, "COLORFPing");
        sleep (1);
        if (optFileName == 1)
        {
            sprintf (message, "LOGF %s", &(logFileName[5]));
            printf("envoi de -%s-\n",message);
            control_message(MSG_INFO, message);
        }
        
        // dessiner le plan de l'appart
        sleep(1);
        draw_plan();
        init_appt_distances ();
        
        // BUG !! on ne devrait pas dialoguer avec la 1e socket tant que la seconde n'est pas établie
        log_message(MSG_INFO, "Now opening arduino channel ... ");
        ardfd = open_ard_socket (PORT_NUMBER + 1);
        
        control_message(MSG_INFO, "Starting activity ... ");
        control_message(MSG_INFO, "Please turn the robot 360° on itself upon startup");
        init_command_mode ();
        strcpy(Expecting, "");
        
        boucle_attente (ACTION_NULL, 0xff, 1);
    }
    // Code executed by both parent and child.
    
    return 0;
}

