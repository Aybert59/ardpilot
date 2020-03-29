//
//  ardpilot.h
//  
//
//  Created by Olivier Messiant on 28/11/2016.
//

#ifndef ardpilot_h
#define ardpilot_h

#define PORT_NUMBER 8001

struct cellule
{
    unsigned char piece; // 4 msb : pièce, 4 lsb : zone dans la pièce
    // correspond aux deux premiers codes du fichier CSV, chacun entre 0 et F
    // par convention 00 = vide, 0F = mur
    unsigned char WifiMatrix; // 0 : pas de matrice de référence ; n = numéro de la matrice mesurée
};

#define APPT_L 193
#define APPT_W  93
#define BORDER 10

struct _ssid
{
    int updated;    // used during learning process
    int val;        // SSID strength
    char label[32]; // SSID name
};

struct wifiref
{
    unsigned int zone;
    char name[32];
    struct _ssid ssid[4][10]; // the actual matrix
    int i; // ordonnée dans le plan de l'appart (unused)
    int j; // abscisse dans le plan de l'appart (unused)
    int Releves; // number of measurements (to calculate average)
    float distance; // store the distance from the robot while locating itself
};

// prototypes

void control_message (char action, char *message, int attente);
unsigned char exec_cmd (char *buffer, int debug_mode);
int write_ard (int fd, char *message);
unsigned char read_ard (int fd);
unsigned char interpret_ard (char *buffer, int debug_mode);
void exit_on_failure (char *reason);
void init_command_mode ();
void ard_block_mode ();
void ard_async_mode ();
int open_cmd_socket (int portno);
unsigned char read_cmd (int fd, char fin);// from environment.c
int bloc_get_memory_free ();
void check_compas ();
void check_voltage ();
void check_free_mem ();
void check_ping ();
void get_top_wifi ();
void check_wifi_environment (char sequence);
int get_health ();
int analyze_environment ();
int record_wifi_reference (int n);
void average_and_save_wifi (int x, int y);
int open_ard_socket (int portno);

// from drawing.c
void draw_plan ();
void consolidate_points (char *buffer, char segment, char orientation, double X[], double Y[], double mesures[]);
void draw_segment (char *buffer, char segment, char orientation);
void draw_line (int xs, int ys, int xe, int ye);
void draw_matched_scan (double x[], double y[], int taille, int posx, int posy);
void display_room_from_matrix (unsigned int zone, char *color);
void draw_path (int x[], int y[], int taille);

// from read_config
void read_plan();
void read_matrices();
void read_wifi_matrixes();
void write_wifi_matrixes();
FILE *open_wifi_matrix_file (int MatRef);
void close_wifi_matrix_file (FILE *fd);
void init_appt_distances ();
void read_and_send_config ();
void rename_matrix ();

// from commandes.c
int boucle_attente (unsigned char parametre, int timeout, int retries);
int oriente_robot (int cap_souhaite, int tolerance);
int bloc_get_top_wifi ();
int bloc_check_voltage ();
int bloc_check_ping ();
int bloc_get_compas ();
int locate_myself ();
int stop_command_script(char *ScriptName);
int run_command_script(char *ScriptName);

// from calculs.c
double map_match (double x[], double y[], int taille, unsigned char piece, int *posx, int *posy);
void oriente_nord (double points[], int taille, int orientation, double xnorm[], double ynorm[]);
double find_best_match (double cap, double spread, double step, double mesures[], int taille, int *minx, int *miny, double *minAngle, double minNormX[], double minNormY[], unsigned char piece);
int find_path_to (int x[], int y[], int TailleMax, int CurX, int CurY, int ToX, int ToY);

int get_path (int x[], int y[], int TailleMax, int CurX, int CurY, int ToX, int ToY);
int lee_expansion (int ApptPath[][APPT_W], int level, int CurX, int CurY, int ToX, int ToY);

#endif /* ardpilot_h */
