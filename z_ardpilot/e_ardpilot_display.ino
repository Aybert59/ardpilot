
// référence ici : https://github.com/olikraus/u8g2/wiki
// Ecran : référence ici : https://github.com/olikraus/u8g2/wiki et https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf


// 8 lignes (de 0 à 7) et 16 charactères (de 0 à 15)
#define DefaultFont u8x8_font_artossans8_r


// notes in the melody:
int melody1[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations1[] = {4, 8, 8, 4, 4, 4, 4, 4};

int melody2[] = {NOTE_E4,NOTE_E4,0,NOTE_DS5,NOTE_D5,NOTE_B4,NOTE_A4,NOTE_B4};
int noteDurations2[] = {8,8,16,8,2,8,8,1};

void begin_ecran ()
{
  ecran.begin();
  ecran.setFont(DefaultFont);
  ecran.setPowerSave(0);
}

void affiche_ecran (int x, int y, String message)
{
  ecran.setPowerSave(0);
  ecran.setCursor(x, y);
  ecran.print(message);
}

// open_iconic_check_1x ok 0, nok 4

void ecran_check ()
{
//  ecran.setFont(u8x8_font_open_iconic_all_1x_t);
  ecran.print('\0');
  ecran.setFont(DefaultFont);
}

void ecran_fail ()
{
//  ecran.setFont(u8g2_font_open_iconic_check_1x);
  ecran.print('\4');
  ecran.setFont(DefaultFont);
}

void play_melody (int num, int rythm) // rythm = 1000 ==> 1 second
{
  int noteDuration;
  int pauseBetweenNotes;
  
  switch (num) { 

      case 1:

        for (int thisNote = 0; thisNote < 8; thisNote++)
        {
          noteDuration = rythm / noteDurations1[thisNote];
          tone(BUZZER, melody1[thisNote], noteDuration);

          pauseBetweenNotes = noteDuration * 1.30;
          delay(pauseBetweenNotes);
          noTone(BUZZER);
        }
        
      break;
      case 2:

        for (int thisNote = 0; thisNote < 8; thisNote++)
        {
          noteDuration = rythm / noteDurations2[thisNote];
          tone(BUZZER, melody2[thisNote], noteDuration);

          pauseBetweenNotes = noteDuration * 1.30;
          delay(pauseBetweenNotes);
          noTone(BUZZER);
        }
        
      break;
  }
}
