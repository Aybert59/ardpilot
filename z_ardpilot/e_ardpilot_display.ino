
// référence ici : https://github.com/olikraus/u8g2/wiki

// 8 lignes (de 0 à 7) et 16 charactères (de 0 à 15)

#define DefaultFont u8x8_font_artosserif8_r

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
