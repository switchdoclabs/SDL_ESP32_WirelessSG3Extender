#include "rgb_lcd.h"

rgb_lcd lcd;

void startlcd()
{
  Serial.println("Setting up the LCD");
  lcd.begin(16, 2);

 
  lcd.print(" SmartGarden 3");
  lcd.setCursor(0, 1);
  lcd.print("   Version:");
  lcd.print(SGSEXTENDERESP32VERSION);


}

void displaytime()
{
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);


}
