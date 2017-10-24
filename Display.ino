#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(DISPLAY_PIN);

void setupDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void updateDisplay(char *contents, int COLOR = WHITE) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(COLOR);
  display.setCursor(0,0);
  display.println(contents);
  display.display();
}

