#include <SPI.h>
#include "DW1000Ranging.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Anchor addresses: TO SWITCH BETWEEN
#define ANCHOR_ADD "86:17:5B:D5:A9:9A:E2:9C" // Anchor 1
// #define ANCHOR_ADD "3F:98:1B:AB:AF:92:68" // Anchor 2
// #define ANCHOR_ADD "cc:88:dc:cf:86:86:95" // Anchor 3

#define ANCHOR_ID 1

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define UWB_RST 27 // reset pin
#define UWB_IRQ 34 // irq pin
#define UWB_SS 21  // spi select pin

#define I2C_SDA 4
#define I2C_SCL 5

int connectedTags = 0;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
uint16_t Adelay = 16590;

void setup()
{
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    delay(1000);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.clearDisplay();

    logoshow();

    // init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(UWB_RST, UWB_SS, UWB_IRQ); // Reset, CS, IRQ pin

    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    DW1000.setAntennaDelay(Adelay);

    DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop()
{
    DW1000Ranging.loop();
    // updateDisplay();
}

void newRange()
{
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
    Serial.println("Connected tags: ");
    // Serial.print(connectedTags);
}

void newBlink(DW1000Device *device)
{
    Serial.print("blink; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
    connectedTags += 1;
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
    connectedTags -= 1;
}

void logoshow(void)
{
display.clearDisplay();

    display.setTextSize(2);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.println(F("Team DT6"));

    display.setTextSize(1);

    display.println(F("Anchor"));
    display.print(ANCHOR_ID);

    display.setCursor(0, 40); // Start at top-left corner
    display.println(ANCHOR_ADD);
    display.display();
}

void updateDisplay() {
    display.clearDisplay();

    display.setTextSize(1);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.println("Anchor ID:");
    display.println(ANCHOR_ADD);
    display.println("");

    display.println("Connected devices:");
    display.println(connectedTags);

    display.display();
}
