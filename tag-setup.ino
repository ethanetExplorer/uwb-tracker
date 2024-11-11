#include <SPI.h>
#include "DW1000Ranging.h"
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <math.h>

#define TAG_ADDR "7D:00:22:EA:82:60:3B:9F"

// #define DEBUG

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define ALARM_PIN 2
#define UWB_RST 27 // reset pin
#define UWB_IRQ 34 // irq pin
#define UWB_SS 21   // spi select pin

#define I2C_SDA 4
#define I2C_SCL 5

const char* ssid = "ssid";     // Your Wi-Fi SSID
const char* password = "password"; // Your Wi-Fi password
int maxRetries = 8;
int retryCount = 0;
unsigned long retryDelay = 5000;

// Track sections definition
struct TrackSection {
    uint16_t anchor1_addr;
    uint16_t anchor2_addr;
    float d_ij;  // Straight line distance between anchors
    float s_ij;  // Arc length for curved sections
    bool isBooked;
};

// check section tolerance
float tolerance = 0.5f;

// Example predefined track sections
TrackSection trackSections[] = {
    {0x88CC, 0x983F, 1.0, 1.0, true},  // Example values
    {0x983F, 0x1786, 1.0, 1.0, false}   // Example values
};


// Distance measurement data structure
struct Link
{
    uint16_t anchor_addr;
    float range;
    float dbm;
    struct Link *next;
};

struct Link *uwb_data;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

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
    // define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    // Enable the filter to smooth the distance
    DW1000Ranging.useRangeFilter(true);

    // we start the module as a tag
    DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    // DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_LONGDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_SHORTDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_LONGDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_LONGDATA_RANGE_ACCURACY);

    uwb_data = init_link();

    // Attempt Wi-Fi Connection
    while (WiFi.status() != WL_CONNECTED && retryCount < maxRetries) {
      Serial.println("Attempting to connect to WiFi...");
      WiFi.begin(ssid, password);  // Try connecting to WiFi
      delay(retryDelay);
      retryCount++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("WARNING: Wi-Fi not connected after multiple attempts");
    }
}

long int runtime = 0;

void loop()
{
    DW1000Ranging.loop();
    if ((millis() - runtime) > 1000)
    {
        display_uwb(uwb_data);
        if (WiFi.status() == WL_CONNECTED) {
          send_data(check_section(generate_raw_json(uwb_data)));
          // send_data(generate_raw_json(uwb_data));
        }
        check_booking(check_section(generate_raw_json(uwb_data)));
        runtime = millis();
    }
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

    fresh_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange(), DW1000Ranging.getDistantDevice()->getRXPower());
    // print_link(uwb_data);
}

void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);

    add_link(uwb_data, device->getShortAddress());
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);

    delete_link(uwb_data, device->getShortAddress());
}

// Data Link

struct Link *init_link()
{
#ifdef DEBUG
    Serial.println("init_link");
#endif
    struct Link *p = (struct Link *)malloc(sizeof(struct Link));
    p->next = NULL;
    p->anchor_addr = 0;
    p->range = 0.0;

    return p;
}

void add_link(struct Link *p, uint16_t addr)
{
#ifdef DEBUG
    Serial.println("add_link");
#endif
    struct Link *temp = p;
    // Find struct Link end
    while (temp->next != NULL)
    {
        temp = temp->next;
    }

    Serial.println("add_link:find struct Link end");
    // Create a anchor
    struct Link *a = (struct Link *)malloc(sizeof(struct Link));
    a->anchor_addr = addr;
    a->range = 0.0;
    a->dbm = 0.0;
    a->next = NULL;

    // Add anchor to end of struct Link
    temp->next = a;

    return;
}

struct Link *find_link(struct Link *p, uint16_t addr)
{
#ifdef DEBUG
    // Serial.println("find_link");
#endif
    if (addr == 0)
    {
        Serial.println("find_link:Input addr is 0");
        return NULL;
    }

    if (p->next == NULL)
    {
        Serial.println("find_link:Link is empty");
        return NULL;
    }

    struct Link *temp = p;
    // Find target struct Link or struct Link end
    while (temp->next != NULL)
    {
        temp = temp->next;
        if (temp->anchor_addr == addr)
        {
            // Serial.println("find_link:Find addr");
            return temp;
        }
    }

    Serial.println("find_link:Can't find addr");
    return NULL;
}

void fresh_link(struct Link *p, uint16_t addr, float range, float dbm)
{
#ifdef DEBUG
    // Serial.println("fresh_link");
#endif
    struct Link *temp = find_link(p, addr);
    if (temp != NULL)
    {

        temp->range = range;
        temp->dbm = dbm;
        return;
    }
    else
    {
        Serial.println("fresh_link:Fresh fail");
        return;
    }
}

void print_link(struct Link *p)
{
#ifdef DEBUG
    Serial.println("print_link");
#endif
    struct Link *temp = p;

    while (temp->next != NULL)
    {
        // Serial.println("Dev %d:%d m", temp->next->anchor_addr, temp->next->range);
        Serial.println(temp->next->anchor_addr, HEX);
        Serial.println(temp->next->range);
        Serial.println(temp->next->dbm);
        temp = temp->next;
    }

    return;
}

void delete_link(struct Link *p, uint16_t addr)
{
#ifdef DEBUG
    Serial.println("delete_link");
#endif
    if (addr == 0)
        return;

    struct Link *temp = p;
    while (temp->next != NULL)
    {
        if (temp->next->anchor_addr == addr)
        {
            struct Link *del = temp->next;
            temp->next = del->next;
            free(del);
            return;
        }
        temp = temp->next;
    }
    return;
}

// SSD1306

void logoshow(void)
{
    display.clearDisplay();

    display.setTextSize(2);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.println(F("Team DT6"));

    display.setTextSize(1);
    display.setCursor(0, 20); // Start at top-left corner
    display.println(F("DW1000 DEMO"));
    display.display();
    delay(2000);
}

void display_uwb(struct Link *p)
{
    struct Link *temp = p;
    int row = 0;

    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);

    if (temp->next == NULL)
    {
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println("No Anchor");
        display.display();
        return;
    }

    while (temp->next != NULL)
    {
        temp = temp->next;

        // Serial.println("Dev %d:%d m", temp->next->anchor_addr, temp->next->range);
        // Serial.println(temp->anchor_addr, HEX);
        // Serial.println(temp->range);

        char c[30];

        // sprintf(c, "%X:%.1f m %.1f", temp->anchor_addr, temp->range, temp->dbm);
        // sprintf(c, "%X:%.1f m", temp->anchor_addr, temp->range);
        sprintf(c, "%.1f m", temp->range);
        display.setTextSize(1);
        display.setCursor(0, row++ * 16); // Start at top-left corner
        display.print(temp->anchor_addr, HEX);
        display.print("|");
        display.print(c);
        display.print("|");
        sprintf(c, "%.1f dbm", temp->dbm);
        display.print(c);

        // if (row >= 1)
        // {
        //     break;
        // }
    }
    delay(500);
    display.display();
    return;
}

String generate_raw_json(struct Link *p)
{
    // Create a JSON document to hold an array
    StaticJsonDocument<500> doc; // Increase size if needed
    JsonArray dataArray = doc.to<JsonArray>();

    struct Link *temp = p->next; // Skip the head node as it is an empty placeholder

    // Loop through each anchor and add its data to the JSON array
    while (temp != NULL)
    {
        // Create a temporary JSON object for this anchor
        JsonObject anchorData = dataArray.createNestedObject();
        anchorData["from"] = String(temp->anchor_addr, HEX);
        anchorData["range"] = temp->range;
        anchorData["rxPower"] = temp->dbm;

        temp = temp->next;
    }

    // Serialize the whole array into a JSON string
    String jsonString;
    serializeJson(doc, jsonString);

    return jsonString;
}

void send_data(String jsonString)
{
    WiFiClient client;
    IPAddress server(172, 20, 10, 4);
    const uint16_t port = 5001;

    // Now send the JSON array as one string
    if (client.connect(server, port))
    {
        client.println(jsonString);
        Serial.println("Data sent: " + jsonString);
    }
    else
    {
        Serial.println("Sending failed!");
    }
}

String check_section(const String &jsonData) {
    // Parse the JSON input according to the format generated by generate_raw_json
    StaticJsonDocument<500> doc;
    DeserializationError error = deserializeJson(doc, jsonData);
    if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return "";
    }

    JsonArray dataArray = doc.as<JsonArray>();

    // Temporary storage for detected anchors and their ranges
    const int maxAnchors = 10;  // Define maximum number of anchors
    uint16_t detectedAnchors[maxAnchors];
    float detectedRanges[maxAnchors];
    int numDetectedAnchors = 0;

    // Loop through JSON array to populate detected anchors and ranges
    for (JsonObject anchorData : dataArray) {
        String addrStr = anchorData["from"];
        uint16_t anchorAddr = strtol(addrStr.c_str(), nullptr, 16);  // Convert hex string to integer
        float range = anchorData["range"];

        if (numDetectedAnchors < maxAnchors) {
            detectedAnchors[numDetectedAnchors] = anchorAddr;
            detectedRanges[numDetectedAnchors] = range;
            numDetectedAnchors++;
        }
    }

    // Create a JSON document for possible sections
    StaticJsonDocument<500> outputDoc;
    JsonArray possibleSections = outputDoc.to<JsonArray>();

    // Check each predefined track section
    for (int i = 0; i < sizeof(trackSections) / sizeof(trackSections[0]); i++) {
        bool foundAnchor1 = false, foundAnchor2 = false;
        float dT_i = 0, dT_j = 0;

        // Find matching anchors and corresponding ranges
        for (int j = 0; j < numDetectedAnchors; j++) {
            if (detectedAnchors[j] == trackSections[i].anchor1_addr) {
                foundAnchor1 = true;
                dT_i = detectedRanges[j];
            }
            if (detectedAnchors[j] == trackSections[i].anchor2_addr) {
                foundAnchor2 = true;
                dT_j = detectedRanges[j];
            }
        }

        // If both anchors are detected, check conditions
        if (foundAnchor1 && foundAnchor2) {
            float d_ij = trackSections[i].d_ij;
            float s_ij = trackSections[i].s_ij;
            float dT_ij = dT_i + dT_j;

            float theta = acos(((d_ij * d_ij) - (dT_i * dT_i) - (dT_j * dT_j)) / (-2 * dT_i * dT_j));
            // calculated estimated arc length
            float sT_ij = sqrt((d_ij * d_ij) / (2 - (2 * cos(theta))));

            // Triangle inequality check
            if (dT_ij >= (d_ij - tolerance)) {
              if (d_ij == s_ij) {
                if ((dT_ij >= (d_ij - tolerance)) && (dT_ij <= (d_ij + tolerance))) {
                // Both the straight check and the triangle inequality check pass
                  JsonObject section = possibleSections.createNestedObject();
                  section["anchor1"] = String(trackSections[i].anchor1_addr, HEX);
                  section["anchor2"] = String(trackSections[i].anchor2_addr, HEX);
                  section["isBooked"] = trackSections[i].isBooked;
                }
              } else if (s_ij > d_ij) {
                if (((d_ij - tolerance) <= dT_ij) && (dT_ij < (s_ij + tolerance))) {
                // Both the curved check and the triangle inequality check pass
                  // Approximate curve check
                  if ((sT_ij >= (s_ij - tolerance)) && (sT_ij <= (s_ij + tolerance))) {
                    JsonObject section = possibleSections.createNestedObject();
                    section["anchor1"] = String(trackSections[i].anchor1_addr, HEX);
                    section["anchor2"] = String(trackSections[i].anchor2_addr, HEX);
                    section["isBooked"] = trackSections[i].isBooked;
                  }
                }
              }
            }
        }
    }

    // Serialize the JSON document to a string and return it
    String jsonOutput;
    serializeJson(outputDoc, jsonOutput);
    return jsonOutput;
}

void sound_alarm(void) {
  digitalWrite(ALARM_PIN, HIGH);
  delay(500);
  digitalWrite(ALARM_PIN, LOW);
  delay(500);
  digitalWrite(ALARM_PIN, HIGH);
  delay(500);
  digitalWrite(ALARM_PIN, LOW);
}

void display_warning(void) {
    display.clearDisplay();

    display.setTextSize(2);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.println(F("Warning"));

    display.setTextSize(1);
    display.setCursor(0, 20); // Start at top-left corner
    display.println(F("Outside booked zones"));
    display.display();
    delay(2000);
}

void check_booking(const String &jsonData) {
    // Parse the JSON output from check_section
    StaticJsonDocument<500> doc;
    DeserializationError error = deserializeJson(doc, jsonData);
    if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return;
    }

    // Extract the array of possible sections from the parsed JSON
    JsonArray possibleSections = doc.as<JsonArray>();

    // Loop through the possible sections to check if they are booked
    bool alarmTriggered = false;
    for (JsonObject section : possibleSections) {
        // Extract anchor addresses for this section
        String anchor1Str = section["anchor1"];
        String anchor2Str = section["anchor2"];
        uint16_t anchor1Addr = strtol(anchor1Str.c_str(), nullptr, 16);
        uint16_t anchor2Addr = strtol(anchor2Str.c_str(), nullptr, 16);
        bool isBooked = section["isBooked"];

        // Check if this section is not booked
        if (!isBooked) {
            // If the section is not booked, sound the alarm and display the warning
            if (!alarmTriggered) {
                sound_alarm();
                alarmTriggered = true; // Ensure alarm is triggered only once
            }
            display_warning();
            break; // Exit after the first unbooked section is found
        }
    }
}
