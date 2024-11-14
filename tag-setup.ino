#include <SPI.h>
#include "DW1000Ranging.h"
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

#define TAG_ADDR "7D:00:22:EA:82:60:3B:9F"

// #define DEBUG

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define UWB_RST 27  // reset pin
#define UWB_IRQ 34  // irq pin
#define UWB_SS 21   // spi select pin

#define I2C_SDA 4
#define I2C_SCL 5

#define ALARM_PIN 0

const char *ssid = "ssid";          // Your Wi-Fi SSID
const char *password = "password";  // Your Wi-Fi password
int maxRetries = 8;
int retryCount = 0;
unsigned long retryDelay = 5000;

struct AnchorData {
  uint16_t anchor_addr;
  float range;
  float dbm;
};

struct TrackSection {
  uint16_t anchor1_addr;
  uint16_t anchor2_addr;
  float d_ij;  // Straight line distance between anchors
  float s_ij;  // Arc length for curved sections
  bool isBooked;
};

// Example predefined track sections
TrackSection trackSections[] = {
  { 0x88CC, 0x983F, 45.7, 45.7, true },  // Example values
  { 0x983F, 0x1786, 31.0, 31.0, false }  // Example values
};

AnchorData detectedAnchors[10] = {};

TrackSection validTrackSections[10]{};

TrackSection bookedTrackSections[]{
  { 0x88CC, 0x983F, 45.7, 45.7, true }
};

struct Link {
  uint16_t anchor_addr;
  float range;
  float dbm;
  struct Link *next;
};

struct Link *uwb_data;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(1000);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();

  logoshow();


  // init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(UWB_RST, UWB_SS, UWB_IRQ);  // Reset, CS, IRQ pin
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

void loop() {
  DW1000Ranging.loop();
  if ((millis() - runtime) > 1000) {
    display_uwb(uwb_data);
    get_detected_anchors(uwb_data);
    // if (detectedAmchors contains one or more entry) {
    get_valid_sections();
    check_booking();
    // }
    if (WiFi.status() == WL_CONNECTED) {
      send_calculated_data();
      // send_raw_data(uwb_data);
    }
    clear_valid_sections();
    clear_detected_anchors();

    runtime = millis();
  }
}

void newRange() {
  // Serial.print("from: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  // Serial.print("\t Range: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  // Serial.print(" m");
  // Serial.print("\t RX power: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  // Serial.println(" dBm");

  fresh_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange(), DW1000Ranging.getDistantDevice()->getRXPower());
  // print_link(uwb_data);
}

void newDevice(DW1000Device *device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);

  add_link(uwb_data, device->getShortAddress());
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);

  delete_link(uwb_data, device->getShortAddress());
}

// Data Link

struct Link *init_link() {
#ifdef DEBUG
  Serial.println("init_link");
#endif
  struct Link *p = (struct Link *)malloc(sizeof(struct Link));
  p->next = NULL;
  p->anchor_addr = 0;
  p->range = 0.0;

  return p;
}

void add_link(struct Link *p, uint16_t addr) {
#ifdef DEBUG
  Serial.println("add_link");
#endif
  struct Link *temp = p;
  // Find struct Link end
  while (temp->next != NULL) {
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

struct Link *find_link(struct Link *p, uint16_t addr) {
#ifdef DEBUG
  // Serial.println("find_link");
#endif
  if (addr == 0) {
    Serial.println("find_link:Input addr is 0");
    return NULL;
  }

  if (p->next == NULL) {
    Serial.println("find_link:Link is empty");
    return NULL;
  }

  struct Link *temp = p;
  // Find target struct Link or struct Link end
  while (temp->next != NULL) {
    temp = temp->next;
    if (temp->anchor_addr == addr) {
      // Serial.println("find_link:Find addr");
      return temp;
    }
  }

  Serial.println("find_link:Can't find addr");
  return NULL;
}

void fresh_link(struct Link *p, uint16_t addr, float range, float dbm) {
#ifdef DEBUG
  // Serial.println("fresh_link");
#endif
  struct Link *temp = find_link(p, addr);
  if (temp != NULL) {

    temp->range = range;
    temp->dbm = dbm;
    return;
  } else {
    Serial.println("fresh_link:Fresh fail");
    return;
  }
}

void print_link(struct Link *p) {
#ifdef DEBUG
  Serial.println("print_link");
#endif
  struct Link *temp = p;

  while (temp->next != NULL) {
    // Serial.println("Dev %d:%d m", temp->next->anchor_addr, temp->next->range);
    Serial.println(temp->next->anchor_addr, HEX);
    Serial.println(temp->next->range);
    Serial.println(temp->next->dbm);
    temp = temp->next;
  }

  return;
}

void delete_link(struct Link *p, uint16_t addr) {
#ifdef DEBUG
  Serial.println("delete_link");
#endif
  if (addr == 0)
    return;

  struct Link *temp = p;
  while (temp->next != NULL) {
    if (temp->next->anchor_addr == addr) {
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

void logoshow(void) {
  display.clearDisplay();

  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("Team DT6"));

  display.setTextSize(1);
  display.setCursor(0, 20);  // Start at top-left corner
  display.println(F("DW1000"));
  display.display();
  delay(2000);
}

void display_uwb(struct Link *p) {
  struct Link *temp = p;
  int row = 0;

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  if (temp->next == NULL) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("No Anchor");
    display.display();
    return;
  }

  while (temp->next != NULL) {
    temp = temp->next;

    // Serial.println("Dev %d:%d m", temp->next->anchor_addr, temp->next->range);
    // Serial.println(temp->anchor_addr, HEX);
    // Serial.println(temp->range);

    char c[30];

    // sprintf(c, "%X:%.1f m %.1f", temp->anchor_addr, temp->range, temp->dbm);
    // sprintf(c, "%X:%.1f m", temp->anchor_addr, temp->range);
    sprintf(c, "%.1f m", temp->range);
    display.setTextSize(1);
    display.setCursor(0, row++ * 16);  // Start at top-left corner
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

void get_detected_anchors(struct Link *p) {
  int index = 0;  // Variable to keep track of the index in the detectedAnchors array

  // Traverse the linked list starting from the first detected anchor
  struct Link *temp = p->next;  // Skip the dummy head node

  // Loop through the linked list and extract the detected anchor data
  while (temp != NULL) {
    // Check if there's room in the detectedAnchors array (assumed size limit)
    if (index < sizeof(detectedAnchors) / sizeof(detectedAnchors[0])) {
      // Populate the AnchorData struct for each detected anchor
      detectedAnchors[index].anchor_addr = temp->anchor_addr;
      detectedAnchors[index].range = temp->range;
      detectedAnchors[index].dbm = temp->dbm;

      // Move to the next anchor in the linked list
      temp = temp->next;
      index++;  // Increment the index to store the next detected anchor
    } else {
      // Exit the loop if the array is full
      break;
    }
  }
  // The detectedAnchors array is now populated and can be used elsewhere in the program.
  Serial.print("Total Detected Anchors: ");
  Serial.println(index);
}

// void get_valid_sections(void) {
//   int numDetectedAnchors = sizeof(detectedAnchors) / sizeof(detectedAnchors[0]);

//   int index = 0;  // Track index for validTrackSections

//   // Loop through each predefined track section to see if it matches detected anchors
//   for (int i = 0; i < sizeof(trackSections) / sizeof(trackSections[0]); i++) {
//     bool foundAnchor1 = false, foundAnchor2 = false;
//     float dT_i = 0, dT_j = 0;

//     // Loop through detected anchors to find matches with track section anchors
//     for (int j = 0; j < numDetectedAnchors; j++) {
//       if (detectedAnchors[j].anchor_addr == trackSections[i].anchor1_addr) {
//         foundAnchor1 = true;
//         dT_i = detectedAnchors[j].range;
//       }
//       if (detectedAnchors[j].anchor_addr == trackSections[i].anchor2_addr) {
//         foundAnchor2 = true;
//         dT_j = detectedAnchors[j].range;
//       }
//     }

//     // If both anchors for this section are detected, proceed with distance checks
//     if (foundAnchor1 && foundAnchor2) {
//       float d_ij = trackSections[i].d_ij;
//       float s_ij = trackSections[i].s_ij;
//       float dT_ij = dT_i + dT_j;

//       float theta = (2 * PI) - (2 * acos(((d_ij * d_ij) - (dT_i * dT_i) - (dT_j * dT_j)) / (-2 * dT_i * dT_j)));
//       float sT_ij = (sqrt((d_ij * d_ij) / (2 - (2 * cos(theta))))) * theta;

//       // Set tolerances for straight and curved section checks
//       float straightTolerance = 0.02 * d_ij;
//       float arcTolerance = 0.02 * s_ij;

//       // Triangle inequality check
//       if (dT_ij >= (d_ij - straightTolerance)) {
//         // Check if section is straight
//         if (d_ij == s_ij) {
//           if ((dT_ij >= (d_ij - straightTolerance)) && (dT_ij <= (d_ij + straightTolerance))) {
//             // Section is straight and matches
//             validTrackSections[index++] = trackSections[i];
//           }
//         }
//         // Check if section is curved
//         else if (s_ij > d_ij) {
//           if ((dT_ij >= (d_ij - straightTolerance)) && (dT_ij < (s_ij + arcTolerance)) && (sT_ij >= (s_ij - arcTolerance)) && (sT_ij <= (s_ij + arcTolerance))) {
//             // Section is curved and matches
//             validTrackSections[index++] = trackSections[i];
//           }
//         }
//       }
//     }
//   }
//   Serial.print("Total Valid Sections: ");
//   Serial.println(index);
// }

void get_valid_sections(void) {
  int detectedAnchorCount = 0;

  // Count the actual number of detected anchors (non-zero anchor_addr)
  for (int i = 0; i < sizeof(detectedAnchors) / sizeof(detectedAnchors[0]); i++) {
    if (detectedAnchors[i].anchor_addr != 0) {
      detectedAnchorCount++;
    }
  }

  int index = 0;  // Track index for validTrackSections

  // Loop through each predefined track section
  for (int i = 0; i < sizeof(trackSections) / sizeof(trackSections[0]); i++) {
    bool foundAnchor1 = false, foundAnchor2 = false;
    float dT_i = 0, dT_j = 0;

    // Match each detected anchor to the track section anchors
    for (int j = 0; j < detectedAnchorCount; j++) {
      if (detectedAnchors[j].anchor_addr == trackSections[i].anchor1_addr) {
        foundAnchor1 = true;
        dT_i = detectedAnchors[j].range;
      }
      if (detectedAnchors[j].anchor_addr == trackSections[i].anchor2_addr) {
        foundAnchor2 = true;
        dT_j = detectedAnchors[j].range;
      }
    }

    // Proceed only if both anchors for this section are detected
    if (foundAnchor1 && foundAnchor2) {
      float d_ij = trackSections[i].d_ij;
      float s_ij = trackSections[i].s_ij;
      float dT_ij = dT_i + dT_j;

      // Check if section is straight or is at start of a curved section thus can use the straight line check
      if ((d_ij == s_ij) || (dT_i == 0) || (dT_j == 0)) {
        float straightTolerance = 0.05 * d_ij;
        if (dT_ij >= (d_ij - straightTolerance) && dT_ij <= (d_ij + straightTolerance)) {
          validTrackSections[index++] = trackSections[i];
        }
      }
      // Check if section is curved
      else if (s_ij > d_ij) {
        // Calculate the angle theta for curved path checks and estimated arc length
        float theta = (2 * PI) - (2 * acos(((d_ij * d_ij) - (dT_i * dT_i) - (dT_j * dT_j)) / (-2 * dT_i * dT_j)));
        float sT_ij = (sqrt((d_ij * d_ij) / (2 - (2 * cos(theta))))) * theta;
        float arcTolerance = 0.02 * s_ij;

        if ((dT_ij >= (d_ij - arcTolerance) && dT_ij <= (s_ij + arcTolerance)) && (sT_ij >= (s_ij - arcTolerance) && sT_ij <= (s_ij + arcTolerance))) {
          validTrackSections[index++] = trackSections[i];
        }
      }
    }
  }

  Serial.print("Total Valid Sections: ");
  Serial.println(index);
}


void send_calculated_data() {
  WiFiClient client;
  IPAddress server(172, 20, 10, 4);
  const uint16_t port = 5001;

  // Create a JSON document to hold an array
  StaticJsonDocument<500> doc;  // Adjust size if needed
  JsonArray dataArray = doc.to<JsonArray>();

  // Iterate through each item in validTrackSections
  for (int i = 0; i < sizeof(validTrackSections) / sizeof(validTrackSections[0]); i++) {
    TrackSection &section = validTrackSections[i];

    // Only send data for occupied sections
    if (section.anchor1_addr != 0 && section.anchor2_addr != 0) {
      JsonObject sectionData = dataArray.createNestedObject();
      sectionData["anchor1_addr"] = String(section.anchor1_addr, HEX);
      sectionData["anchor2_addr"] = String(section.anchor2_addr, HEX);
      // sectionData["d_ij"] = section.d_ij;
      // sectionData["s_ij"] = section.s_ij;
      sectionData["isBooked"] = section.isBooked;
    }
  }

  // Serialize the JSON array to a string
  String jsonString;
  serializeJson(doc, jsonString);

  // Send the JSON array as one string if connected
  if (client.connect(server, port)) {
    client.println(jsonString);
    Serial.println("Data sent: " + jsonString);
  } else {
    Serial.println("Sending failed!");
  }
}

void check_booking() {
  bool warningTriggered = false;
  // Loop through each valid track section
  for (int i = 0; i < sizeof(validTrackSections) / sizeof(validTrackSections[0]); i++) {
    TrackSection &section = validTrackSections[i];

    // Check if the section is occupied (non-zero anchor addresses)
    if (section.anchor1_addr != 0 && section.anchor2_addr != 0) {
      bool isBookedSection = false;

      // Loop through bookedTrackSections to see if this section is booked
      for (int j = 0; j < sizeof(bookedTrackSections) / sizeof(bookedTrackSections[0]); j++) {
        TrackSection &bookedSection = bookedTrackSections[j];

        // Check if the current valid section matches a booked section
        if (section.anchor1_addr == bookedSection.anchor1_addr && section.anchor2_addr == bookedSection.anchor2_addr) {
          isBookedSection = true;
          break;
        }
      }

      // Trigger warning if the section is not booked or if `isBooked` is false
      if (!isBookedSection || !section.isBooked) {
        warningTriggered = true;
        break;
      }
    }
  }

  // If any section requires a warning, display it and sound the alarm
  if (warningTriggered) {
    display_warning();
    sound_alarm();
  }
}

void send_raw_data(struct Link *p) {
  WiFiClient client;
  IPAddress server(172, 20, 10, 2);
  // IPAddress server(172, 20, 10, 4);
  const uint16_t port = 5001;

  // Create a JSON document to hold an array
  StaticJsonDocument<500> doc;  // Increase size if needed
  JsonArray dataArray = doc.to<JsonArray>();

  struct Link *temp = p->next;  // Skip the head node as it is an empty placeholder

  // Loop through each anchor and add its data to the JSON array
  while (temp != NULL) {
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

  // Now send the JSON array as one string
  if (client.connect(server, port)) {
    client.println(jsonString);
    Serial.println("Data sent: " + jsonString);
  } else {
    Serial.println("Sending failed!");
  }
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

  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("Warning"));

  display.setTextSize(1);
  display.setCursor(0, 20);  // Start at top-left corner
  display.println(F("Outside booked zones"));
  display.display();
  delay(2000);
}

void clear_detected_anchors() {
  for (int i = 0; i < 10; i++) {
    detectedAnchors[i].anchor_addr = 0;
    detectedAnchors[i].range = 0.0f;
    detectedAnchors[i].dbm = 0.0f;
  }
}

void clear_valid_sections() {
  for (int i = 0; i < 10; i++) {
    // Reset each TrackSection field to default values
    validTrackSections[i].anchor1_addr = 0;
    validTrackSections[i].anchor2_addr = 0;
    validTrackSections[i].d_ij = 0.0f;
    validTrackSections[i].s_ij = 0.0f;
    validTrackSections[i].isBooked = false;
    // Add any other fields in TrackSection that need to be reset to default values
  }
}
