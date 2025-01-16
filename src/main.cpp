#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <Ticker.h>
#include <PubSubClient.h>
#include <qrCode.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>


#define SD_CS_PIN 5

#define MQTT_BROKER  "broker.emqx.io"
#define MQTT_PORT 1883

#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11

#define R_SENSE 0.11
#define EN_PIN 8
#define DIR_PIN 15
#define STEP_PIN 17

#define LED_BUILDIN 2

DHT dht(DHTPIN, DHTTYPE);

// UART Configuration for TMC2208 using HardwareSerial
HardwareSerial MySerial(1);  // Use UART1 on ESP32
TMC2208Stepper driver = TMC2208Stepper(&MySerial, R_SENSE); // Initialize with hardware serial
constexpr uint32_t steps_per_mm = 80; // Configure steps per mm as needed

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
unsigned long lastChangeTime = 0;
bool directionCW = true;

Servo servo;

WiFiManager wifiManager;

String BaseUrl = "http://135.237.185.240/api"; 
// String BaseUrl = "http://127.0.0.1:8000/api";
String RegisterUrl = BaseUrl + "/device/register";

String DeviceUID;
int DeviceID;
int temperature;
int currentState;

void registerDevice();
// void downloadImage();
bool tftRenderJpeg(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap);
void uploadTemperature(double temperature);
void saveUIDToSD(const char* uid);
void saveIDtoSD(int id);
String readUIDfromSD();
int readDeviceIDFromSD();
void mqttCallback(char* topic, byte* payload, unsigned int len);
boolean mqttConnect();
void mqttLoop();
void reconnectWIFI();
void mqttReconnect();
void readTemperature();
void setupStepper();
void rotateStepper(int direction);
int findShortestPath(int current, int target, int totalContainers);
void rotateStepperToState(int direction, int steps);

int ContainerState;

// Initialize TFT screen
TFT_eSPI tft = TFT_eSPI();

TFT_eSprite sprite = TFT_eSprite(&tft);

int imageW=320; // image size must be the same size with the creen or clsoe enough
int imageH=480;

int screenW=320;
int screenH=480;
int m=imageW;
int start=1;
//unsigned short imageS[60000]={0}; // edit this screenW * screen H // u can oversize it a bit

char g_szDeviceId[30];
Ticker tickerTimer;
WiFiClient espClient;
PubSubClient mqtt(espClient);

void setup() {
    Serial.begin(115200);
    MySerial.begin(115200, SERIAL_8N1, 4, 5); // Initialize UART on GPIO 4 (RX) and GPIO 5 (TX)

    pinMode(1, INPUT);
    pinMode(2, OUTPUT);
    pinMode(20, INPUT);
    
    // set pin 2 high
    digitalWrite(2, HIGH);

    servo.attach(37); // Attach the servo to GPIO 37

    servo.write(130);

    Serial.println("Starting Stepper Motor");

    setupStepper();  // Initialize the stepper motor

    for (size_t i = 0; i < 2; i++)
    {
        rotateStepper(1); // Rotate the stepper motor
    }

    // Initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card failed to initialize.");
        return;
    }
    Serial.println("SD Card initialized.");

    // Check if already connected to WiFi
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Not connected to WiFi. Starting WiFiManager...");
        // Start WiFiManager portal on demand
        wifiManager.startConfigPortal("MinderAP");
    }

    // If you get here you have connected to the WiFi
    Serial.println("Connected to WiFi!");
    Serial.println("IP Address: ");
    Serial.println(WiFi.localIP());

    // Check if the device has already been registered
    DeviceUID = readUIDfromSD();
    if (DeviceUID == "") {
        registerDevice();
    } else
    {
        Serial.println("Device already registered with UID: " + DeviceUID);
        DeviceID = readDeviceIDFromSD();
    }

    // MQTT
    mqttConnect();
    mqtt.setCallback(mqttCallback);
}

void loop() {
    mqttLoop();
    readTemperature();
    if (WiFi.status() != WL_CONNECTED) {
        reconnectWIFI();
    }
    delay(10);
}

void registerDevice() {
    // Create a JSON object to send to the server (if required)
    StaticJsonDocument<200> doc;
    // Add any required fields here for the backend to process the registration.
    // doc["key"] = "value"; (if needed)

    // Serialize the JSON object
    String json;
    serializeJson(doc, json);

    // Send the JSON object to the server
    HTTPClient http;
    http.begin(RegisterUrl);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(json);

    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println("Response: " + response);

        // Parse the JSON response to extract the `uid`
        StaticJsonDocument<300> responseDoc;
        DeserializationError error = deserializeJson(responseDoc, response);
        if (error) {
            Serial.println("Error parsing JSON response");
            return;
        }

        // Extract the `uid` field from the JSON response
        const char* uid = responseDoc["device"]["uid"];
        const int id = responseDoc["device"]["id"];
        if (uid) {
            Serial.println("Extracted UID: " + String(uid));
            DeviceUID = String(uid);
            DeviceID = id;
            saveUIDToSD(uid);
            saveIDtoSD(id);
        } else {
            Serial.println("UID not found in response");
        }
    } else {
        Serial.println("Error on HTTP request");
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println(http.getString());
    }
    http.end();
}


// TODO: fix this function
// void downloadImage() {
//     HTTPClient http;
//     String qrCodeUrl = "http://135.237.185.240/api/device/624bf917-ebda-4961-bdaf-5ffd5485ea92/qr-code";
//     // String qrCodeUrl = BaseUrl + "/device/" + DeviceUID + "/qr-code";

//     qrCodeUrl.trim(); // Clean the URL

//     String contentType = http.header("Content-Type");
//     Serial.println("Content-Type: " + contentType);


//     Serial.print("Fetching image from: ");
//     Serial.println(qrCodeUrl);

//     http.begin(qrCodeUrl); // Start connection to the image URL
//     int httpCode = http.GET(); // Send GET request

//     if (httpCode == 200) { // HTTP 200 means the request was successful
//         Serial.println("Image fetched successfully");

//         // Open a file on the SD card to write the image
//         File imageFile = SD.open("/qr-code.png", FILE_WRITE);
//         if (!imageFile) {
//             Serial.println("Failed to open file for writing");
//             return;
//         }

//         // Get the image content and save it to the SD card
//         WiFiClient *stream = http.getStreamPtr();
//         byte buffer[128]; // Buffer to store incoming data
//         int bytesRead;
//         int totalBytesRead = 0;
//         while ((bytesRead = stream->read(buffer, sizeof(buffer))) > 0) {
//             imageFile.write(buffer, bytesRead); // Write data to file
//             totalBytesRead += bytesRead;
//         }

//         // Close the file after writing
//         imageFile.close();
//         Serial.println("Image saved to SD card");

//         // Log the total number of bytes downloaded
//         Serial.print("Total bytes downloaded: ");
//         Serial.println(totalBytesRead);

//     } else {
//         Serial.println("Failed to fetch image, HTTP code: " + String(httpCode));
//     }

//     // Close the HTTP connection
//     http.end();
// }


void saveUIDToSD(const char* uid) {
    // Open file for writing on SD card with .txt extension
    File file = SD.open("/uid.txt", FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing.");
        return;
    }

    // Write the UID to the file
    file.println(uid);

    // Close the file
    file.close();
    Serial.println("UID saved to SD card successfully!");
}

String readUIDfromSD() {
    // Open file for reading on SD card
    File file = SD.open("/uid.txt", FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading.");
        return "";
    }

    // Read the UID from the file
    String uid = file.readString();

    // Close the file
    file.close();
    return uid;
}

void saveIDtoSD(int id) {
    // Open file for writing on SD card with .txt extension
    File file = SD.open("/id.txt", FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing.");
        return;
    }

    // Write the ID to the file
    file.println(id);

    // Close the file
    file.close();
    Serial.println("ID saved to SD card successfully!");
}

int readDeviceIDFromSD() {
    // Open file for reading on SD card
    File file = SD.open("/id.txt", FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading.");
        return -1;
    }

    // Read the ID from the file
    int id = file.parseInt();

    // Close the file
    file.close();
    return id;
}

// Function to render JPEG on TFT screen
bool tftRenderJpeg(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
    tft.pushImage(x, y, w, h, bitmap);
    return 1;
}

void uploadTemperature(double temperature) {
    // Create a JSON object to send to the server (if required)
    StaticJsonDocument<200> doc;
    doc["temperature"] = temperature;

    // Serialize the JSON object
    String json;
    serializeJson(doc, json);

    // Send the JSON object to the server
    HTTPClient http;
    http.begin(BaseUrl + "/device/temperature/" + DeviceID);
    http.addHeader("Content-Type", "application/json");
    // print url
    Serial.println(BaseUrl + "/device/temperature/" + DeviceID);
    int httpResponseCode = http.POST(json);

    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println("Response: " + response);
    } else {
        Serial.println("Error on HTTP request");
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println(http.getString());
    }
    http.end();
}

void reconnectWIFI() {
    // Check if already connected to WiFi
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Not connected to WiFi. Starting WiFiManager...");
        // Start WiFiManager portal on demand
        wifiManager.startConfigPortal("MinderAP");
    }

    // If you get here you have connected to the WiFi
    Serial.println("Connected to WiFi!");
    Serial.println("IP Address: ");
    Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload, len);
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.write(payload, len);
    digitalWrite(2, payload[0]-'0');
    digitalWrite(3, payload[1]-'0');

    if (error) {
        Serial.print("JSON deserialization failed: ");
        Serial.println(error.f_str());
        return;  // Exit if JSON parsing fails
    }

    if (String(topic).equals("device/" + DeviceUID + "/notification")) {
        Serial.print("Message arrived [");
        Serial.print(topic);
        Serial.print("]: ");
        Serial.write(payload, len);
        digitalWrite(2, payload[0]-'0');
        digitalWrite(3, payload[1]-'0');    
    } else if (String(topic).equals("device/" + DeviceUID + "/state"))
    {
        Serial.print("Message arrived [");
        Serial.print(topic);
        Serial.print("]: ");
        Serial.write(payload, len);
        digitalWrite(2, payload[0]-'0');
        digitalWrite(3, payload[1]-'0');

        digitalWrite(LED_BUILDIN, HIGH);

        int targetState = doc["current_state"];  // Extract target state from message

        // check if the target state is different from the current state set delay
        // if (targetState != currentState) {
        //     delay(1000);
        // }

        Serial.println(targetState);

        // Find the shortest path to the target state
        int shortestPath = findShortestPath(currentState, targetState, 4);
        int direction = shortestPath > 0 ? 1 : -1;

        Serial.println("Shortest path: " + String(shortestPath));
        Serial.println("Direction: " + String(direction));

        // Rotate the stepper motor in the shortest path direction
        rotateStepperToState(direction, shortestPath);

        currentState = targetState;  // Update the current state

        digitalWrite(LED_BUILDIN, HIGH);
    }
    


    Serial.println();
}

boolean mqttConnect() { 
  sprintf(g_szDeviceId, "esp32_%08X",(uint32_t)ESP.getEfuseMac());
  mqtt.setServer(MQTT_BROKER, 1883);
  mqtt.setCallback(mqttCallback);
  Serial.printf("Connecting to %s clientId: %s\n", MQTT_BROKER, g_szDeviceId);

  boolean fMqttConnected = false;
  for (int i=0; i<3 && !fMqttConnected; i++) {
    Serial.println("Connecting to mqtt broker...");
    fMqttConnected = mqtt.connect(g_szDeviceId);
    if (fMqttConnected == false) {
      Serial.print(" fail, rc=");
      Serial.println(mqtt.state());
      delay(1000);
    }
  }
  if (fMqttConnected)
  {
    Serial.println(" success");
    DeviceUID.trim();

    String notifTopic = "device/" + DeviceUID + "/notification";
    String stateTopic = "device/" + DeviceUID + "/state";
    mqtt.subscribe(notifTopic.c_str());
    mqtt.subscribe(stateTopic.c_str());
  }
  return mqtt.connected();
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(g_szDeviceId)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqttLoop() {
  if (!mqtt.connected()) {
    mqttReconnect();
  }
  mqtt.loop();
}

void readTemperature() {
    float humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float temperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
    // Compute heat index in Celsius (isFahrenheit = false)
    float heatIndexC = dht.computeHeatIndex(temperature, humidity, false);

    // update temperature at first time and check if it is changed to update it
    if (start || (int)temperature != (int)temperature) {
        start=0;
        // update temperature
        temperature = (int)temperature;
        // print temperature
        Serial.print(F("Humidity: "));
        Serial.print(humidity);
        Serial.print(F("%  Temperature: "));
        Serial.print(temperature);
        Serial.print(F("°C "));
        Serial.print(F("Heat index: "));
        Serial.print(heatIndexC);
        Serial.print(F("°C "));
        // upload temperature to server
        uploadTemperature(temperature);
    }
}

void setupStepper() {
    driver.begin();                           // Initialize the TMC2208
    driver.toff(5);                           // Enable the driver (set toff to a non-zero value)
    driver.rms_current(800);                  // Set motor current in mA
    driver.microsteps(16);                    // Set microstepping
    driver.pwm_autoscale(1);                  // Enable StealthChop (quiet stepping)

    // AccelStepper configuration
    stepper.setMaxSpeed(50 * steps_per_mm);         // Max speed
    stepper.setAcceleration(1000 * steps_per_mm);   // Acceleration
    stepper.setEnablePin(EN_PIN);                   // Set enable pin to GPIO 18
    stepper.setPinsInverted(false, false, true);    // Invert EN pin only
    stepper.enableOutputs();                        // Enable motor output
}

// rotate the stepper motor clockwise continuously
void rotateStepper(int direction) {
    int currentVal = digitalRead(1);
    unsigned long startTime = millis(); // Record the start time
    const unsigned long timeout = 5000; // Timeout after 5 seconds

    do {
        while (currentVal == LOW && (millis() - startTime) < timeout) {
            if (stepper.distanceToGo() == 0) {
                stepper.disableOutputs();
                stepper.move(91 * steps_per_mm * direction);
                stepper.enableOutputs();
            }
            stepper.run();
            currentVal = digitalRead(1);
        }

        if (stepper.distanceToGo() == 0) {
            stepper.disableOutputs();
            stepper.move(91 * steps_per_mm * direction);
            stepper.enableOutputs();
        }
        stepper.run();
        currentVal = digitalRead(1);

        // Check for timeout
        if (millis() - startTime >= timeout) {
            Serial.println("rotateStepper timed out!");
            break;
        }
    } while (currentVal == HIGH);

    stepper.move(0); // Stop the stepper motor
}


void rotateServo() {
    servo.write(180);
    for (int i = 0; i < 180; i++) {
        servo.write(i);
        delay(15);
    }
    for (int i = 180; i > 0; i--) {
        servo.write(i);
        delay(15);
    }
}

int findShortestPath(int current, int target, int totalContainers) {
    // Calculate the clockwise distance
    int clockwise = (target - current + totalContainers) % totalContainers;

    // Calculate the counter-clockwise distance
    int counterClockwise = (current - target + totalContainers) % totalContainers;

    // Return the shorter distance (or decide direction based on use case)
    if (clockwise <= counterClockwise) {
        Serial.print("Shortest path is clockwise. Steps: ");
        return clockwise; // Rotate clockwise
    } else {
        Serial.print("Shortest path is counter-clockwise. Steps: ");
        return -counterClockwise; // Negative for counter-clockwise
    }
}

// Function to rotate stepper motor based on direction and steps
void rotateStepperToState(int direction, int steps) {
  for (int i = 0; i < steps; i++) {
    rotateStepper(direction);
    rotateStepper(direction);   //TODO: remove this line when sensor is fixed
    delay(10);
    rotateServo();
  }
}
