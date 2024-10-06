#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <WiFiClientSecure.h>
#include <secrets.h>  // Include your WiFi and WebSocket server credentials

// Initialize WebSocket client with a secure WiFi client
WiFiClientSecure wifiClient;  // Secure client for TLS
WebSocketsClient webSocket;

unsigned long previousMillis = 0;
const long sendInterval = 10;  // Send data every 10ms

// Connect to WiFi function
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }

  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
}

// WebSocket event handler
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("Disconnected from WebSocket server");
      break;
    case WStype_CONNECTED:
      Serial.println("Connected to WebSocket server over TLS");
      break;
    case WStype_TEXT:
      Serial.printf("Message from server: %s\n", payload);
      break;
    case WStype_BIN:
      Serial.println("Binary message received");
      break;
    default:
      break;
  }
}

// Function to send TRNG data in binary format over WebSocket
void sendTRNGDataBinary() {
  uint32_t randomValue = esp_random();  // Generate random number from the ESP32 TRNG
  uint8_t binaryData[4];
  
  // Split the 32-bit random value into 4 bytes
  binaryData[0] = (randomValue >> 24) & 0xFF;
  binaryData[1] = (randomValue >> 16) & 0xFF;
  binaryData[2] = (randomValue >> 8) & 0xFF;
  binaryData[3] = randomValue & 0xFF;

  // Send the 4-byte binary data as a WebSocket message
  webSocket.sendBIN(binaryData, sizeof(binaryData));
}

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi
  connectToWiFi();

  // Initialize WiFiClientSecure with the Root CA certificate
  wifiClient.setCACert(rootCACertificate);
  
  // Initialize WebSocket client
  webSocket.beginSSL(websocketServerHost, websocketServerPort, websocketServerPath);
  
  // Set up WebSocket events and auto-reconnect
  webSocket.onEvent(webSocketEvent);  
  webSocket.setReconnectInterval(5000);  // Reconnect every 5 seconds if disconnected
  webSocket.enableHeartbeat(15000, 3000, 2);  // Heartbeat for ping-pong mechanism
}

void loop() {
  webSocket.loop();  // Maintain WebSocket connection
  
  // Check if WiFi is still connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost connection, reconnecting...");
    connectToWiFi();
  }

  // Send data periodically
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sendInterval) {
    previousMillis = currentMillis;
    if (webSocket.isConnected()) {
      sendTRNGDataBinary();
    }
  }
}
