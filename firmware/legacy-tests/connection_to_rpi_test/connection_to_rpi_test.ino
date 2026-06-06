// Smooth Rainbow + Web Server for Waveshare ESP32-S3 Mini
// Onboard RGB LED on GPIO 21

#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WebServer.h>

const char* ssid     = "RPi";      // ← CHANGE
const char* password = "1l0v3y0uS4m";  // ← CHANGE

#define LED_PIN    21
#define LED_COUNT  1

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
WebServer server(80);

int currentMode = 0;        // 0 = Rainbow, 1-7 = named colors
uint8_t brightness = 80;

// Named colors (you can add more)
const uint32_t colors[] = {
  0x000000, // 0: Off (not used as button)
  0xFF0000, // 1: Red
  0x00FF00, // 2: Green
  0x0000FF, // 3: Blue
  0xFFFF00, // 4: Yellow
  0xFF00FF, // 5: Magenta
  0x00FFFF, // 6: Cyan
  0xFFFFFF  // 7: White
};

const String colorNames[] = {"Off", "Red", "Green", "Blue", "Yellow", "Magenta", "Cyan", "White"};

void setup() {
  Serial.begin(115200);
  delay(1000);

  strip.begin();
  strip.setBrightness(brightness);
  strip.show();

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    rainbowStep();
    delay(30);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());

  // Web server routes
  server.on("/", handleRoot);
  server.on("/rainbow", []() { currentMode = 0; server.send(200, "text/plain", "Rainbow mode activated"); });
  server.on("/color", handleColor);

  server.begin();
  Serial.println("Web server started!");
}

void loop() {
  server.handleClient();   // Handle web requests

  if (currentMode == 0) {
    rainbowStep();         // Smooth rainbow
    delay(15);
  } else {
    strip.setPixelColor(0, colors[currentMode]);
    strip.show();
    delay(100);            // Small delay when in static color
  }
}

// ====================== RAINBOW ======================
void rainbowStep() {
  static uint8_t offset = 0;
  uint32_t color = wheel(offset);
  strip.setPixelColor(0, color);
  strip.show();
  offset = (offset + 2) % 256;
}

uint32_t wheel(byte pos) {
  if (pos < 85) {
    return strip.Color(255 - pos * 3, pos * 3, 0);
  } else if (pos < 170) {
    pos -= 85;
    return strip.Color(0, 255 - pos * 3, pos * 3);
  } else {
    pos -= 170;
    return strip.Color(pos * 3, 0, 255 - pos * 3);
  }
}

// ====================== WEB PAGES ======================
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32-S3 LED Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 0; padding: 20px; background: #111; color: #fff; }
    h1 { color: #0f0; }
    .btn { padding: 15px 25px; margin: 8px; font-size: 18px; border: none; border-radius: 8px; cursor: pointer; }
    .rainbow { background: linear-gradient(90deg, red, yellow, green, cyan, blue, magenta); color: white; }
    .color-btn { width: 120px; }
  </style>
</head>
<body>
  <h1>ESP32-S3 RGB Control</h1>
  <p><strong>IP:</strong> )rawliteral";

  html += WiFi.localIP().toString();
  html += R"rawliteral(</p>

  <button class="btn rainbow" onclick="setMode(0)">🌈 Rainbow Mode</button>
  <br><br>

  <h2>Static Colors</h2>
  <button class="btn color-btn" style="background:red;color:white" onclick="setColor(1)">Red</button>
  <button class="btn color-btn" style="background:green;color:white" onclick="setColor(2)">Green</button>
  <button class="btn color-btn" style="background:blue;color:white" onclick="setColor(3)">Blue</button><br>
  <button class="btn color-btn" style="background:yellow;color:black" onclick="setColor(4)">Yellow</button>
  <button class="btn color-btn" style="background:magenta;color:white" onclick="setColor(5)">Magenta</button>
  <button class="btn color-btn" style="background:cyan;color:black" onclick="setColor(6)">Cyan</button>
  <button class="btn color-btn" style="background:white;color:black" onclick="setColor(7)">White</button>

  <script>
    function setMode(m) {
      fetch('/rainbow').then(() => location.reload());
    }
    function setColor(c) {
      fetch('/color?c=' + c).then(() => location.reload());
    }
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleColor() {
  if (server.hasArg("c")) {
    int c = server.arg("c").toInt();
    if (c >= 1 && c <= 7) {
      currentMode = c;
    }
  }
  server.send(200, "text/plain", "Color updated");
}
