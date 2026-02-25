#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

// ======================
// OBJETOS
// ======================

Adafruit_BMP280 bmp;
TinyGPS gps;

// Pines (ajusta si es necesario)
SoftwareSerial gpsSerial(4, 3);    // RX, TX GPS
SoftwareSerial pmsSerial(6, 5);    // RX, TX PMS5003
SoftwareSerial radioSerial(8, 7);  // RX, TX APC220

unsigned long packetNumber = 0;

// ======================
// PMS5003
// ======================

struct PMSData {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
};

PMSData pms;

bool readPMS(PMSData &data) {
  if (pmsSerial.available() < 32) return false;

  if (pmsSerial.read() != 0x42) return false;
  if (pmsSerial.read() != 0x4D) return false;

  uint16_t frameLen = (pmsSerial.read() << 8) | pmsSerial.read();
  if (frameLen != 28) return false;

  data.pm1_0 = (pmsSerial.read() << 8) | pmsSerial.read();
  data.pm2_5 = (pmsSerial.read() << 8) | pmsSerial.read();
  data.pm10  = (pmsSerial.read() << 8) | pmsSerial.read();

  // Descartar resto del frame
  for (int i = 0; i < 20; i++) pmsSerial.read();

  return true;
}

// ======================
// SETUP
// ======================

void setup() {
  Serial.begin(115200);        // USB
  gpsSerial.begin(9600);
  pmsSerial.begin(9600);
  radioSerial.begin(9600);

  if (!bmp.begin(0x76)) {
    Serial.println("ERROR: BMP280 no encontrado");
    while (1);
  }

  Serial.println("CANSAT Saturnitos iniciado");
}

// ======================
// LOOP
// ======================

void loop() {

  // -------- GPS (TinyGPS) --------
  bool newData = false;
  unsigned long age;

  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) newData = true;
  }

  float lat = 0.0, lon = 0.0, alt = 0.0;
  int sats = 0;

  if (newData) {
    gps.f_get_position(&lat, &lon, &age);
    alt = gps.f_altitude();
    sats = gps.satellites();
  }

  // -------- BMP280 --------
  float temp = bmp.readTemperature();
  float pres = bmp.readPressure() / 100.0; // hPa

  // -------- PMS5003 --------
  bool pmsOK = readPMS(pms);

  // -------- Contador --------
  packetNumber++;

  // -------- CSV --------
  // Equipo,Paquete,Lat,Lon,Alt,Sat,Temp,Pres,PM1,PM2.5,PM10
  String csv = "Saturnitos,";
  csv += String(packetNumber) + ",";
  csv += String(lat, 6) + ",";
  csv += String(lon, 6) + ",";
  csv += String(alt, 1) + ",";
  csv += String(sats) + ",";
  csv += String(temp, 2) + ",";
  csv += String(pres, 2) + ",";

  if (pmsOK) {
    csv += String(pms.pm1_0) + ",";
    csv += String(pms.pm2_5) + ",";
    csv += String(pms.pm10);
  } else {
    csv += "-1,-1,-1";
  }

  // -------- ENVÍO --------
  radioSerial.println(csv);  // 📡 APC220
  Serial.println(csv);       // 💻 USB

  delay(1000);
}
