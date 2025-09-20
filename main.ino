#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>

// --- Sensors ---
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp; // I2C

// --- WiFi ---
const char* ssid = "desk";
const char* password = "password01";
const char* serverURL = "http://172.16.11.29:4444/api/v1/sensor/pushdata";

// --- Sampling ---
#define SAMPLES 64
#define SAMPLING_FREQUENCY 128
const float targetFrequencies[] = {5.0, 10.0, 20.0, 40.0};
const int numFreqs = sizeof(targetFrequencies) / sizeof(targetFrequencies[0]);

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");

  // --- MPU6050 ---
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 Initialized!");

  // --- BMP280 ---
  if (!bmp.begin(0x76)) { // Common I2C address 0x76 or 0x77
    Serial.println("Failed to find BMP280 chip");
    while (1) delay(10);
  }
  Serial.println("BMP280 Initialized!");
}

void loop() {
  // --- Variables for MPU6050 ---
  float rmsX=0, rmsY=0, rmsZ=0;
  float peakX=0, peakY=0, peakZ=0;
  float steX=0, steY=0, steZ=0;

  float q1X[numFreqs]={0}, q2X[numFreqs]={0};
  float q1Y[numFreqs]={0}, q2Y[numFreqs]={0};
  float q1Z[numFreqs]={0}, q2Z[numFreqs]={0};

  float coeff[numFreqs];
  for(int f=0; f<numFreqs; f++){
    float k = 0.5 + ((SAMPLES * targetFrequencies[f]) / SAMPLING_FREQUENCY);
    float omega = (2.0 * PI * k) / SAMPLES;
    coeff[f] = 2.0 * cos(omega);
  }

  // --- Sample loop ---
  for(int i=0;i<SAMPLES;i++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float x = a.acceleration.x;
    float y = a.acceleration.y;
    float z = a.acceleration.z;

    // RMS / STE
    rmsX += x*x; rmsY += y*y; rmsZ += z*z;
    steX += x*x; steY += y*y; steZ += z*z;

    // Peak
    if(fabs(x)>peakX) peakX=fabs(x);
    if(fabs(y)>peakY) peakY=fabs(y);
    if(fabs(z)>peakZ) peakZ=fabs(z);

    // --- Goertzel update ---
    for(int f=0; f<numFreqs; f++){
      float q0X = coeff[f]*q1X[f]-q2X[f]+x;
      q2X[f]=q1X[f]; q1X[f]=q0X;

      float q0Y = coeff[f]*q1Y[f]-q2Y[f]+y;
      q2Y[f]=q1Y[f]; q1Y[f]=q0Y;

      float q0Z = coeff[f]*q1Z[f]-q2Z[f]+z;
      q2Z[f]=q1Z[f]; q1Z[f]=q0Z;
    }

    delay(1000/SAMPLING_FREQUENCY);
  }

  rmsX = sqrt(rmsX/SAMPLES);
  rmsY = sqrt(rmsY/SAMPLES);
  rmsZ = sqrt(rmsZ/SAMPLES);

  // --- Compute Goertzel magnitudes ---
  float magX[numFreqs], magY[numFreqs], magZ[numFreqs];
  for(int f=0; f<numFreqs; f++){
    magX[f] = sqrt( (q1X[f]-q2X[f]*cos(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY))*(q1X[f]-q2X[f]*cos(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY)) +
                     (q2X[f]*sin(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY))*(q2X[f]*sin(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY)) );

    magY[f] = sqrt( (q1Y[f]-q2Y[f]*cos(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY))*(q1Y[f]-q2Y[f]*cos(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY)) +
                     (q2Y[f]*sin(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY))*(q2Y[f]*sin(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY)) );

    magZ[f] = sqrt( (q1Z[f]-q2Z[f]*cos(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY))*(q1Z[f]-q2Z[f]*cos(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY)) +
                     (q2Z[f]*sin(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY))*(q2Z[f]*sin(2*PI*targetFrequencies[f]/SAMPLING_FREQUENCY)) );
  }

  // --- BMP280 readings ---
  float temperature = bmp.readTemperature(); // °C
  float pressure = bmp.readPressure()/100.0F; // hPa
  float altitude = bmp.readAltitude(1013.25); // m (adjust sea level pressure if needed)

  // --- Create JSON ---
  StaticJsonDocument<1024> doc;
  // MPU6050 X/Y/Z
  JsonObject xObj = doc.createNestedObject("x");
  xObj["RMS"]=rmsX; xObj["Peak"]=peakX; xObj["STE"]=steX;
  JsonArray xFreq = xObj.createNestedArray("FrequencyBins");
  for(int f=0; f<numFreqs; f++){
    JsonObject freq = xFreq.createNestedObject();
    freq["Hz"]=targetFrequencies[f]; freq["Magnitude"]=magX[f];
  }

  JsonObject yObj = doc.createNestedObject("y");
  yObj["RMS"]=rmsY; yObj["Peak"]=peakY; yObj["STE"]=steY;
  JsonArray yFreq = yObj.createNestedArray("FrequencyBins");
  for(int f=0; f<numFreqs; f++){
    JsonObject freq = yFreq.createNestedObject();
    freq["Hz"]=targetFrequencies[f]; freq["Magnitude"]=magY[f];
  }

  JsonObject zObj = doc.createNestedObject("z");
  zObj["RMS"]=rmsZ; zObj["Peak"]=peakZ; zObj["STE"]=steZ;
  JsonArray zFreq = zObj.createNestedArray("FrequencyBins");
  for(int f=0; f<numFreqs; f++){
    JsonObject freq = zFreq.createNestedObject();
    freq["Hz"]=targetFrequencies[f]; freq["Magnitude"]=magZ[f];
  }

  // BMP280
  JsonObject bmpObj = doc.createNestedObject("bmp280");
  bmpObj["Temperature"]=temperature;
  bmpObj["Pressure"]=pressure;
  bmpObj["Altitude"]=altitude;

  String jsonOutput;
  serializeJson(doc, jsonOutput);
  Serial.println("JSON: " + jsonOutput);

  // --- Send HTTP POST ---
  if(WiFi.status()==WL_CONNECTED){
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type","application/json");
    int httpResponseCode = http.POST(jsonOutput);
    if(httpResponseCode>0) Serial.println("POST Response: "+String(httpResponseCode));
    else Serial.println("POST failed: "+String(httpResponseCode));
    http.end();
  }

  delay(1000);
}