#include "DHT.h"
#include <math.h>


const int PELTIER = 13;
const int CABIN_SENSOR=4;
const int AMBIENT_SENSOR=2;
float tCabin = NAN;
float tAmbient = NAN;
float hCabin = NAN;
float hAmbient = NAN;

const float targetTLow = 55;
const float targetTHigh = 58;

const bool debug = true;
unsigned long lastDebugReportTime = 0;
const float delayTime = 15000;

//predict for the next 2 minutes
const float predictIterations = 2.0 * 60.0 * 1000.0 / delayTime;
  

float sensorSmooth = 0.8;

DHT cabinSensor = DHT(CABIN_SENSOR, DHT11);
DHT ambientSensor = DHT(AMBIENT_SENSOR, DHT11);

struct SensorState {
  float t;
  float tNext;
  float h;
  float hNext;
};

struct SensorStates {
  SensorState cabin;
  SensorState ambient;
};

struct OutputState {
  bool cooling;
};

OutputState gLast;


void setup() {
  Serial.begin(9600);
  Serial.println("Starting");

  gLast.cooling = false;

  cabinSensor.begin();
  ambientSensor.begin();

  //actual pump
  pinMode(PELTIER, OUTPUT);

  Serial.println("Waiting for cabin sensor");
  initSensorOrStall(cabinSensor, tCabin, hCabin);
  
  Serial.println("Waiting for ambient sensor");
  initSensorOrStall(ambientSensor, tAmbient, hAmbient);

  Serial.println("Iteration Delay: " + String(delayTime));
  Serial.println("Predict Iterations: " + String(predictIterations));

  stopCooling();
}


void startCooling() {
  digitalWrite(PELTIER, HIGH); 
  gLast.cooling = true;
}

void stopCooling() {
  digitalWrite(PELTIER, LOW);
  gLast.cooling = false;
}


float smooth(float g, float now, float smooth) {
  return g * smooth + (1.0 - smooth) * now;
}

float predict(float oldValue, float newValue) {
  return newValue + (newValue - oldValue) * predictIterations;
}

void initSensorOrStall(DHT &dht, float & tInit, float & hInit) {
  float t = dht.readTemperature(true);
  float h = dht.readHumidity();
  while(isnan(t) || isnan(h)) {
    Serial.println("Sensor Not Ready");
    delay(1000);

    t = dht.readTemperature(true);
    h = dht.readHumidity();
  } 


  tInit = t;
  hInit = h;
}

SensorStates getSensorStates() {
  
  float tCabinNext = smooth(tCabin, cabinSensor.readTemperature(true), sensorSmooth);
  float tAmbientNext = smooth(tAmbient, ambientSensor.readTemperature(true), sensorSmooth);
  float hCabinNext = smooth(hCabin, cabinSensor.readHumidity(), sensorSmooth);
  float hAmbientNext = smooth(hAmbient, ambientSensor.readHumidity(), sensorSmooth);

  
  SensorState ambient, cabin;

  ambient.t = tAmbientNext;
  ambient.tNext = predict(tAmbient, tAmbientNext);
  ambient.h = hAmbientNext;
  ambient.hNext = predict(hAmbient, hAmbientNext);
 
  cabin.t = tCabinNext;
  cabin.tNext = predict(tCabin, tCabinNext);
  cabin.h = hCabinNext;
  cabin.hNext = predict(hCabin, hCabinNext);

  tAmbient = tAmbientNext;
  hAmbient = hAmbientNext;
  tCabin = tCabinNext;
  hCabin = hCabinNext;


  SensorStates ret;
  ret.ambient = ambient;
  ret.cabin = cabin;

  return ret;
}


void report(SensorStates sensors) {
 Serial.println(
    String("Report:\n") +
    String("\tcabin: ") + String(sensors.cabin.t) + String(", prediction: ") + String(sensors.cabin.tNext) + String("\n") + 
    String("\ttarget: ") + String(targetTLow) + String(" - ") + String(targetTHigh) + String ("\tdelta: ") + String(sensors.cabin.t - (targetTHigh + targetTLow) * 0.5) +  String("\n") + 
    String("\tambient: ") + String(sensors.ambient.t) + String ("\tdelta: ") + String(sensors.cabin.t - sensors.ambient.t) +  String("\n") + 
    String("\tcurrently: ") + String(gLast.cooling ? "cooling" : "stalling") + String("\n") +

    String("\thumidity: cabin:") + String(sensors.cabin.h) + String(", outside: ") + String(sensors.ambient.h) + String("\n") + 
    String("\n")
  ); 
}

void loop() {

  const unsigned long currentTime = millis();
  SensorStates sensors = getSensorStates();
 

  if (sensors.cabin.t < targetTLow || sensors.cabin.tNext < targetTLow) {
    if (gLast.cooling) {
      stopCooling();
      report(sensors);  
    }
    
  }
  else if (sensors.cabin.t > targetTHigh || sensors.cabin.tNext > targetTHigh) {
    if (!gLast.cooling) {
      startCooling();    
      report(sensors);
    }
  }

  if (debug || (currentTime - lastDebugReportTime) >= 10*60*1000) {
    report(sensors);  
    lastDebugReportTime = currentTime;
  }
  
  delay(delayTime);
}

