#include "SparkFunBME280.h"
#include "math.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

BME280 sensor;

String temperature;
String humidity;
String pressure;
String webhook;
String dewpoint;

bool canSleep = false;

const long subscribeWaitingInterval = 8000; //wait time for PUBLISH feedback (millis)
const unsigned long MAX_TIME_TO_CONNECT_MS = 20000; //wait time for wiFi/cloud connect (millis)
const int sleepIntervalTimeOut = 1800; //sleep time if cloud connect timed out (s)
const int sleepIntervalNormal = 720; //normal time between measurements (s)
const int lowBatterySleepTime = 21600; //sleep time if low battery is triggered (s)
int motorRunTime = 50000; //millis motor should be on
int lastResetTime = 0;

const int sensorPower = D7;
int analogPinSupply = A4;
int controlPinSupply = D4;
int analogPinBattery = A3;
int controlPinBattery = D5;
int motorPin = A5;

double batteryV;
double supplyV;
int batteryV_top;
int supplyV_top;

double h = 0.0;
double h_previous = 5.0;
double t_f = 0.0;
double t_f_previous = 500.0;
double p = 0.0;
double p_previous = 50.0;
double dewptf = 0.0;

void initializeSensor() {
    //For I2C, enable the following and disable the SPI section
    sensor.settings.commInterface = I2C_MODE;
    sensor.settings.I2CAddress = 0x77;

    //***Operation settings*****************************//

    //runMode can be:
    //  0, Sleep mode
    //  1 or 2, Forced mode
    //  3, Normal mode
    sensor.settings.runMode = 1;

    //tStandby can be:
    //  0, 0.5ms
    //  1, 62.5ms
    //  2, 125ms
    //  3, 250ms
    //  4, 500ms
    //  5, 1000ms
    //  6, 10ms
    //  7, 20ms
    sensor.settings.tStandby = 0;

    //filter can be off or number of FIR coefficients to use:
    //  0, filter off
    //  1, coefficients = 2
    //  2, coefficients = 4
    //  3, coefficients = 8
    //  4, coefficients = 16
    sensor.settings.filter = 0;

    //tempOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    sensor.settings.tempOverSample = 1;

    //pressOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    sensor.settings.pressOverSample = 1;

    //humidOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    sensor.settings.humidOverSample = 1;

    //initialize i2c
    sensor.begin();
}

void subscribeHandler(const char* event, const char* data) {
    canSleep = true;
}

void setup() {
    pinMode(sensorPower, OUTPUT);
    pinMode(motorPin, OUTPUT);
    pinMode(D3, INPUT);
    Particle.subscribe("webhook", subscribeHandler, MY_DEVICES);
    lastResetTime = Time.now();
}

void loop() {

    checkPeriodicReset();

    if (checkSolarPwr()) {
        runMotor();
        delay(3000);
        getSensorReading();
        delay(1000);
    }
    else {
        getSensorReading();
    }

    publishData();

    System.sleep(D3,RISING,sleepIntervalNormal);
}

double calculateDewPoint(double T,double h) { /*Magnus Equation https://pdfs.semanticscholar.org/e873/a898ba9373af4e12907841411f3e9d83cb9a.pdf */
    double dewPointF;
    dewPointF = ((-243.04*T - 59068.4)*log(h) - 3164.34*T + 272020)/((T + 243.04)*log(h)-4.60517*T-5402.82);
    return dewPointF;
}

double checkBatteryVoltage(){

    //if battery voltage is too low, switch system to deep sleep for X seconds
    pinMode(controlPinBattery, OUTPUT);
    pinMode(analogPinBattery, AN_INPUT);
    digitalWrite(controlPinBattery, HIGH);
    delay(1);
    digitalWrite(controlPinBattery, LOW);
    batteryV_top = analogRead(analogPinBattery);
    batteryV = 2*batteryV_top*0.0008;
    delay(4);
    digitalWrite(controlPinBattery, HIGH);

    if (batteryV<=3.3) {
        System.sleep(D3,RISING,lowBatterySleepTime);
    }

    return batteryV;
}

bool checkSolarPwr(){ //if solar panel is active and charging battery, returns True

    checkBatteryVoltage();

    pinMode(controlPinSupply, OUTPUT);
    pinMode(analogPinSupply, AN_INPUT);
    digitalWrite(controlPinSupply, HIGH);
    delay(1);
    digitalWrite(controlPinSupply, LOW);
    supplyV_top = analogRead(analogPinSupply);
    supplyV = (2.463415)*supplyV_top*0.0008; //Vin -> 12k -> 8.2k -> GND, for max 3.3V (max input) @ 8V (Vocc of solar panel)
    delay(4);
    digitalWrite(controlPinSupply, HIGH);

    double supplyV_adj = batteryV + 0.150;

    if (supplyV >= supplyV_adj) {
        return true;
    }
    else {
        return false;
    }
}

void runMotor(){

    //run fan for X seconds, taking battery voltage measurement during current draw

    long startMillisMotor = millis();
    long currentMillisMotor = millis();
    while (currentMillisMotor - startMillisMotor <= motorRunTime) {
        currentMillisMotor = millis();
        analogWrite(motorPin, 127, 100);
    }

    checkBatteryVoltage();

    analogWrite(motorPin, 0, 100);
}

void checkPeriodicReset() {
    //reset photon every week
    int currentTime = Time.now();
    if (currentTime - lastResetTime > 604800) {
        System.reset();
    }
}

void publishData()
{
    if ( (!isAlmostEqual(t_f_previous,t_f,0.5)) || (!isAlmostEqual(h_previous,h,0.5)) || (!isAlmostEqual(p_previous,p,0.02))) {
        t_f_previous = t_f;
        h_previous = h;
        p_previous = p;

        Particle.connect();
        //timeout occurred go back to deep sleep
        if (!waitFor(Particle.connected, MAX_TIME_TO_CONNECT_MS)) {
            Particle.disconnect();
            WiFi.off();
            System.sleep(D3,RISING,sleepIntervalTimeOut);
        }

        temperature = String(t_f).format("%1.2f", t_f);
        humidity = String(h).format("%1.2f", h);
        pressure = String(p).format("%1.2f", p);
        dewpoint = String(p).format("%1.2f", dewptf);
        webhook =  "{\"baromin\": " + pressure + ", " +"\"dewptf\": " + dewpoint + ", " + "\"tempf\": " + temperature + ", " + "\"humidity\": " + humidity + "}";
        Particle.publish("webhook", webhook, PRIVATE);

        // waiting for the event - delay runs the system idle code
        long startTime = millis();

        while (!canSleep && (millis() - startTime < subscribeWaitingInterval)) {
            Particle.process();
            delay(100);
        }
        //reset sleep checker bool to initial condition and then sleep
        canSleep = false;
        Particle.disconnect();
        WiFi.off();
    }
}

void getSensorReading()
{
    //turn on sensor
    digitalWrite(sensorPower, HIGH);

    delay(10);

    initializeSensor();

    delay(25); //sensor power on time

    t_f = sensor.readTempF();;
    h = sensor.readFloatHumidity();
    p = sensor.readFloatPressure();

    p = 0.0002952998*p; //convert Pa to inHg
    dewptf = calculateDewPoint(t_f,h);

    //turn off sensor
    digitalWrite(sensorPower, LOW);
}

bool isAlmostEqual(double a, double b, double epsilon)
{
    // if the distance between a and b is less than epsilon, then a and b are "close enough"
    //abs is integer math
    return fabs(a - b) <= epsilon;
}