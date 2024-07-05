#include "Adafruit_VL53L0X.h"

#define NUM_SENSORS 5

// address we will assign if dual sensor is present
const uint8_t LOX_ADDRESS[NUM_SENSORS] = {
    0x30, 0x31, 0x32, 0x33, 0x34
};

// set the pins to shutdown
const uint8_t SHT_LOX[NUM_SENSORS] = {
    2, 3, 4, 5, 6
};

// objects for the vl53l0x
Adafruit_VL53L0X lox[NUM_SENSORS];

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure[NUM_SENSORS];

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
    // all reset
    for(int i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(SHT_LOX[i], LOW);
    }
    delay(10);
    // all unreset
    for(int i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(SHT_LOX[i], HIGH);
    }
    delay(10);

    // reset all
    for(int i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(SHT_LOX[i], LOW);
    }
    delay(10);

    for(int i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(SHT_LOX[i], HIGH);
        delay(10);

        if(!lox[i].begin(LOX_ADDRESS[i])) {
            Serial.println(F("Failed to boot VL53L0X"));
            while(1);
        }
        delay(10);
    }
}

void read_sensors() {

    for(int i = 0; i < NUM_SENSORS; i++) {
        lox[i].rangingTest(&measure[i], false);

        if(measure[i].RangeStatus != 4) {
            String str = ":" + String(i) + ":" + String(measure[i].RangeMilliMeter);
            Serial.print(str);
            Serial.println();
        } else {
            String str = ":" + String(i) + ":1";
            Serial.print(str);
            Serial.println();
        }
    }
}

void setup() {
    Serial.begin(115200);

    // wait until serial port opens for native USB devices
    while(! Serial) {
        delay(1);
    }

    for(int i = 0; i < NUM_SENSORS; i++) {
        pinMode(SHT_LOX[i], OUTPUT);
    }

    Serial.println(F("Shutdown pins inited..."));

    for(int i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(SHT_LOX[i], LOW);
    }

    Serial.println(F("Both in reset mode...(pins are low)"));

    Serial.println(F("Starting..."));
    setID();
}

void loop() {
    read_sensors();
}