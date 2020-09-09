#ifndef PRESENCE_LASER_SENSOR_H
#define PRESENCE_LASER_SENSOR_H

#include <esphome.h>
#include <Wire.h>

#define THRESHOLD 700
#define FRONT_LASER_ADDRESS 55
#define BACK_LASER_ADDRESS 82

#define FRONT_OCCUPIED 'o'
#define FRONT_FREE 'f'
#define BACK_OCCUPIED 'b'
#define BACK_FREE 'v'
#define EVENT_BUFFER_SIZE 4

using namespace esphome;
using namespace sensor;
using namespace api;

class EventStorage {
private:
    char eventsBuffer[EVENT_BUFFER_SIZE];
    int bufferIndex = -1;

    int getNextBufferIndex(int index) {
        if (index == EVENT_BUFFER_SIZE - 1) {
            return 0;
        }

        return index + 1;
    }

    int indexOf(char element) {
        for (int i = 0; i < EVENT_BUFFER_SIZE; i++) {
            if (eventsBuffer[i] == element) {
                return i;
            }
        }
        return -1;
    }

public:
    EventStorage() {
        clearBuffer();
    }

    void clearBuffer() {
        for (int i = 0; i < EVENT_BUFFER_SIZE; i++) {
            eventsBuffer[i] = '0';
        }
    }

    bool compareBuffer(char desiredState[]) {
        //Looking for start of desiredState
        int index = indexOf(desiredState[0]);

        if (index == -1) {
            return false;
        }

        for (int i = 0; i < EVENT_BUFFER_SIZE; i++) {
            if (eventsBuffer[index] != desiredState[i]) {
                return false;
            }
            index = getNextBufferIndex(index);
        }

        return true;
    }

    void publishEvent(char event) {
        bufferIndex = getNextBufferIndex(bufferIndex);
        eventsBuffer[bufferIndex] = event;
    }
};

class LaserSensor {
private:
    int address;
    bool lastValue;
    int lastEventTime;
    char sensorOccupied;
    char sensorFree;
    EventStorage *eventStorage;

public:
    LaserSensor(int address, char sensorOccupied, char sensorFree, EventStorage *eventStorage) {
        this->address = address;
        this->sensorFree = sensorFree;
        this->sensorOccupied = sensorOccupied;
        this->eventStorage = eventStorage;

        this->lastValue = false;
    }

    void setup() {
        lastEventTime = millis();
    }

    void checkState() {
        sendMeasureCommand(address);

        delay(1);

        auto distance = readLength(address);

        int timeSinceLastTry = millis() - lastEventTime;

        if (distance < THRESHOLD && !lastValue && timeSinceLastTry > 900) {
            lastValue = true;
            lastEventTime = millis();
            eventStorage->publishEvent(sensorOccupied);
        } else if (distance >= THRESHOLD && lastValue && timeSinceLastTry > 900) {
            lastValue = false;
            lastEventTime = millis();
            eventStorage->publishEvent(sensorFree);
        }
    }

    void sendMeasureCommand(int address) {
        Wire.beginTransmission(address);
        Wire.write(byte(0x00));
        Wire.endTransmission();
    }

    unsigned short readLength(int address) {
        unsigned char responseBuffer[2];
        Wire.requestFrom(address, 2);
        if (2 <= Wire.available()) {
            responseBuffer[0] = Wire.read();
            responseBuffer[1] = Wire.read();
        }
        unsigned short length;

        length = responseBuffer[0];
        length = length << 8;
        length |= responseBuffer[1];

        return length;
    }
};

class PresenceLaserSensor : public Component, public Sensor, public CustomAPIDevice  {
private:
    char PERSON_COME_INSIDE[EVENT_BUFFER_SIZE] = {FRONT_OCCUPIED, BACK_OCCUPIED, FRONT_FREE, BACK_FREE};
    char PERSON_COME_OUTSIDE[EVENT_BUFFER_SIZE] = {BACK_OCCUPIED, FRONT_OCCUPIED, BACK_FREE, FRONT_FREE};

    float peopleCount = 0;
    int timeInside = 0;
    int timeOutside = 0;
    EventStorage *eventStorage;
    LaserSensor *frontSensor;
    LaserSensor *backSensor;

protected:
    std::string icon() override {
        return "mdi:account-multiple";
    }

public:
    PresenceLaserSensor(int frontSensorAddress, int backSensorAddress) {
        eventStorage = new EventStorage();

        frontSensor = new LaserSensor(frontSensorAddress, FRONT_OCCUPIED, FRONT_FREE, eventStorage);
        backSensor = new LaserSensor(backSensorAddress, BACK_OCCUPIED, BACK_FREE, eventStorage);
    }
    void setPeopleCount(float count) {
        this->peopleCount = count;
        publish_state(peopleCount);
    }

    void setup() override {
        Wire.begin(16, 14);

        publish_state(peopleCount);
        backSensor->setup();
        frontSensor->setup();
        timeInside = millis();
        timeOutside = millis();

        register_service(&PresenceLaserSensor::setPeopleCount, "set_people_count", {"count"});
    }

    void loop() override {
        backSensor->checkState();
        frontSensor->checkState();

        int timeSinceInside = millis() - timeInside;
        int timeSinceOutside = millis() - timeOutside;

        if (eventStorage->compareBuffer((char *) PERSON_COME_INSIDE)) {
            if (timeSinceOutside > 900) {
                timeInside = millis();
                setPeopleCount(peopleCount + 1);
            }
            eventStorage->clearBuffer();
        } else if (eventStorage->compareBuffer((char *) PERSON_COME_OUTSIDE)) {
            if(timeSinceInside > 900) {
                timeOutside = millis();
                peopleCount = peopleCount - 1;
                if (peopleCount < 0.0) {
                    peopleCount = 0.0;
                }

                setPeopleCount(peopleCount);
            }
            eventStorage->clearBuffer();
        }
    }
};


#endif //PRESENCE_LASER_SENSOR_H
