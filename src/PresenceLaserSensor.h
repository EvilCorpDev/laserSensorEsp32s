#ifndef PRESENCE_LASER_SENSOR_H
#define PRESENCE_LASER_SENSOR_H

#include "esphome.h"

#define THRESHOLD 800
#define ANALOG_PIN_MODE ADC_6db

#define FRONT_OCCUPIED 'o'
#define FRONT_FREE 'f'
#define BACK_OCCUPIED 'b'
#define BACK_FREE 'v'
#define EVENT_BUFFER_SIZE 4

using namespace esphome;
using namespace sensor;
using namespace mqtt;

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
    int pin;
    bool lastValue;
    char sensorOccupied;
    char sensorFree;
    EventStorage *eventStorage;

public:
    LaserSensor(int pin, char sensorOccupied, char sensorFree, EventStorage *eventStorage) {
        this->pin = pin;
        this->sensorFree = sensorFree;
        this->sensorOccupied = sensorOccupied;
        this->eventStorage = eventStorage;

        this->lastValue = false;
    }

    void setup() {
        adcAttachPin(pin);
        analogSetPinAttenuation(pin, ANALOG_PIN_MODE);
    }

    void checkState() {
        auto analogValue = analogRead(pin);

        if (analogValue > THRESHOLD && !lastValue) {
            lastValue = true;
            eventStorage->publishEvent(sensorOccupied);
        } else if (analogValue <= THRESHOLD && lastValue) {
            lastValue = false;
            eventStorage->publishEvent(sensorFree);
        }
    }
};

class PresenceLaserSensor : public Component, public Sensor, public CustomMQTTDevice {
private:
    char PERSON_COME_INSIDE[EVENT_BUFFER_SIZE] = {FRONT_OCCUPIED, BACK_OCCUPIED, FRONT_FREE, BACK_FREE};
    char PERSON_COME_OUTSIDE[EVENT_BUFFER_SIZE] = {BACK_OCCUPIED, FRONT_OCCUPIED, BACK_FREE, FRONT_FREE};

    float peopleCount = 0;
    EventStorage *eventStorage;
    LaserSensor *frontSensor;
    LaserSensor *backSensor;

    void setPeopleCount(float count) {
        this->peopleCount = count;
        publish_state(peopleCount);
    }

protected:
    std::string icon() override {
        return "mdi:account-multiple";
    }

public:
    PresenceLaserSensor(int frontSensorPin, int backSensorPin) {
        eventStorage = new EventStorage();

        frontSensor = new LaserSensor(frontSensorPin, FRONT_OCCUPIED, FRONT_FREE, eventStorage);
        backSensor = new LaserSensor(backSensorPin, BACK_OCCUPIED, BACK_FREE, eventStorage);
    }

    void setup() override {
        frontSensor->setup();
        backSensor->setup();

        publish_state(peopleCount);

        //TODO: Rename topic
//        subscribe("the/topic", &PresenceLaserSensor::on_reset);
    }

    void loop() override {
        backSensor->checkState();
        frontSensor->checkState();

        if (eventStorage->compareBuffer((char *) PERSON_COME_INSIDE)) {
            setPeopleCount(peopleCount + 1);
            eventStorage->clearBuffer();
        } else if (eventStorage->compareBuffer((char *) PERSON_COME_OUTSIDE)) {
            setPeopleCount(peopleCount - 1);
            eventStorage->clearBuffer();
        }
    }


    void on_reset(const std::string &payload) {

    }
};


#endif //PRESENCE_LASER_SENSOR_H
