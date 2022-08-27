#include "switch.h"
#include "config.h"
#include "util.h"
#include <Arduino.h>
#include <Servo.h>

Servo cameraServo;

void SetupSwitches() {
    // camera switch
	cameraServo.attach(CAMERA_SERVO_PIN);
	cameraServo.write(CAMERA_OFF_ANGLE);

    // led
	InitPin(LED_BUILTIN, LOW);

    // todo: ring light setup
}

void SetCameraState(bool on) {
    if (on) {
        cameraServo.write(CAMERA_ON_ANGLE);
    } else {
        cameraServo.write(CAMERA_OFF_ANGLE);
    }
}

void SetRingLightState(bool on) {
    // todo: implement ring light setter (test the solenoid first)
}

void SetMainsState(bool on) {
    //SetSingleRelay(MAINS_RELAY_PIN, on);
}

void SetLED(bool on) {
    if (on) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
}