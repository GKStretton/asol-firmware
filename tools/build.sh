#!/bin/bash

arduino-cli compile --fqbn arduino:avr:mega --libraries ~/Arduino/libraries --output-dir ./build .
