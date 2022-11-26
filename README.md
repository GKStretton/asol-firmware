# Light

This is the firmware for [A Study of Light](https://www.youtube.com/@StudyOfLight). Written for an Arduino Mega 2560.

## Dependencies

- arduino-cli
- Additional Arduino libraries
    - AccelStepper
    - PPM-reader

## Build

- Ensure arduino-cli is installed
- `arduino-cli core install arduino:avr`
- `./build.sh`

## Upload

Options:

- Send to mosquitto mqtt broker the binary as a message to topic `mega/flash`.
- Use local upload script for `/dev/ttyACM0`