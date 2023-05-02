# Light

This is the firmware for [A Study of Light](https://www.youtube.com/@StudyOfLight). Written for an Arduino Mega 2560.

## Dependencies

- arduino-cli
- Additional Arduino libraries
    - AccelStepper
    - PPM-reader

## Instructions

### Build

- Ensure arduino-cli is installed
- `arduino-cli core install arduino:avr`
- `./build.sh`

### Upload

Options:

- Send to mosquitto mqtt broker the binary as a message to topic `mega/flash`.
- Use local upload script for `/dev/ttyACM0`

## License

This work is released under the CC0 1.0 Universal Public Domain Dedication. You can find the full text of the license here: https://creativecommons.org/publicdomain/zero/1.0/

### Polite Request for Attribution

While it's not legally required, we kindly ask that you give credit if you use or modify this work. Attribution helps support the project and encourages future learning and contributions. You can provide credit by linking to this repository or mentioning the original author's name. Thank you for your cooperation!