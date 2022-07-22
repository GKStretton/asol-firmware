#pragma once

// SetupSwitches initialises pins, device etc. To be called on startup.
void SetupSwitches();
// SetCameraState turns the DSLR on or off.
void SetCameraState(bool on);
// SetRingLightState turns the ring light on or off.
void SetRingLightState(bool on);
// SetMainsState switches the AC power input
void SetMainsState(bool on);
// SetLED sets the on-board LED.
void SetLED(bool on);