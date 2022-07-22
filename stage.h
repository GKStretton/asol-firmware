#pragma once

// SetupStage initialises pins, devices etc. To be called on startup.
void SetupStage();
// Drain will drain the bowl if true, else stop draining.
void Drain(bool drain);