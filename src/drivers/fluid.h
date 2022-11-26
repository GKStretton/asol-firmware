#pragma once

// SetupFluid initialises pins.
void SetupFluid();

// SetAirFlow is a direct function for testing that sets the flow for air in
// the fluid line.
void SetAirFlow(bool flow);

// SetFlushFlow is a direct function for testing that sets the flow for the
// flushing fluid.
void SetFlushFlow(bool flow);

// SetCanvasFlow is a direct function for testing that sets the flow on the
// canvas fluid.
void SetCanvasFlow(bool flow);