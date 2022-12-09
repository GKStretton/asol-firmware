#pragma once
#include <Arduino.h>

#define FS_I6_CHANNELS 6

namespace FS_I6 {
	enum Channel {
		// Right Horizontal stick
		RH=1,
		// Right Vertical stick
		RV=2,
		// Left Vertical stick
		LV=3,
		// Left Horizontal stick
		LH=4,
		// Left switch
		S1=5,
		// Right switch
		S2=6,
	};

	String GetChannelName(enum Channel c);

	int GetChannelRaw(enum Channel c);

	float GetStick(enum Channel c);
	int GetSwitch(enum Channel c);

	void PrintRawChannels();
	void PrintProcessedChannels();

};