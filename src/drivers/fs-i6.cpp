#include "fs-i6.h"
#include <PPMReader.h>
#include "../config.h"
#include "../middleware/logger.h"
#include "../middleware/serialmqtt.h"

namespace FS_I6 {

	namespace {
		PPMReader ppm(CONTROLLER_PPM, PPM_CHANNELS);
	}

	String GetChannelName(enum Channel c) {
		switch (c) {
		case RH:
			return "RH";
		case RV:
			return "RV";
		case LV:
			return "LV";
		case LH:
			return "LH";
		case S1:
			return "S1";
		case S2:
			return "S2";
		default:
			return "UNKNOWN";
		}
	}

	int GetChannelRaw(enum Channel c) {
		return (int) ppm.latestValidChannelValue(c, 1500);
	}

	float GetStick(enum Channel c) {
		int raw = GetChannelRaw(c);
		
		// DEADBANDING
		
		// size of deadband each side of value
		int bandSize = 50;

		// group certain values into a single value to cut out stationary noise
		int withBounds = raw;

		// values
		int min = 1000;
		int mid = 1500;
		int max = 2000;


		// banding
		// else if (withBounds < min + bandSize) {
		//     withBounds = min;
		// }
		if (withBounds > mid - bandSize && withBounds < mid + bandSize) {
			withBounds = mid;
		}
		// make 0 start at edge of band
		if (withBounds > mid + bandSize) {
			withBounds -= bandSize;
		}
		if (withBounds < mid - bandSize) {
			withBounds += bandSize;
		}
		// else if (withBounds > max - bandSize) {
		//     withBounds = max;
		// }

		// TRANSFORM values -> (-1.0, 1.0)

		float transformed = float(withBounds - mid)  / float(max - mid);

		return transformed;
	}

	int GetSwitch(enum Channel c) {
		int raw = GetChannelRaw(c);
		if (c == S1) {
			return raw > 1600;
		} else {
			if (raw < 1250) {
				return 0;
			} else if (raw < 1750) {
				return 1;
			} else {
				return 2;
			}
		}
	}

	void PrintRawChannels() {
		for (int i = 1; i <= FS_I6_CHANNELS; i++) {
			String name = GetChannelName((enum Channel) i) + "_raw";
			int raw = GetChannelRaw((enum Channel) i);

			SerialMQTT::Publish("mega/"+name, String(raw));
		}
	}

	void PrintProcessedChannels() {
		for (int i = 1; i <= FS_I6_CHANNELS; i++) {
			String name = GetChannelName((enum Channel) i) + "_proc";
			if (i == S1 || i == S2) { // The switch channels
				int val = GetSwitch((enum Channel) i);
				SerialMQTT::Publish("mega/"+name, String(val));
			} else {
				float val = GetStick((enum Channel) i);
				SerialMQTT::Publish("mega/"+name, String(val));
			}
		}
	}
}