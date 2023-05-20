#pragma once

namespace Sleep {
	// idea: this could be binary compositable so you can parse multiple flags.
	enum SleepStatus {
		UNKNOWN = 0,
		SAFE = 1,
		CRIT = 2,
	};
	void Update();
	void Wake();
	void Sleep(SleepStatus status);
	bool IsSleeping();
	bool IsEStopActive();
	SleepStatus GetLastSleepStatus();
	void OverrideLastSleepStatus(SleepStatus status);
	void SetOnSleepHandler(void (*f)(SleepStatus sleepStatus));
	void SetOnWakeHandler(void (*f)(SleepStatus lastSleepStatus));
};