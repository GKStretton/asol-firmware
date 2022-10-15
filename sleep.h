#pragma once

// 0 disables sleep
#define SLEEP_TIME_MINUTES 5

namespace Sleep {
    void Update();
    void Wake();
    bool IsSleeping();
};