#pragma once
#include "../app/state.h"

class Controller {
public:
    void Update(State *s);
private:
    void autoUpdate(State *s);
    void manualUpdate(State *s);
};