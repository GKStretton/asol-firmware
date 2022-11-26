#include "controller.h"
#include "../app/navigation.h"

void Controller::autoUpdate(State *s) {
    Navigation::UpdateNodeNavigation(s);
}