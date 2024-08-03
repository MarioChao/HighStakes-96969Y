#include "Mechanics/botLift.h"
#include "main.h"

namespace {
    void setLiftState(bool);
    void switchLiftState();

    bool liftDebounce = false;

    int liftState = 0;
}

namespace botlift {
    void setLiftState(bool value) {
        ::setLiftState(value);
    }

    void switchState() {
        ::switchLiftState();
    }
}

namespace {
    void setLiftState(bool value) {
        liftState = value;
        IntakeLiftPneumatic.set(liftState);
    }

    /// @brief Change the lift's position to high or low.
    void switchLiftState() {
        if (!liftDebounce) {
            liftDebounce = true;

            setLiftState(liftState ^ 1);
            task::sleep(10);

            liftDebounce = false;
        }
    }
}
