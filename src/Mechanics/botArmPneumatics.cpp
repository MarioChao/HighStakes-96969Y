#include "Mechanics/botArmPneumatics.h"
#include "main.h"

namespace {
    void setArmState(bool);
    void switchState();

    bool armDebounce = false;

    int armState = 0;
}

namespace botarmpneu {
    void setArmState(bool value) {
        ::setArmState(value);
    }

    void switchState() {
        ::switchState();
    }
}

namespace {
    void setArmState(bool value = 0) {
        armState = value;
        BotArmPneumatics.set(armState);
    }

    /// @brief Change the arm's position to high or low.
    void switchState() {
        if (!armDebounce) {
            armDebounce = true;

            setArmState(armState ^ 1);
            task::sleep(10);

            armDebounce = false;
        }
    }
}