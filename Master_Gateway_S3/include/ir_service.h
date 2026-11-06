#pragma once
#include <Arduino.h>

enum IRPairingState {
    IR_PAIRING_NONE = 0,
    IR_PAIRING_UP,
    IR_PAIRING_DOWN,
    IR_PAIRING_LEFT,
    IR_PAIRING_RIGHT,
    IR_PAIRING_ENTER,
    IR_PAIRING_ESC,
    IR_PAIRING_STANDBY,
    IR_PAIRING_POWEROFF,
    IR_PAIRING_DONE
};

namespace IRService {
    void init();
    void resetRecvBuffer();
    void loop();
    
    // Joystick & Power simulation states
    bool isUp();
    bool isDown();
    bool isLeft();
    bool isRight();
    bool isEnter();
    bool isEsc();
    bool isStandby();
    bool isPowerOff();
    
    // Live Inspector Getters
    uint64_t getLastRecvCode();
    void clearLastRecvCode();
    uint64_t getCodeUp();
    uint64_t getCodeDown();
    uint64_t getCodeLeft();
    uint64_t getCodeRight();
    uint64_t getCodeEnter();
    uint64_t getCodeEsc();
    uint64_t getCodeStandby();
    uint64_t getCodePowerOff();
    
    // Hardware Power API
    void enterDeepSleep();
    
    // Pairing & Shielding & Profile API
    void setActiveProfile(int profile);
    int getActiveProfile();
    
    void startPairingKey(IRPairingState targetKey);
    void startSequentialPairing();
    bool isSequentialPairing();
    IRPairingState getPairingState();
    void cancelPairing();
    bool isPairingActive();
    
    bool isDuplicateError();
    void clearDuplicateError();
    
    void setShielding(bool shield);
    bool isShielded();
}
