#pragma once
#include <Arduino.h>

class Button {
public:
    // Constructor: takes the pin number 6
    Button(int pin);

    // In arduino: Button startButton(6);

    // Call in setup()
    void begin();

    // Call repeatedly in loop(); returns true once when button is pressed
    bool check();

private:
    int _pin;
    int _buttonState;
    int _lastButtonState;
    unsigned long _lastDebounceTime;
    unsigned long _debounceDelay;
};
