#include "Button.h"

Button::Button(int pin)
    : _pin(pin),
      _buttonState(LOW),
      _lastButtonState(LOW),
      _lastDebounceTime(0),
      _debounceDelay(50) // Default debounce time
{}

void Button::begin() {
    pinMode(_pin, INPUT);
}

bool Button::check() {
    int reading = digitalRead(_pin);

    if (reading != _lastButtonState) {
        _lastDebounceTime = millis();
    }

    if ((millis() - _lastDebounceTime) > _debounceDelay) {
        if (reading != _buttonState) {
            _buttonState = reading;

            if (_buttonState == HIGH) {
                _lastButtonState = reading;
                return true; // Button was just pressed
            }
        }
    }

    _lastButtonState = reading;
    return false; // No new press detected
}