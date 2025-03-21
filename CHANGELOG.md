# Change Log

## release v1.2.2
_other_
  silense some compiller warning in C++17

## release v1.2.1
_new features:_
  add PseudoRotaryEncoder::enable, PseudoRotaryEncoder::disable calls
_bugfixes:_
  fix bug with deleting longPressTimer
_other_
  use bitset for _lpcnt

## release v1.2.0
_new features:_
  Rotary encoder emulated with two push buttons
_bugfixes:_
  rework gpio's interrupts configuration. Using percise interrupt configuration for gpio level instead of edge cross.
    This way it works more reliable for noisy buttons and makes button state transition error-prone.

## release v1.1.0
_new features:_
  Implement AsyncEventButton class

_bugfixes:_
 - fix bug when autorepeat was engaged when longPress timer expired during gpio debounce interval
 - fix gpio interrupt handling bug when debouncing was not working properly
 - set GenericButton::Counters members type to int32_t

## rel v1.0.0
### reworking code of rwmingis/InterruptButton lib
 - implement GenericButton class, event policy, event loop hooks, callback menu, gpio button
