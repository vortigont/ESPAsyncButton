# ESPAsyncButton

This project has started as a fork of [rwmingis/InterruptButton](https://github.com/rwmingis/InterruptButton) - a really nice lib for ESP32 interrupt based buttons. But then I reworked it down to scratch to implement a set of classes for versatile button management for ESP32 family chips. It uses event-driven approach to register, transfer an handle button events. Utilizing RTOS features and HW timers event handling is fully asynchronous and decoupled from gpio ISRs. It allows to build flexible event-drivent derivatives for asynchronous, thread-aware projects.

## Features:

 * GPIO-based momentary buttons with/without debouncing
 * Interrupts + HW timers are used to register button press/release and generate events
 * button event propagation and handling via ESP Event loop
 * gpios and event processing are decoupled from each other, i.e. it is possible to handle multiple gpios from a single handler
 * selectable button behavior, from simple `press`/`release` to featured `autorepeat`, `multiclicks`, and other
 * policy-based class templates to easy integrate user-defined event handlers 

## Operation states

Depending on enabled event types button generates events when switching between states. Following diagrams could help to understand it better.

### Short/Long press events

By default only two events are enabled for GeneralButton object:
 - `ESPButton::event_t::press` - generated each time button is presses
 - `ESPButton::event_t::release` - generated each time button is released

If `LongPress`/`LongRelease` events are enabled, then each time button is pressed a timer is started with timeout `TimeOuts::longPress` ms. If this timer expires before button is released, then an event `ESPButton::event_t::longPress` is generated. Then, if longPress was triggered, on button release `ESPButton::event_t::longRelease` event will be generated instead of `ESPButton::event_t::release`.

Timeline events diagram:

![short/long press](short_press.svg)

#### Thanks
Thanks to [R. Mingis](https://github.com/rwmingis) for his original lib and inspiration for this project.

