ESPAsyncButton - Examples
======

All examples are ready-to-build [Platformio](https://platformio.org/) projects. Pls switch to example folder and run `pio run -t upload && pio device monitor`
Arduino IDE users should also be fine, but I have not tested it, sorry.


[Async Event Button](/examples/00_AsyncEventButton) - An all-in-one GPIO button class with callbacks demo.
It's a simple wrapper that combines GPIO button, event loop handler and a set of callbacks for each event type.
Use this class if you need just a simple single button solution with callbacks.
Build, run and watch serial monitor for event messages. Try to do different types of clicks with buttons - click, fast double click, multiple clicks, hold, etc... 

[Basic Events Demo](/examples/01_BasicEvents) - Simple example for two buttons that demonstrate all kind of events a button could generate that could be received via event bus.
Build, run and watch serial monitor for event messages. Try to do different types of clicks with buttons - click, fast double click, multiple clicks, hold, etc... 

[CallBack Menu](/examples/02_Callbackmenu) - An example that demonstrates how to create a multi-leveled button Menu where on each level buttons could call different callback functions, change their behavior, etc...
Build, run and watch serial monitor for help messages.
