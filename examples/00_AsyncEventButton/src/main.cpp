#include "Arduino.h"
#include "espasyncbutton.hpp"


#define BUTTON_1  GPIO_NUM_35                 // Set GPIO for the button #1


/**
 * @brief AsyncEventButton is a very simple object that allows to create a GPIO-attached button
 * and a set of callback functions assigned to it.
 * For every kind of button event a dedicated callback function will trigger
 * 
 */

/**
 * @brief AsyncEventButton object construtor has the following parameters
 * 
 * @param gpio - GPIO number
 * @param logicLevel - Logic level state of the gpio when button is in 'PRESSED' state, LOW or HIGHT
 * @param pull - GPIO pull mode as defined in   https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html#_CPPv416gpio_pull_mode_t
 * @param mode - GPIO mode as defined in        https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html#_CPPv411gpio_mode_t
 * @param debounce = enable/disable debounce feature for gpio

    AsyncEventButton(gpio_num_t gpio, bool logicLevel, gpio_pull_mode_t pull = GPIO_PULLUP_ONLY, gpio_mode_t mode = GPIO_MODE_INPUT, bool debounce = true);
 */

/**
 * @brief A simple gpio mapped button with Logic level LOW, i.e. button shorts gpio to the ground, gpio must be pulled high
 */
AsyncEventButton b1(BUTTON_1, LOW);


// Let's define our callback functions

void on_press(){
    Serial.println("Button pressed");
}

void on_release(){
    Serial.println("Button released");
}

// Multiple clicks callback
void on_multiclick(int32_t counter){
    Serial.printf("You clicked %d times!\n", counter);
    Serial.println("You can try triple-click to enable auto-repeat on this button, or 5 times click to disable auto-repeat");

    // On triple-click we enable auto-repeat action callback 
    if (counter == 3){
        // lets enable auto-repeat for this button
        Serial.println("Auto-repeat on Long-Press enabled");
        Serial.println("If you keep auto-repeat up to 15 times, button will disable MultipleClick actions");
        // we use lambda function as a callback
        b1.onAutoRepeat(
            [](int32_t counter){
                Serial.printf("auto-repeat %d times\n", counter);
                if(counter == 15) { b1.onMultiClick(nullptr); Serial.print("Multiple click actions disabled, use LongPress to enable it again"); };
            }
        );
    } else if (counter == 5){
        // for 5 clicks we disable auto-repeat
        Serial.println("Auto-repeat disabled!");
        b1.onAutoRepeat(nullptr);
    }

};

// long button press
void on_LongPress(){
    Serial.println("Long press action! Now you can try double, triple, or any number of consecutive clicks...");
    // enable MultiClicks
    b1.onMultiClick(on_multiclick);
    // and let's disable "press", "release" actions to reduce number of printed text
    b1.onPress(nullptr);
    b1.onRelease(nullptr);
    Serial.println("Note: 'press' and 'release' actions are disabled to reduce number of printed text");
}




void setup(){
    Serial.begin(115200);
    // We MUST create default ESPevent loop unless WiFi or Bluetooth is used
    // you do not need to call this if your sketch includes any WiFi/BT setup functions
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // activate event processing for button
    // NOTE: Button is stil in 'disabled' state, you need to 'enable' it to start handling GPIO interrupts
    b1.begin();

/*
    We can also configure various button/gpio timeouts on how fast/slow Button
    reacts to click/multiclick evets, etc..

    b1.timeouts.setDebounce(t)  -   debounce    time in microseconds, default is 5000
    b1.timeouts.setLongPress(t) -   LongPress   timeout in milliseconds, how long you need to hold the button for it to trigger LongPress event
    b1.timeouts.setAutoRepeat   -   AutoRepeat  timeout in milliseconds, how fast the button will generate 'autorepeat' events when held down
    b1.timeouts.setMultiClick   -   MultiClick  timeout in milliseconds, how long Button will wait for consecutive clicks before triggering MultiClick event
*/

    // Let's assign callbacks to some of the button actions
    // we will only assign 'press', 'release', 'click' and 'LognPress' actions.
    // Other actions will be activated later by special key sequences

    // on-Press callback
    b1.onPress(on_press);

    // on-Release callback
    b1.onRelease(on_release);

    // on-LongPress callback
    b1.onLongPress(on_LongPress);

    // you can also assign functional callbacks or lamda functions
    b1.onClick([](){ Serial.println("Click!"); });

    // enable button
    b1.enable();

    // let's pause for 2 seconds to let serial-monitor attach to the outpus after flash/reboot 
    delay(2000);

    // Print some help message
    Serial.println("\n\nAsyncEventButton enabled, pls try 'press', 'release' and 'click' actions.");
    Serial.println("Multiple clicks and autorepeats are currently disabled");
    Serial.println("To activate MultiClicks use 'LongPress' action");
}

void loop(){
    // Simply do nothing here, all button events are processed asynchronously
    delay(1000);
}

