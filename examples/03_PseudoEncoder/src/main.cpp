#include "Arduino.h"
#include "espasyncbutton.hpp"

// define gpio's for pseudo-encoder
#define BUTTON_DECREMENT  GPIO_NUM_0
#define BUTTON_INCREMENT  GPIO_NUM_35
#define BUTTON_ACTION     GPIO_NUM_21


// shortcut alias
using ESPButton::event_t;


/**
 * @brief Define PseudoRotary Encoder
 * I use two push-buttons, Decrementing and Incrementing, to simulate a rotary encoder
 * on button press encoder will generate an event with counter value relative to number of increment/decrement actions
 * or 'virtual' position of an encoder
 * 
 * @param gpio_decr - GPIO number for decrementing button
 * @param gpio_incr - GPIO number for incrementing button
 * other constructor parameters are identical to GPIOButton class
 */
PseudoRotaryEncoder enc(BUTTON_DECREMENT, BUTTON_INCREMENT, LOW);

/**
 * @brief also define an action button, will use it to switch encoder modes
 * 
 */
AsyncEventButton b1(BUTTON_ACTION, LOW);

/**
 * @brief function declaration that will subsribe to encoder events
 * 
 * @param handler_args - not used in this example, NULL here
 * @param base - EBTN_ENC_EVENTS identifier
 * @param id - event id
 * @param event_data - supplied data pointer via void*
 */
void encoder_events(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);

/**
 * @brief function declaration that will subsribe to Action button events
 * 
 * @param handler_args - not used in this example, NULL here
 * @param base - EBTN_EVENTS identifier
 * @param id - event id
 * @param event_data - supplied data pointer via void*
 */
void action_button(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);

// in this function I will assign various callbacks for action button
void setup_action_button();

void setup(){
    Serial.begin(115200);
    // We MUST create default event loop unless WiFi or Bluetooth is used
    // you do not need to call this if your sketch includes any WiFi/BT setup functions
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // subscribe our callback function to encoder events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(EBTN_ENC_EVENTS, ESP_EVENT_ANY_ID, encoder_events, NULL, NULL));

    // enable pseudo-encoder
    enc.begin();

    // setup and enable action button
    setup_action_button();

    Serial.println("Try pressing or hold inc/decr buttons to see encoder counter events");
    Serial.println("Use action button to switch encoder modes");
}

void loop(){
    // Simply do nothing here, all button events are processed asynchronously
    delay(1000);
}

// this function will simply print encoder events and count value
void encoder_events(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
    // get pointer to additional data about the event, we can get gpio number and click counter for the button that generated the event
    EventMsg *e = reinterpret_cast<EventMsg*>(event_data);

    // print event message
    Serial.printf( "Encoder gpio:%d, counter:%d\n", e->gpio, e->cntr);
}

// define actions for button
void setup_action_button(){
    // activate events for the action button
    b1.begin();
    // make debounce a bit more aggresive
    b1.timeouts.setDebounce(20000);

    // on press print help message
    b1.onClick([](){
        Serial.println();
        Serial.println(" - Long press will reset encoder to defaults");
        Serial.println(" - Double-click will set a constrained encoder that could count only from -10 to 10 with step 2");
        Serial.println(" - Triple-click will set a constrained encoder that could count only from 0 to 20 with step 1 and will rollover on min and max values");
        Serial.println(" - Quad-click will reset the encoder and set multiplier factor to 2");
        Serial.println("       you can try single, double, triple or n-times clicks and see how cont step is progressing");
        Serial.println();
     });

    b1.onLongPress([](){
        Serial.println("Reset to defaults: counter value 0, step 1, no constrains");
        enc.reset();
    });

    b1.onMultiClick([](int32_t cnt){
        switch(cnt){
            case 2 :
                Serial.println("Constrain -10:10 with step 2");
                enc.reset();
                enc.setCounter(0, 2, -10, 10);
            break;
            case 3 :
                Serial.println("Constrain 0:20 with step 1 and rollover");
                enc.reset();
                enc.setCounter(0, 1, 0, 20);
                enc.setRollover(true);
            break;
            case 4 :
                Serial.println("Default counter with multiplier factor 2");
                enc.reset();
                enc.setMultiplyFactor(2);
            break;
        }

    });

    // enable button
    b1.enable();
}

