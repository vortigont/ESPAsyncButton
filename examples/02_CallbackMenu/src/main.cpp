#include "Arduino.h"
#include "espasyncbutton.hpp"

#define BUTTON_1  GPIO_NUM_0                 // Set GPIO for the button #1
#define BUTTON_2  GPIO_NUM_35                // Set GPIO for the button #2

using ESPButton::event_t;

// Buttons
GPIOButton<ESPEventPolicy> b1(BUTTON_1, LOW);
GPIOButton<ESPEventPolicy> b2(BUTTON_2, LOW);

// Button callback menu
ButtonCallbackMenu menu;

// CallBack functions
void cursor_control(event_t e, const EventMsg* m);
void volume_control(event_t e, const EventMsg* m);
void counters(event_t e, const EventMsg* m);
void menu_toggle();


void setup(){
    Serial.begin(115200);
    // We must create default event loop unless WiFi or Bluetooth is used
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // attach to system event loop to listen for button evnts
    ESP_ERROR_CHECK(esp_event_handler_instance_register(EBTN_EVENTS,
                                                        ESP_EVENT_ANY_ID,
                                                        // this lambda will simply translate loop events into btn_callback_t callback function
                                                        [](void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
                                                            menu.handleEvent(ESPButton::int2event_t(id), reinterpret_cast<EventMsg*>(event_data));
                                                        }, 
                                                        NULL, NULL) );

    // disable simple press/release events, we do not need them
    b1.enableEvent(event_t::press, false);
    b1.enableEvent(event_t::release, false);
    b2.enableEvent(event_t::press, false);
    b2.enableEvent(event_t::release, false);

    // enable clicks
    b1.enableEvent(event_t::click);
    b2.enableEvent(event_t::click);

    // enable longpress to cycle through menu
    b1.enableEvent(event_t::longPress);
    b2.enableEvent(event_t::longPress);

    // assign callback functions
    // I can assign same callbacks to different gpios and deal with each button later

    // menu 0
    menu.assign(BUTTON_1, 0, cursor_control);
    menu.assign(BUTTON_2, 0, cursor_control);
    // menu 1
    menu.assign(BUTTON_1, 1, volume_control);
    menu.assign(BUTTON_2, 1, volume_control);
    // menu 2
    menu.assign(BUTTON_1, 2, counters);
    menu.assign(BUTTON_2, 2, counters);

    // enable buttons
    b1.enable();
    b2.enable();
}

void loop(){
    // Simply do nothing here
    delay(1000);
}


// this function will toggle menu level on any button longPress
void menu_toggle(){
    menu.setMenuLevel( (menu.getMenuLevel()+1)%4 );
    // on menu level 2 we will enable autorepeat for  button_1 and multiclick counter for button_2
    if (menu.getMenuLevel() == 2){
        b1.enableEvent(event_t::autoRepeat);
        b2.enableEvent(event_t::multiClick);
    } else {
        b1.enableEvent(event_t::autoRepeat, false);
        b2.enableEvent(event_t::multiClick, false);
    }
    Serial.print("Switched to menu level:"); Serial.println(menu.getMenuLevel());
};

// this function will pretend its moving a cursor
void cursor_control(event_t e, const EventMsg* m){
    switch(e){
        // Use click event to move cursor Up and Down (just print)
        case event_t::click :
            Serial.print("Cursor "); Serial.println(m->gpio == BUTTON_1 ? "Up" : "Down");
            break;
        // any button longpress event will cycle menu level 0->1->2->3->0
        case event_t::longPress :
            menu_toggle();
            break;
    }
};

// this function will pretend its changing volume
void volume_control(event_t e, const EventMsg* m){
    switch(e){
        // Use click event to control Volume Up and Down (just print)
        case event_t::click :
            Serial.print("Volume "); Serial.println(m->gpio == BUTTON_1 ? "Up" : "Down");
            break;
        // any button longpress event will cycle menu level 0->1->2->3->0
        case event_t::longPress :
            menu_toggle();
            break;
    }
};

// this function will show how to use AutoRepeat and MultiCLicks
void counters(event_t e, const EventMsg* m){
    switch(e){
    // on a single click we will instruct user to do something more with buttons
    case event_t::click :
        if (m->gpio == BUTTON_1) Serial.println("Hold button1 for autorepeat...");
        else Serial.println("Single click is not that fun, try double or triple clicks...");
        break;
    // autorepeat action
    case event_t::autoRepeat :
        Serial.print("*");
        break;
    // multiclicks
    case event_t::multiClick :
        Serial.printf("gpio: %d clicked %u times. ", m->gpio, m->cntr);
        Serial.println("Click 5 times to exit to menu 0");
        // toggle menu on 5th clicks
        if (m->cntr == 5)
            menu_toggle();
        break;
    }
    // since we use autorepeat here we can no longer use longPress to toggle menu, because autorepeat is triggered after longPress
    //case event_t::longPress :
    //    menu_toggle();
    //    break;
};
