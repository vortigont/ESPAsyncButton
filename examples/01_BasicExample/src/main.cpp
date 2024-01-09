#include "Arduino.h"
#include "espasyncbutton.hpp"
#include <array>

#define BUTTON_1  GPIO_NUM_0                 // Top Left or Bottom Right button
#define BUTTON_2  GPIO_NUM_35                // Bottom Left or Top Right button

using ESPButton::event_t;

static constexpr std::array<const char *, 8> event_name = {
    "press",
    "release",
    "click",
    "longPress",
    "longRelease",
    "autoRepeat",
    "multiClick",
    "undefined"
};

GPIOButton<ESPEventPolicy> b1(BUTTON_1, LOW);
GPIOButton<ESPEventPolicy> b2(BUTTON_2, LOW);

void evt_hndlr(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);

void setup(){
    Serial.begin(115200);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //ESP_ERROR_CHECK(esp_event_handler_instance_register_with(evt::get_hndlr(), ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, Lamp::event_hndlr, this, &_events_lamp_cmd));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(EBTN_EVENTS, ESP_EVENT_ANY_ID, evt_hndlr, NULL, NULL));
    b1.enable();
    b2.enable();

    b1.enableEvent(event_t::press, false);
    b1.enableEvent(event_t::release, false);
    b1.enableEvent(event_t::click);
    b1.enableEvent(event_t::longPress);
    //b1.enableEvent(event_t::longRelease);
    b1.enableEvent(event_t::autoRepeat);
    b1.enableEvent(event_t::multiClick);

    b2.enableEvent(event_t::multiClick);

}

void loop(){
    delay(100);
}

void evt_hndlr(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
    EventMsg *e = reinterpret_cast<EventMsg*>(event_data);
    const char* event =  id < 0 || id >= event_name.size() ? event_name.at(7) : event_name.at(id);
    Serial.printf( "Event %s:%s gpio:%d, ctr:%u\n", base, event, e->gpio, e->cntr);

}
