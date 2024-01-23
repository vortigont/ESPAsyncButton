//#include "freertos/FreeRTOS.h"
//#include "freertos/queue.h"
#include "espasyncbutton.hpp"


#ifndef IBUTTON_Q_DEPTH
#define IBUTTON_Q_DEPTH               10
#endif

#ifndef IBUTTON_EVT_TASK_PRIO
#define IBUTTON_EVT_TASK_PRIO         2             // a bit higher than arduino's loop()
#endif

#ifndef IBUTTON_EVT_TASK_STACK
#define IBUTTON_EVT_TASK_STACK        4096
#endif

#define IBUTTON_EVT_TASK_NAME         "BTN_ACTN"

using ESPButton::event_t;

ESP_EVENT_DEFINE_BASE(EBTN_EVENTS);

/*
QueueHandle_t   q_action = nullptr;
TaskHandle_t    t_action = nullptr;
LList<ESPButton::btnaction_t> btn_actions;

QueueHandle_t* getActionQ(){
  if (q_action)
    return &q_action;

  q_action = xQueueCreate( IBUTTON_Q_DEPTH, sizeof(btntrigger_t) ); // make q for button actions

  return &q_action;
}

void actionsTask(void* pvParams){
  btntrigger_t t;

  for (;;){
    if (xQueueReceive(q_action, (void*)&t, (portTickType)portMAX_DELAY)) {
      for (const auto& i : btn_actions){                                   // search for registered callback functions
        if (i.t == t){                                                     // execute callback if btntrigger_t matches the one provided via Q
          i.cb();
          break;
        }
      }
    }
  }
  vTaskDelete(NULL);
}
*/
namespace ESPButton {

/**
 * @brief event loop handler where to post button events
 * if not set, then post events to system default event loop
 */
static esp_event_loop_handle_t ebtn_hndlr = nullptr;

esp_event_loop_handle_t get_evthndlr(){ return ebtn_hndlr; };

void set_event_loop_hndlr( esp_event_loop_handle_t handler){ ebtn_hndlr = handler; };

event_t int2event_t(int32_t e){
  return (e < 0 || e >= IBTN_EVENT_T_SIZE) ? event_t::undefined : static_cast<event_t>(e);
}

/*
bool startBtnQTask(){
  if (!getActionQ())
    return false;

  if (!t_action)
      return xTaskCreate(actionsTask, IBUTTON_EVT_TASK_NAME, IBUTTON_EVT_TASK_STACK, NULL, IBUTTON_EVT_TASK_PRIO, &t_action) == pdPASS;
  else
      return true;  
}
void stopBtnQTask(){
  if (t_action){
    vTaskDelete(t_action);
    t_action = nullptr;    
  }

  if (q_action){
    vQueueDelete(q_action);
    q_action = nullptr;
  }
}
*/

} // namespace ESPButton

void ButtonCallbackMenu::assign(int32_t gpio, uint32_t menuLevel, btn_callback_t callback){
  callbacks.emplace_back(EventCallback(menuLevel, gpio, callback));
}

void ButtonCallbackMenu::deassign(int32_t gpio, uint32_t menuLevel){
  callbacks.remove_if( ESPButton::MatchEventCallback<EventCallback>(gpio, menuLevel) );
}

void ButtonCallbackMenu::handleEvent(ESPButton::event_t event, const EventMsg* m){
  // I need independend copy of _level in case any callback will trigger level change
  // so to avoid recoursive loop I compare to a local value of _level
  uint32_t _pendinglevel = _level;

  for (auto &it : callbacks ){
    //Serial.printf("i.m:%u, i.g:%d,  m:%u,g:%d\n", it.menulvl, it.gpio, _level, m->gpio);
    if (it.menulvl == _pendinglevel && it.gpio == m->gpio)
      it.cb(event, m);
  }

/*
  auto it = std::find_if(callbacks.begin(), callbacks.end(), MatchEventCallback<EventCallback>(gpio, menuLevel));
  if (it != callbacks.end()){
    it.cb = cb;
  }
*/
}


// === ESPEventPolicy methods ===

void ESPEventPolicy::event(ESPButton::event_t e, EventMsg *msg){
  if (ESPButton::ebtn_hndlr)
    esp_event_post_to(ESPButton::ebtn_hndlr, EBTN_EVENTS, static_cast<int32_t>(e), msg, sizeof(EventMsg), portMAX_DELAY);
  else
    esp_event_post(EBTN_EVENTS, static_cast<int32_t>(e), msg, sizeof(EventMsg), portMAX_DELAY);
}

void TimeOuts::setDebounce( uint32_t us ){
  debounce = us < IBTN_DEBOUNCE_MIN_TIME_US ? IBTN_DEBOUNCE_MIN_TIME_US : us;
  debounce = us > IBTN_DEBOUNCE_MAX_TIME_US ? IBTN_DEBOUNCE_MAX_TIME_US : us;
}
