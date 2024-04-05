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

// AsyncButton events
ESP_EVENT_DEFINE_BASE(EBTN_EVENTS);
// Encoder events
ESP_EVENT_DEFINE_BASE(EBTN_ENC_EVENTS);

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

using namespace ESPButton;

// a simple constrain function
template<typename T>
T clamp(T value, T min, T max){
  return (value < min)? min : (value > max)? max : value;
}

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


// === AsyncEventButton methods ===
// d-tor
AsyncEventButton::~AsyncEventButton(){
  // unsunscribe event handler
  if (_evt_handler){
    if (ESPButton::ebtn_hndlr)
      esp_event_handler_instance_unregister_with(ESPButton::ebtn_hndlr, EBTN_EVENTS, ESP_EVENT_ANY_ID, _evt_handler);
    else
      esp_event_handler_instance_unregister(EBTN_EVENTS, ESP_EVENT_ANY_ID, _evt_handler);

    _evt_handler = nullptr;
  }
}

void AsyncEventButton::begin(){
  if (_evt_handler) return;
  // event bus subscription
  if (ESPButton::ebtn_hndlr){
    // subscribe to custom loop handler if set
    ESP_ERROR_CHECK(esp_event_handler_instance_register_with(
                      ESPButton::ebtn_hndlr,
                      EBTN_EVENTS, ESP_EVENT_ANY_ID,
                      [](void* self, esp_event_base_t base, int32_t id, void* data) {
                          if ( reinterpret_cast<EventMsg*>(data)->gpio != static_cast<AsyncEventButton*>(self)->getGPIO() ) return;
                          static_cast<AsyncEventButton*>(self)->_evt_picker(base, id, data);
                      },
                      this,
                      &_evt_handler)
                  );
  } else {
    // otherwise subscribe to default ESP event bus
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
                      EBTN_EVENTS, ESP_EVENT_ANY_ID,
                      [](void* self, esp_event_base_t base, int32_t id, void* data) {
                          if ( reinterpret_cast<EventMsg*>(data)->gpio != static_cast<AsyncEventButton*>(self)->getGPIO() ) return;
                          static_cast<AsyncEventButton*>(self)->_evt_picker(base, id, data);
                      },
                      this,
                      &_evt_handler)
                  );
  }

  // disable all events by default
  deactivateAll();
}

void AsyncEventButton::_evt_picker(esp_event_base_t base, int32_t id, void* data){
  switch(int2event_t(id)){
    case event_t::press :
      if (_cb.press) _cb.press();
      break;
    case event_t::release :
      if (_cb.release) _cb.release();
      break;
    case event_t::click :
      if (_cb.click) _cb.click();
      break;
    case event_t::longPress :
      if (_cb.longPress) _cb.longPress();
      break;
    case event_t::longRelease :
      if (_cb.longRelease) _cb.longRelease();
      break;
    case event_t::autoRepeat :
      if (_cb.autoRepeat) _cb.autoRepeat(reinterpret_cast<EventMsg*>(data)->cntr);
      break;
    case event_t::multiClick :
      if (_cb.multiCLick) _cb.multiCLick(reinterpret_cast<EventMsg*>(data)->cntr);
      break;
  }
}

void AsyncEventButton::onPress(callback_t f){
  _cb.press = f;
  enableEvent(event_t::press, (f != nullptr));
}

void AsyncEventButton::onRelease(callback_t f){
  _cb.release = f;
  enableEvent(event_t::release, (f != nullptr));
}

void AsyncEventButton::onClick(callback_t f){
  _cb.click = f;
  enableEvent(event_t::click, (f != nullptr));
}

void AsyncEventButton::onLongPress(callback_t f){
  _cb.longPress = f;
  enableEvent(event_t::longPress, (f != nullptr));
}

void AsyncEventButton::onLongRelease(callback_t f){
  _cb.longRelease = f;
  enableEvent(event_t::longRelease, (f != nullptr));
}

void AsyncEventButton::onAutoRepeat(callback_cnt_t f){
 _cb.autoRepeat = f;
  enableEvent(event_t::autoRepeat, (f != nullptr));
}

void AsyncEventButton::onMultiClick(callback_cnt_t f){
 _cb.multiCLick = f;
  enableEvent(event_t::multiClick, (f != nullptr));
}


// === PseudoRotaryEncoder methods ===
// c- tor
PseudoRotaryEncoder::PseudoRotaryEncoder(
  gpio_num_t gpio_decr,
  gpio_num_t gpio_inc,
  bool logicLevel,
  gpio_pull_mode_t pull,
  gpio_mode_t mode,
  bool debounce
  ) : 
  _decr(gpio_decr, logicLevel, pull, mode, debounce),
  _incr(gpio_inc, logicLevel, pull, mode, debounce) {}

// d-tor
PseudoRotaryEncoder::~PseudoRotaryEncoder(){
  // unsunscribe event handler
  if (_evt_handler){
    if (ESPButton::ebtn_hndlr)
      esp_event_handler_instance_unregister_with(ESPButton::ebtn_hndlr, EBTN_EVENTS, ESP_EVENT_ANY_ID, _evt_handler);
    else
      esp_event_handler_instance_unregister(EBTN_EVENTS, ESP_EVENT_ANY_ID, _evt_handler);

    _evt_handler = nullptr;
  }
}

void PseudoRotaryEncoder::begin(){
  if (_evt_handler) return;
  // event bus subscription
  if (ESPButton::ebtn_hndlr){
    // subscribe to custom loop handler if set
    ESP_ERROR_CHECK(esp_event_handler_instance_register_with(
                      ESPButton::ebtn_hndlr,
                      EBTN_EVENTS, ESP_EVENT_ANY_ID,
                      [](void* self, esp_event_base_t base, int32_t id, void* data) {
                          static_cast<PseudoRotaryEncoder*>(self)->_evt_picker(base, id, data);
                      },
                      this,
                      &_evt_handler)
                  );
  } else {
    // otherwise subscribe to default ESP event bus
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
                      EBTN_EVENTS, ESP_EVENT_ANY_ID,
                      [](void* self, esp_event_base_t base, int32_t id, void* data) {
                          static_cast<PseudoRotaryEncoder*>(self)->_evt_picker(base, id, data);
                      },
                      this,
                      &_evt_handler)
                  );
  }

  _decr.deactivateAll();
  _incr.deactivateAll();

  _decr.enableEvent(event_t::click);
  _decr.enableEvent(event_t::autoRepeat);
  _incr.enableEvent(event_t::click);
  _incr.enableEvent(event_t::autoRepeat);

  _decr.enable();
  _incr.enable();

}

void PseudoRotaryEncoder::_evt_picker(esp_event_base_t base, int32_t id, void* data){
  // skip foreign gpios
  if (reinterpret_cast<EventMsg*>(data)->gpio != _decr.getGPIO() && reinterpret_cast<EventMsg*>(data)->gpio != _incr.getGPIO())
    return;
  
  int32_t increment = _rc.step;
  // check for decrement and change sign
  if (reinterpret_cast<EventMsg*>(data)->gpio == _decr.getGPIO())
    increment *= -1;
  
  switch(int2event_t(id)){
    case event_t::click :
    case event_t::autoRepeat :
      _updCnt(increment, reinterpret_cast<EventMsg*>(data)->gpio);
      break;
    case event_t::multiClick :
      // m-factor will apply amplification to increment based on number of multiclicks
      _updCnt(increment * reinterpret_cast<EventMsg*>(data)->cntr * _rc.mfactor, reinterpret_cast<EventMsg*>(data)->gpio);
      break;
  }
}

void PseudoRotaryEncoder::setCounter(int32_t value, int32_t step, int32_t min, int32_t max){
  _rc.value = value;
  if (step != 0)
    _rc.step = step;

  if (min < max){
    _rc.min = min;
    _rc.max = max;
    setConstrain(true);
  }
}

void PseudoRotaryEncoder::_updCnt(int32_t increment, int32_t gpio){
  _rc.value += increment;
  if (_rc.constrain && _rc.rollover){
    if (_rc.value < _rc.min)
      _rc.value = _rc.max;
    else if (_rc.value > _rc.max)
      _rc.value = _rc.min;
  } else if (_rc.constrain)
    _rc.value = clamp(_rc.value, _rc.min, _rc.max);

  EventMsg msg{gpio, _rc.value};
  // generate pseudo-encoder event
  if (ESPButton::ebtn_hndlr)
    esp_event_post_to(ESPButton::ebtn_hndlr, EBTN_ENC_EVENTS, static_cast<int32_t>(event_t::encCount), &msg, sizeof(EventMsg), portMAX_DELAY);
  else
    esp_event_post(EBTN_ENC_EVENTS, static_cast<int32_t>(event_t::encCount), &msg, sizeof(EventMsg), portMAX_DELAY);
}

void PseudoRotaryEncoder::setMultiplyFactor(int32_t mfactor){
  if (mfactor > 1){
    _rc.mfactor = mfactor;
    _decr.enableEvent(event_t::multiClick);
    _incr.enableEvent(event_t::multiClick);
  } else {
    _rc.mfactor = 1; 
    _decr.enableEvent(event_t::multiClick, false);
    _incr.enableEvent(event_t::multiClick, false);
  }
}

void PseudoRotaryEncoder::setConstrain(bool constrain){
  if (constrain && _rc.min < _rc.max)
    _rc.constrain = true;
  else
    _rc.constrain = false;
};