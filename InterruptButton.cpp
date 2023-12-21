#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "InterruptButton.h"
#include "LList.h"

#ifdef ARDUINO
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
#endif

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
#define ESP_INTR_FLAG_DEFAULT         0

using ESPButton::event_t;
using ESPButton::btnaction_t;
using ESPButton::btntrigger_t;

static const char* TAG = "IBTN";      // IDF log tag

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

namespace ESPButton {

bool startBtnTask(){
  if (!getActionQ())
    return false;

  if (!t_action)
      return xTaskCreate(actionsTask, IBUTTON_EVT_TASK_NAME, IBUTTON_EVT_TASK_STACK, NULL, IBUTTON_EVT_TASK_PRIO, &t_action) == pdPASS;
  else
      return true;  
}

void stopBtnTask(){
  if (t_action){
    vTaskDelete(t_action);
    t_action = nullptr;    
  }

  if (q_action){
    vQueueDelete(q_action);
    q_action = nullptr;
  }
}
} // namespace ESPButton

void InterruptButton::btnmenu_t::eventSet(event_t evt, bool state, uint8_t menulvl){
  if (menulvl >= IBTN_MAX_MENU_DEPTH)
    return;

  switch(evt){
    case event_t::longPress :
      longpress.set(menulvl, state);
      break;
    case event_t::autoRepeat :
      repeat.set(menulvl, state);
      break;
    case event_t::multiClick :
      click.set(menulvl, state);
      break;
    case event_t::anyClick :
      longpress.set(menulvl, state);
      repeat.set(menulvl, state);
      click.set(menulvl, state);
      break;
    default:;
  }

}

bool InterruptButton::btnmenu_t::eventGet(event_t evt, uint8_t menulvl) const {
  if (menulvl >= IBTN_MAX_MENU_DEPTH)
    return false;

  switch(evt){
    case event_t::longPress :
      return longpress.test(menulvl);
    case event_t::autoRepeat :
      return repeat.test(menulvl);
    case event_t::multiClick :
      return click.test(menulvl);
    default:;
  }
  return true;      // all other events are enabled by default

}



// === InterruptButton methods ===

void IRAM_ATTR InterruptButton::isr_handler(void* arg){
  if (!q_action)    // quit if no Q
    return;

  static_cast<InterruptButton*>(arg)->gpio_update_from_isr();
}

void IRAM_ATTR InterruptButton::gpio_update_from_isr(){

  switch(m_state){
    case btnState_t::idle:                                  // Was sitting idle but just detected a signal from the button
      m_state = btnState_t::pressDebounce;
      break;
    case btnState_t::onHold:
      m_state = btnState_t::releaseDebounce;
      break;
    case btnState_t::onLongHold:
      m_state = btnState_t::releaseLongDebounce;
      break;
    default:
      // have an interrupt while in a wrong state, disable button
      m_state = btnState_t::undefined;
      gpio_intr_disable(m_pin);
      return;
  }

  gpio_intr_disable(m_pin);                                     // disable gpio ISR while we poll for a valid press (debouncer)
  _ctr.debounce = 0;                                             // reset debounce counter
  if (m_DebounceTimer)
    esp_timer_start_periodic(m_DebounceTimer, _t.debounce / IBTN_DEBOUNCE_CNT);
}

//-- Method to monitor button, called by button change and various timer interrupts ----------------------
void InterruptButton::readButton(){
  if (!q_action)    // quit if no Q
    return;

  //BaseType_t xCntxSwitch = pdFALSE;
  btntrigger_t t = { m_pin, m_menu.level, event_t::press, 0};

  switch(m_state){
    case btnState_t::pressDebounce: {                                         // we get here each time the debounce timer expires (onchange interrupt disabled remember)
      if (gpio_get_level(m_pin) == _logicLevel)
        ++_ctr.debounce;
      else
        --_ctr.debounce;

      // if debounce counter reached negative threshold => false press or noise
      if (_ctr.debounce == -1*IBTN_DEBOUNCE_CNT){
        m_state = btnState_t::idle;
        esp_timer_stop(m_DebounceTimer);                                      // stop debounce timer
        gpio_intr_enable(m_pin);                                              // enable gpio interrupt
        return;
      }

      if (_ctr.debounce < IBTN_DEBOUNCE_CNT)
        return;                                                               // keep debouncing

      // if debounce counter reached positive threshold => confirmed click
      m_state = btnState_t::pressed;                                          // change button state to confirmed click
    }
    // spill sthrough here to pinState_t::pressed
    [[fallthrough]];

    case btnState_t::pressed: {                                               // VALID press, assumed pressed if it had valid polls more than half the time
      esp_timer_stop(m_DebounceTimer);                                        // cancel debounce polling

      t.event = event_t::press;
      xQueueSendToBack(q_action, &t, (TickType_t) 0);                         // queue press event

      if (m_menu.eventGet(event_t::longPress) || m_menu.eventGet(event_t::autoRepeat))  // set long_press timeout if longpress/repeat is active on current menulevel
        esp_timer_start_once(m_LongPressTimer, _t.longPress * 1000);

      m_state = btnState_t::onHold;
      gpio_intr_enable(m_pin);                                                 // Begin monitoring pin again
      break;
    }

    case btnState_t::releaseDebounce :
    case btnState_t::releaseLongDebounce : {
      if(gpio_get_level(m_pin) == _logicLevel)
        --_ctr.debounce;
      else
        ++_ctr.debounce;

      // if debounce counter reached negative threshold => false release or noise
      if (_ctr.debounce == -1*IBTN_DEBOUNCE_CNT){
        if (m_state == btnState_t::releaseLongDebounce)
          m_state = btnState_t::onLongHold;
        else
          m_state = btnState_t::onHold;

        esp_timer_stop(m_DebounceTimer);                                      // stop debounce timer
        gpio_intr_enable(m_pin);                                              // enable gpio interrupt
        return;
      }

      // if debounce counter reached positive threshold => confirmed KeyRelease
      if (_ctr.debounce < IBTN_DEBOUNCE_CNT)                                   // keep debouncing or monitor for gpio interrupt
        return;

      if (m_state == btnState_t::releaseLongDebounce)
        m_state = btnState_t::releasedLong;
      else
        m_state = btnState_t::released;                                 // change button state to confirmed keyrelease
    }
    // spill through here to pinState_t::released
    [[fallthrough]];

    case btnState_t::released:
    case btnState_t::releasedLong: {
      esp_timer_stop(m_DebounceTimer);
      if(m_LongPressTimer){                                                   // cancel longpress and autorepeat action on release
        esp_timer_stop(m_LongPressTimer);
        _ctr.repeat = 0;
      }

      t.event = m_state == btnState_t::released ? event_t::release : event_t::longRelease;
      xQueueSendToBack(q_action, &t, (TickType_t) 0);                         // queue release event

      // check if multiclicks are enabled and we are not in LongPress state
      if (m_state == btnState_t::released && m_menu.eventGet(event_t::multiClick)){
        esp_timer_stop(m_ClickTimer);                                         // (re)start click timer
        esp_timer_start_once(m_ClickTimer, _t.multiClick * 1000);
        ++_ctr.click;                                                         // increment click counter
      } else if (m_state == btnState_t::released) {                           // otherwise queue click event
        t.event = event_t::click;
        xQueueSendToBack(q_action, &t, (TickType_t) 0);
      }
      // else is a 'releasedLong' state, reserved for future
      //t.event = event_t::releasedLong;
      //xQueueSendToBack(q_action, &t, (TickType_t) 0);

      m_state = btnState_t::idle;                                         // change button state to 'release' and wait for next press/click
      gpio_intr_enable(m_pin);
      return;
    }
    default:;
  } // End of SWITCH statement

//  if( xCntxSwitch )               // perform a context switch if required
//    portYIELD_FROM_ISR ();
} // End of readButton function


//-- Method to handle longKeyPresses (called by timer)----------------------------------------------------
void InterruptButton::longPressTimeout(){
  if (!q_action || gpio_get_level(m_pin) != _logicLevel)                         // recheck if button is still in "pressed" state
    return;

  m_state = btnState_t::onLongHold;
  btntrigger_t t = { m_pin, m_menu.level, event_t::longPress, 0};

  // check if repeat on Hold is activated
  if(m_menu.eventGet(event_t::autoRepeat)){
      t.event = _ctr.repeat ? event_t::autoRepeat : event_t::longPress;         // first event is 'LongPress', all consecutive is 'Autorepeat'
      t.param = _ctr.repeat++;                                                  // postincrement the counter
      esp_timer_start_once(m_LongPressTimer, _t.autoRepeat * 1000);
  }

  xQueueSendToBack(q_action, (void*)&t, (TickType_t)0);
}

//-- Method to return to interpret previous release as a click instead of a doubleClick if it times out.
void InterruptButton::clickTimeout(){
  if (!q_action)
    return;

  btntrigger_t t = { m_pin, m_menu.level, event_t::click};

  // check click counter, if there was only one click, than report it as 'click', otherwise report as 'MiltipleClick'
  if (_ctr.click > 1){
    t.event = event_t::multiClick;
    t.param = _ctr.click;
  }

  _ctr.click = 0;
  xQueueSendToBack(q_action, (void*)&t, (TickType_t)0);
}

//-- Helper method to simplify starting a timer ----------------------------------------------------------
void InterruptButton::createTimer(timer_event_t tevent, esp_timer_handle_t &timer){
  if (timer != nullptr)
    return;         // timer already exist

  esp_timer_create_args_t tmrConfig;

  tmrConfig.arg = static_cast<void*>(this);
  tmrConfig.dispatch_method = ESP_TIMER_TASK;
  tmrConfig.skip_unhandled_events = true;

  switch (tevent)  {
    case timer_event_t::debounce :
      tmrConfig.callback = [](void* self) { static_cast<InterruptButton*>(self)->readButton(); };
      tmrConfig.name = "btn_dbnc";
      break;

    case timer_event_t::click :
      tmrConfig.callback = [](void* self) { static_cast<InterruptButton*>(self)->clickTimeout(); };
      tmrConfig.name = "btn_clck";
      break;

    case timer_event_t::longpress :
      tmrConfig.callback = [](void* self) { static_cast<InterruptButton*>(self)->longPressTimeout(); };
      tmrConfig.name = "btn_lp";
      break;

    default:
      return;
  }

  esp_timer_create(&tmrConfig, &timer);

}

//-- Helper method to kill a timer -----------------------------------------------------------------------
void InterruptButton::killTimer(esp_timer_handle_t &timer){
  if(timer){
    esp_timer_stop(timer);
    esp_timer_delete(timer);
    timer = nullptr;
  }
}


//-- CLASS MEMBERS AND METHODS SPECIFIC TO A SINGLE INSTANCE (BUTTON) ------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------

// Class object control and setup functions -------------------------------------
// ------------------------------------------------------------------------------

// Constructor ------------------------------------------------------------------
InterruptButton::InterruptButton(uint8_t pin, bool logicLevel, gpio_mode_t pinMode) :
                                 _logicLevel(logicLevel),
                                 m_pinMode(pinMode) {

  // gpio number sanity check
  if (GPIO_IS_VALID_GPIO(pin))
    m_pin = static_cast<gpio_num_t>(pin);
  else {
    ESP_LOGW(TAG, "%d is not valid gpio on this platform", pin);
    m_pin = static_cast<gpio_num_t>(-1);    //GPIO_NUM_NC (enum not showing up as defined);
  }

}

// Destructor --------------------------------------------------------------------
InterruptButton::~InterruptButton() {
  disable();
  unbindall();
  gpio_reset_pin(m_pin);
}

// Initialiser -------------------------------------------------------------------
void InterruptButton::enable(){
    if (m_pin == GPIO_NUM_NC)
      return;

    ESPButton::startBtnTask();                                          // ensure we have a Q to work on and event consumer Task

    createTimer(timer_event_t::debounce, m_DebounceTimer);              // preallocate debounce timer

    gpio_config_t gpio_conf = {};
    gpio_conf.mode = m_pinMode;
    gpio_conf.pin_bit_mask = BIT64(m_pin);
    gpio_conf.pull_down_en = (_logicLevel) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en =   (_logicLevel) ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
    gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&gpio_conf);                                                    // configure GPIO with the given settings

    m_state = (gpio_get_level(m_pin) == _logicLevel) ? btnState_t::onHold : btnState_t::idle;    // Set to current state when initialising

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);            // it' OK to call this function multiple times
    ESP_LOGD(TAG, "GPIO ISR service installed with exit status: %d", err);

    _ctr.debounce = _ctr.click = _ctr.repeat = 0;                                  // clear counters

    gpio_isr_handler_add(m_pin, InterruptButton::isr_handler, static_cast<void*>(this));
}


//-- TIMING INTERVAL GETTERS AND SETTERS -----------------------------------------------------------------
void      InterruptButton::setLongPressInterval(uint16_t intervalMS)    { _t.longPress = intervalMS; }
void      InterruptButton::setAutoRepeatInterval(uint16_t intervalMS)   { _t.autoRepeat = intervalMS;   }
void      InterruptButton::setMultiClickInterval(uint16_t intervalMS)  { _t.multiClick = intervalMS;  }


// -- FUNCTIONS RELATED TO EXTERNAL ACTIONS --------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------

/*
void InterruptButton::bind(event_t event, uint8_t menuLevel, func_ptr action){
  if(!m_firstButtonInitialised){
    ESP_LOGE(TAG, "You must call the 'begin()' function prior to binding external fuctions to a button!");
  } else if(menuLevel >= m_numMenus) {
    ESP_LOGE(TAG, "Specified menu level is greater than the number of menus!");
  } else if(event >= NumEventTypes) {
    ESP_LOGE(TAG, "Specified event is invalid!");
  } else {
    eventActions[menuLevel][event] = action;
    if(!eventEnabled(event)) enableEvent(event);
  }
}
*/
void InterruptButton::bind(event_t event, btn_callback_t action, uint8_t menuLevel){
  btnaction_t a = { {m_pin, menuLevel, event}, action};
  btn_actions.add(a);
  m_menu.eventSet(event, true, menuLevel);

  if (m_menu.eventGet(event_t::autoRepeat) || m_menu.eventGet(event_t::longPress))
    createTimer(timer_event_t::longpress, m_LongPressTimer);

  if (m_menu.eventGet(event_t::multiClick))
    createTimer(timer_event_t::click, m_ClickTimer);
}

void InterruptButton::unbind(event_t event, uint8_t menuLevel){
  if (!btn_actions.size())    // action list is empty
    return;

  for (int i = 0; i != btn_actions.size(); ++i){
    if (btn_actions.get(i).t.gpio == m_pin && btn_actions.get(i).t.event == event && btn_actions.get(i).t.menulvl == menuLevel){
      m_menu.eventSet(event, false, menuLevel);
      btn_actions.remove(i);
      return;
    }
  }
}

void InterruptButton::unbindall(){
  if (!btn_actions.size())    // action list is empty
    return;

  int size = btn_actions.size();
  for (int i = 0; i != size; ++i){
    if (btn_actions.get(i).t.gpio == m_pin){
      btn_actions.remove(i);
      --size;
      --i;
    }
  }
}

void InterruptButton::disable(){
  gpio_isr_handler_remove(m_pin);
  killTimer(m_DebounceTimer);
  killTimer(m_LongPressTimer);
  killTimer(m_ClickTimer);
  m_state = btnState_t::undefined;
}

void InterruptButton::setMenuLevel(uint8_t level){
  if (level > IBTN_MAX_MENU_DEPTH)
    return;

  m_menu.level = level;

  if (m_menu.eventGet(event_t::autoRepeat) || m_menu.eventGet(event_t::longPress))
    createTimer(timer_event_t::longpress, m_LongPressTimer);
  else
    killTimer(m_LongPressTimer);

  if (m_menu.eventGet(event_t::multiClick))
    createTimer(timer_event_t::click, m_ClickTimer);
  else
    killTimer(m_ClickTimer);

}
