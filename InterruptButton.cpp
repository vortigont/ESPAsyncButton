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
#define IBUTTON_Q_DEPTH               5
#endif

#ifndef IBUTTON_EVT_TASK_PRIO
#define IBUTTON_EVT_TASK_PRIO         2             // a bit higher than arduino's loop()
#endif

#ifndef IBUTTON_EVT_TASK_STACK
#define IBUTTON_EVT_TASK_STACK        4096
#endif

#define IBUTTON_EVT_TASK_NAME         "BTN_ACTN"
#define ESP_INTR_FLAG_DEFAULT         0

static const char* TAG = "IBTN";      // IDF log tag

QueueHandle_t   q_action = nullptr;
TaskHandle_t    t_action = nullptr;
LList<btnaction_t> btn_actions;

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

void btnmenu_t::eventSet(event_t evt, bool state, uint8_t menulvl){
  if (menulvl >= IBTN_MAX_MENU_DEPTH)
    return;

  switch(evt){
    case event_t::LongKeyPress :
      longpress.set(menulvl, state);
      break;
    case event_t::AutoRepeatKeyPress :
      repeat.set(menulvl, state);
      break;
    case event_t::MultiClick :
      click.set(menulvl, state);
      break;
    case event_t::AnyEvent :
      longpress.set(menulvl, state);
      repeat.set(menulvl, state);
      click.set(menulvl, state);
  }

}

bool btnmenu_t::eventGet(event_t evt, uint8_t menulvl) const {
  if (menulvl >= IBTN_MAX_MENU_DEPTH)
    return false;

  switch(evt){
    case event_t::LongKeyPress :
      return longpress.test(menulvl);
    case event_t::AutoRepeatKeyPress :
      return repeat.test(menulvl);
    case event_t::MultiClick :
      return click.test(menulvl);
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
    case btnState_t::Released:                                  // Was sitting released but just detected a signal from the button
      m_state = btnState_t::PressDebounce;
      break;
    case btnState_t::PressOnHold:
      m_state = btnState_t::ReleaseDebounce;
      break;
    case btnState_t::PressOnLongHold:
      m_state = btnState_t::ReleaseLongDebounce;
      break;
    default:
      return;                                                   // ignore unknown cases
  }

  gpio_intr_disable(m_pin);                                     // disable gpio ISR while we poll for a valid press (debouncer)
  debounce_ctr = 0;                                             // reset debounce counter
  if (m_DebounceTimer)
    esp_timer_start_periodic(m_DebounceTimer, m_gpio_debounceUS / IBTN_DEBOUNCE_CNT);
}

//-- Method to monitor button, called by button change and various timer interrupts ----------------------
void InterruptButton::readButton(){
  if (!q_action)    // quit if no Q
    return;

  //BaseType_t xCntxSwitch = pdFALSE;
  btntrigger_t t = { m_pin, m_menu.level, event_t::KeyDown, 0};

  switch(m_state){
    case btnState_t::PressDebounce: {                                         // we get here each time the debounce timer expires (onchange interrupt disabled remember)
      if(gpio_get_level(m_pin) == m_pressedState)
        ++debounce_ctr;
      else
        --debounce_ctr;

      // if debounce counter reached negative threshold => false press or noise
      if (debounce_ctr == -1*IBTN_DEBOUNCE_CNT){
        m_state = btnState_t::Released;
        esp_timer_stop(m_DebounceTimer);                                      // stop debounce timer
        gpio_intr_enable(m_pin);                                              // enable gpio interrupt
        return;
      }

      if (debounce_ctr < IBTN_DEBOUNCE_CNT)
        return;                                                               // keep debouncing

      // if debounce counter reached positive threshold => confirmed keypress
      m_state = btnState_t::PressDown;                                        // change button state to confirmed keypress
      // spill sthrough here to pinState_t::PressDown
    }

    case btnState_t::PressDown: {                                             // VALID KEYDOWN, assumed pressed if it had valid polls more than half the time
      esp_timer_stop(m_DebounceTimer);                                        // cancel debounce polling

      t.event = event_t::KeyDown;
      xQueueSendToBack(q_action, &t, (TickType_t) 0);                         // queue KeyDown event

      if (m_menu.eventGet(event_t::LongKeyPress) || m_menu.eventGet(event_t::AutoRepeatKeyPress))  // set long_press timeout if longpress/repeat is active on current menulevel
        esp_timer_start_once(m_LongPressTimer, m_longKeyPressMS * 1000);

      m_state = btnState_t::PressOnHold;
      gpio_intr_enable(m_pin);                                                 // Begin monitoring pin again
      break;
    }

    case btnState_t::ReleaseDebounce :
    case btnState_t::ReleaseLongDebounce : {
      if(gpio_get_level(m_pin) == m_pressedState)
        --debounce_ctr;
      else
        ++debounce_ctr;

      // if debounce counter reached negative threshold => false release or noise
      if (debounce_ctr == -1*IBTN_DEBOUNCE_CNT){
        if (m_state == btnState_t::ReleaseLongDebounce)
          m_state = btnState_t::PressOnLongHold;
        else
          m_state = btnState_t::PressOnHold;

        esp_timer_stop(m_DebounceTimer);                                      // stop debounce timer
        gpio_intr_enable(m_pin);                                              // enable gpio interrupt
        return;
      }

      // if debounce counter reached positive threshold => confirmed KeyRelease
      if (debounce_ctr < IBTN_DEBOUNCE_CNT)                                   // keep debouncing or monitor for gpio interrupt
        return;

      if (m_state == btnState_t::ReleaseLongDebounce)
        m_state = btnState_t::PressLongRelease;
      else
        m_state = btnState_t::PressRelease;                                 // change button state to confirmed keyrelease
      // spill through here to pinState_t::PressRelease
    }

    case btnState_t::PressRelease:
    case btnState_t::PressLongRelease: {
      esp_timer_stop(m_DebounceTimer);
      if(m_LongPressTimer){                                                   // cancel longpress and autorepeat action on release
        esp_timer_stop(m_LongPressTimer);
        repeat_ctr = 0;
      }

      t.event = event_t::KeyUp;
      xQueueSendToBack(q_action, &t, (TickType_t) 0);                         // queue KeyUp event

      // check if multiclicks are enabled and we are not in LongPress state
      if (m_state == btnState_t::PressRelease && m_menu.eventGet(event_t::MultiClick)){
        esp_timer_stop(m_ClickTimer);                                         // (re)start click timer
        esp_timer_start_once(m_ClickTimer, m_doubleClickMS * 1000);
        ++click_ctr;                                                          // increment click counter
      } else if (m_state == btnState_t::PressRelease) {                       // otherwise queue KeyPress event
        t.event = event_t::KeyPress;
        xQueueSendToBack(q_action, &t, (TickType_t) 0);
      }
      // else is a 'PressLongRelease' state, skip all events

      m_state = btnState_t::Released;                                         // change button state to 'release' and wait for next press/click
      gpio_intr_enable(m_pin);
      return;
    }
  } // End of SWITCH statement

//  if( xCntxSwitch )               // perform a context switch if required
//    portYIELD_FROM_ISR ();
} // End of readButton function


//-- Method to handle longKeyPresses (called by timer)----------------------------------------------------
void InterruptButton::longPressTimeout(){
  if (!q_action || gpio_get_level(m_pin) != m_pressedState)                         // recheck if button is still in "pressed" state
    return;

  m_state = btnState_t::PressOnLongHold;
  btntrigger_t t = { m_pin, m_menu.level, event_t::LongKeyPress, 0};

  // check if repeat on Hold is activated
  if(m_menu.eventGet(event_t::AutoRepeatKeyPress)){
      t.event = repeat_ctr ? event_t::AutoRepeatKeyPress : event_t::LongKeyPress;   // first event is 'LongPress', all consecutive is 'Autorepeat'
      t.param = repeat_ctr++;                                                       // postincrement the counter
      esp_timer_start_once(m_LongPressTimer, m_autoRepeatMS * 1000);
  }

  xQueueSendToBack(q_action, (void*)&t, (TickType_t)0);
}

//-- Method to return to interpret previous keyUp as a keyPress instead of a doubleClick if it times out.
void InterruptButton::clickTimeout(){
  if (!q_action)
    return;

  btntrigger_t t = { m_pin, m_menu.level, event_t::KeyPress};

  // check click counter, if there was only one click, than report it as 'KeyPress', otherwise report as 'MiltipleClick'
  if (click_ctr > 1){
    t.event = event_t::MultiClick;
    t.param = click_ctr;
  }

  click_ctr = 0;
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
InterruptButton::InterruptButton(uint8_t pin, uint8_t pressedState, gpio_mode_t pinMode,
                                 uint16_t longKeyPressMS, uint16_t autoRepeatMS, 
                                 uint16_t doubleClickMS,  uint32_t debounceUS) :
                                 m_pressedState(pressedState),
                                 m_pinMode(pinMode),
                                 m_longKeyPressMS(longKeyPressMS),
                                 m_autoRepeatMS(autoRepeatMS),
                                 m_doubleClickMS(doubleClickMS),
                                 m_gpio_debounceUS(debounceUS) {

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

    startBtnTask();                                     // ensure we have a Q to work on and event consumer Task

    createTimer(timer_event_t::debounce, m_DebounceTimer);               // preallocate debounce timer

    gpio_config_t gpio_conf = {};
    gpio_conf.mode = m_pinMode;
    gpio_conf.pin_bit_mask = BIT64(m_pin);
    gpio_conf.pull_down_en = (m_pressedState) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en =   (m_pressedState) ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
    gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&gpio_conf);                                                    // configure GPIO with the given settings

    m_state = (gpio_get_level(m_pin) == m_pressedState) ? btnState_t::PressOnHold : btnState_t::Released;    // Set to current state when initialising

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);            // it' OK to call this function multiple times
    ESP_LOGD(TAG, "GPIO ISR service installed with exit status: %d", err);

    debounce_ctr = click_ctr = repeat_ctr = 0;                                  // clear counters

    gpio_isr_handler_add(m_pin, InterruptButton::isr_handler, static_cast<void*>(this));
}


//-- TIMING INTERVAL GETTERS AND SETTERS -----------------------------------------------------------------
void      InterruptButton::setLongPressInterval(uint16_t intervalMS)    { m_longKeyPressMS = intervalMS; }
void      InterruptButton::setAutoRepeatInterval(uint16_t intervalMS)   { m_autoRepeatMS = intervalMS;   }
void      InterruptButton::setDoubleClickInterval(uint16_t intervalMS)  { m_doubleClickMS = intervalMS;  }


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

  if (m_menu.eventGet(event_t::AutoRepeatKeyPress) || m_menu.eventGet(event_t::LongKeyPress))
    createTimer(timer_event_t::longpress, m_LongPressTimer);

  if (m_menu.eventGet(event_t::MultiClick))
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
  m_state = btnState_t::Undefined;
}

void InterruptButton::setMenuLevel(uint8_t level){
  if (level > IBTN_MAX_MENU_DEPTH)
    return;

  m_menu.level = level;

  if (m_menu.eventGet(event_t::AutoRepeatKeyPress) || m_menu.eventGet(event_t::LongKeyPress))
    createTimer(timer_event_t::longpress, m_LongPressTimer);
  else
    killTimer(m_LongPressTimer);

  if (m_menu.eventGet(event_t::MultiClick))
    createTimer(timer_event_t::click, m_ClickTimer);
  else
    killTimer(m_ClickTimer);

}
