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

#define IBUTTON_EVT_TASK_NAME                 "BTN_ACTN"
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


void IRAM_ATTR InterruptButton::isr_handler(void* arg){
  if (!q_action)    // quit if no Q
    return;

  InterruptButton* btn = static_cast<InterruptButton*>(arg);
  btn->gpio_update_from_isr();
}

void InterruptButton::timers_handler(timer_event_t cbt){
  switch(cbt){
    case  timer_event_t::debounce:
      readButton();
      return;
    case  timer_event_t::longpress:
      longPressEvent();
      return;
    case  timer_event_t::click:
      clickTimeout();
    default:
      return;
  }
}


void IRAM_ATTR InterruptButton::gpio_update_from_isr(){

  switch(m_state){
    case pinState_t::Released:                                  // Was sitting released but just detected a signal from the button
      m_state = pinState_t::PressDebounce;
      break;
    case pinState_t::PressOnHold:
      m_state = pinState_t::ReleaseDebounce;
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
  btntrigger_t t = { m_pin, m_menuLevel, event_t::KeyDown, 0};

  switch(m_state){
    case pinState_t::PressDebounce: {                                         // we get here each time the debounce timer expires (onchange interrupt disabled remember)
      if(gpio_get_level(m_pin) == m_pressedState)
        ++debounce_ctr;
      else
        --debounce_ctr;

      // if debounce counter reached positive threshold => confirmed keypress
      if (debounce_ctr == IBTN_DEBOUNCE_CNT){
        m_state = pinState_t::PressDown;                                      // change button state to confirmed keypress
        return;                                                               // return, on a next timer call press event will be processed
      }

      // if debounce counter reached negative threshold => false press or noise
      if (debounce_ctr == -1*IBTN_DEBOUNCE_CNT){
        m_state = pinState_t::Released;
        esp_timer_stop(m_DebounceTimer);                                      // stop debounce timer
        gpio_intr_enable(m_pin);                                              // enable gpio interrupt
      }

      // keep debouncing or monitor for gpio interrupt
      return;
    }

    case pinState_t::PressDown: {                                             // VALID KEYDOWN, assumed pressed if it had valid polls more than half the time
      esp_timer_stop(m_DebounceTimer);                                        // cancel debounce polling

      t.event = event_t::KeyDown;
      xQueueSendToBack(q_action, &t, (TickType_t) 0);                         // queue KeyDown event

      if (menu_longpress.test(m_menuLevel))                                   // set long_press timeout if longpress is active on current menulevel
        esp_timer_start_once(m_LongPressTimer, m_longKeyPressMS * 1000);

      m_state = pinState_t::PressOnHold;
      gpio_intr_enable(m_pin);                                                 // Begin monitoring pin again
      break;
    }

    case pinState_t::ReleaseDebounce: {
      if(gpio_get_level(m_pin) == m_pressedState)
        --debounce_ctr;
      else
        ++debounce_ctr;

      // if debounce counter reached positive threshold => confirmed KeyRelease
      if (debounce_ctr == IBTN_DEBOUNCE_CNT){
        m_state = pinState_t::PressRelease;                                   // change button state to confirmed keyrelease
        return;                                                               // return, on a next timer call release event will be processed
      }

      // if debounce counter reached negative threshold => false release or noise
      if (debounce_ctr == -1*IBTN_DEBOUNCE_CNT){
        m_state = pinState_t::PressOnHold;
        esp_timer_stop(m_DebounceTimer);                                      // stop debounce timer
        gpio_intr_enable(m_pin);                                              // enable gpio interrupt
      }

      // keep debouncing or monitor for gpio interrupt
      return;
    }
/**
    case pinState_t::ReleaseDebounce:       // we get here when debounce timer or doubleclick timeout timer alarms (onchange interrupt disabled remember)
                            // stay in this state until released, because button could remain locked down if release missed.
      btn->m_totalPolls++;
      if(gpio_get_level(btn->m_pin) != btn->m_pressedState){
        btn->m_validPolls++;
        if(btn->m_totalPolls < m_targetPolls || btn->m_validPolls * 2 <= btn->m_totalPolls) {           // If we haven't polled enough or not high enough success rate
          startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &InterruptButton::readButton, btn, "W4R_polling_");  // Then keep sampling pin state until release is confirmed
          return;
        }                                                                                 // Otherwise, spill through to "Releasing"
      } else {
        if(btn->m_validPolls > 0) {
          btn->m_validPolls--;
        } else {
          btn->m_totalPolls = 0;                                                          // Key is being held down, don't let total polls get too far ahead.
        }
        startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &InterruptButton::readButton, btn, "W4R_invalidPoll"); // Keep sampling pin state until released
      }
      //btn->m_state = pinState_t::PressRelease;
    // Intended spill through here to "Releasing" once keyUp confirmed.
**/

    case pinState_t::PressRelease: {
      esp_timer_stop(m_DebounceTimer);
      if(menu_repeat.test(m_menuLevel)){                                      // cancel longpress and autorepeat action on release
        esp_timer_stop(m_LongPressTimer);
        repeat_ctr = 0;
      }

      t.event = event_t::KeyUp;
      xQueueSendToBack(q_action, &t, (TickType_t) 0);                         // queue KeyUp event

      if (menu_dblclck.test(m_menuLevel)){                                    // if click events are active at current menu level
        esp_timer_stop(m_ClickTimer);                                         // (re)start click timer
        esp_timer_start_once(m_ClickTimer, m_doubleClickMS * 1000);
        ++click_ctr;                                                          // increment click counter
      } else {                                                                // otherwise queue KeyPress event
        t.event = event_t::KeyPress;
        xQueueSendToBack(q_action, &t, (TickType_t) 0);
      }

      m_state = pinState_t::Released;                                         // change button state to 'release' and wait for next press/click
      gpio_intr_enable(m_pin);
      return;

/*
      // If double-clicks are enabled and either defined
      if(btn->eventEnabled(event_t::DoubleClick) ) {
        if(btn->m_stateDblClick == pinState_t::DblClickIdle && !btn->m_longPress_preventKeyPress) {   // Commence detection process
          btn->m_stateDblClick = pinState_t::DblClickWaiting;
          startTimer(btn->m_buttonDoubleClickTimer, uint64_t(btn->m_doubleClickMS * 1000), &InterruptButton::doubleClickTimeout, btn, "W4R_DCsetup_");

        } else if (btn->m_stateDblClick == pinState_t::DblClickWaiting) {                             // VALID DOUBLE-CLICK (second keyup without a timeout)
          killTimer(btn->m_buttonDoubleClickTimer);
          btn->m_stateDblClick = pinState_t::DblClickIdle;
          t.event = event_t::DoubleClick;
          xQueueSendToBackFromISR(q_action, &t, &xCntxSwitch);
        }
      } else if(!btn->m_longPress_preventKeyPress) {                                      // Otherwise, treat as a basic keyPress
        t.event = event_t::KeyPress;
        xQueueSendToBackFromISR(q_action, &t, &xCntxSwitch);
      } 
      btn->m_state = pinState_t::Released;
      gpio_intr_enable(btn->m_pin);
      break;
*/
    }
  } // End of SWITCH statement

//  if( xCntxSwitch )               // perform a context switch if required
//    portYIELD_FROM_ISR ();
} // End of readButton function


//-- Method to handle longKeyPresses (called by timer)----------------------------------------------------
void InterruptButton::longPressEvent(){
  if (!q_action || gpio_get_level(m_pin) != m_pressedState)                         // recheck if button is still in "pressed" state
    return;

  btntrigger_t t = { m_pin, m_menuLevel, event_t::LongKeyPress, 0};

  // cehck if repeat on Hold is activated
  if(menu_repeat.test(m_menuLevel)){
      t.event = repeat_ctr ? event_t::LongKeyPress : event_t::AutoRepeatPress;      // first event is 'LongPress', all consecutive is 'Autorepeat'
      t.param = repeat_ctr++;                                                       // postincrement the counter
      esp_timer_start_once(m_LongPressTimer, m_autoRepeatMS * 1000);
  }

  xQueueSendToBack(q_action, (void*)&t, (TickType_t)0);
}

//-- Method to handle autoRepeatPresses (called by timer)-------------------------------------------------
/*
void InterruptButton::autoRepeatPressEvent(){
  if (!q_action)
    return;

  InterruptButton* btn = static_cast<InterruptButton*>(arg);

  btntrigger_t t = { btn->m_pin, btn->m_menuLevel, event_t::KeyPress};

  if(btn->eventEnabled(event_t::DoubleClick)) {
    t.event = event_t::AutoRepeatPress;
  }

  if(gpio_get_level(btn->m_pin) == btn->m_pressedState) {                             // Sanity check to stop autorepeats in case we somehow missed button release
    startTimer(btn->m_buttonLPandRepeatTimer, uint64_t(btn->m_autoRepeatMS * 1000), &InterruptButton::autoRepeatPressEvent, btn, "ARPE_");
  }
  xQueueSendToBack(q_action, (void*)&t, (TickType_t)0);
}
*/

//-- Method to return to interpret previous keyUp as a keyPress instead of a doubleClick if it times out.
void InterruptButton::clickTimeout(){
  if (!q_action)
    return;

  btntrigger_t t = { m_pin, m_menuLevel, event_t::KeyPress};

  // check click counter, if there was only one click, than report it as 'KeyPress', otherwise report as 'MiltipleClick'
  if (click_ctr > 0){
    t.event = event_t::MultiClick;
    t.param = click_ctr;
    click_ctr = 0;
  }

  xQueueSendToBack(q_action, (void*)&t, (TickType_t)0);
}

//-- Helper method to simplify starting a timer ----------------------------------------------------------
//void IRAM_ATTR InterruptButton::startTimer(esp_timer_handle_t &timer, uint32_t duration_US, void (*callBack)(void* arg), InterruptButton* btn, const char *msg){
/*
void InterruptButton::startTimer(esp_timer_handle_t &timer, uint64_t duration_US, timer_event_t cbtype, const char *name){
  esp_timer_create_args_t tmrConfig;
  tmrConfig.arg = static_cast<void*>(this);
  tmrConfig.callback = std::bind(&InterruptButton::timer_handler, this, cbtype);
  tmrConfig.dispatch_method = ESP_TIMER_TASK;
  tmrConfig.name = name;
  killTimer(timer);
  esp_timer_create(&tmrConfig, &timer);
  esp_timer_start_once(timer, duration_US);
}
*/

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
  gpio_isr_handler_remove(m_pin);
  killTimer(m_DebounceTimer); killTimer(m_LongPressTimer); killTimer(m_ClickTimer);

  unbindall();
  gpio_reset_pin(m_pin);
}

// Initialiser -------------------------------------------------------------------
void InterruptButton::begin(void){
    if (m_pin == GPIO_NUM_NC)
      return;

    startBtnTask();                                     // enshure we have a Q to work on and event consumer Task

    // create debounce timer
    esp_timer_create_args_t tmrConfig;
    tmrConfig.callback = [](void* self) { static_cast<InterruptButton*>(self)->timers_handler(timer_event_t::debounce); };
    tmrConfig.arg = static_cast<void*>(this);
    tmrConfig.dispatch_method = ESP_TIMER_TASK;
    tmrConfig.name = "ibtn_t1";
    tmrConfig.skip_unhandled_events = true;
    esp_timer_create(&tmrConfig, &m_DebounceTimer);

    // create click timer
    //esp_timer_create_args_t tmrConfig;
    tmrConfig.callback = [](void* self) { static_cast<InterruptButton*>(self)->timers_handler(timer_event_t::click); };
    tmrConfig.arg = static_cast<void*>(this);
    tmrConfig.dispatch_method = ESP_TIMER_TASK;
    tmrConfig.name = "ibtn_t2";
    tmrConfig.skip_unhandled_events = true;
    esp_timer_create(&tmrConfig, &m_ClickTimer);

    // create click timer
    //esp_timer_create_args_t tmrConfig;
    tmrConfig.callback = [](void* self) { static_cast<InterruptButton*>(self)->timers_handler(timer_event_t::longpress); };
    tmrConfig.arg = static_cast<void*>(this);
    tmrConfig.dispatch_method = ESP_TIMER_TASK;
    tmrConfig.name = "ibtn_t3";
    tmrConfig.skip_unhandled_events = true;
    esp_timer_create(&tmrConfig, &m_LongPressTimer);

    gpio_config_t gpio_conf = {};
    gpio_conf.mode = m_pinMode;
    gpio_conf.pin_bit_mask = BIT64(m_pin);
    gpio_conf.pull_down_en = (m_pressedState) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en =   (m_pressedState) ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
    gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&gpio_conf);                                                  //configure GPIO with the given settings

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);            // it' OK to call this function multiple times
    ESP_LOGD(TAG, "GPIO ISR service installed with exit status: %d", err);

    gpio_isr_handler_add(m_pin, InterruptButton::isr_handler, static_cast<void*>(this));

    m_state = (gpio_get_level(m_pin) == m_pressedState) ? pinState_t::PressOnHold : pinState_t::Released;    // Set to current state when initialising
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
void InterruptButton::bind(event_t event,btn_callback_t action, uint8_t menuLevel){
  btnaction_t a = { {m_pin, menuLevel, event}, action};
  btn_actions.add(a);
  if (event == event_t::MultiClick ){
    enableEvent(event_t::MultiClick);
  }

  if (event == event_t::AutoRepeatPress){
    enableEvent(event_t::AutoRepeatPress);
  }
}

void InterruptButton::unbind(event_t event, uint8_t menuLevel){
  if (!btn_actions.size())    // action list is empty
    return;

  for (int i = 0; i != btn_actions.size(); ++i){
    if (btn_actions.get(i).t.gpio == m_pin && btn_actions.get(i).t.event == event && btn_actions.get(i).t.menulvl == menuLevel){
      btn_actions.remove(i);
      return;
    }
  }
  // TODO: unbind can't simply clear 'doubleclk_actions' or 'repeat_actions' flags
}

/*
void InterruptButton::action(event_t event){
  action(event, m_menuLevel);
}
void InterruptButton::action(event_t event, uint8_t menuLevel){
  if(menuLevel >= m_numMenus)                                                 return;   // Invalid menu level
  if(!eventEnabled(event))                                                    return;   // Specific event is disabled
  if(eventActions[menuLevel][static_cast<uint8_t>(event)] == nullptr)                               return;   // Event is not defined
  if(event >= event_t::KeyDown && event <= event_t::DoubleClick && !eventEnabled(event_t::AsyncEvents))  return;   // This is an async event and they are disabled
  if(event >= event_t::SyncKeyPress && event <= event_t::SyncDoubleClick && !eventEnabled(event_t::SyncEvents))  return;   // This is a Sync event and they are disabled
  
  eventActions[menuLevel][static_cast<uint8_t>(event)]();
}
*/
void InterruptButton::enableEvent(event_t event){
  if(event < event_t::NumEventTypes) eventMask |= (1UL << static_cast<uint8_t>(event));    // Set the relevant bit
}

void InterruptButton::disableEvent(event_t event){
  if(event < event_t::NumEventTypes) eventMask &= ~(1UL << static_cast<uint8_t>(event));   // Clear the relevant bit
}

bool InterruptButton::eventEnabled(event_t event) {
  return ((eventMask >> static_cast<uint8_t>(event)) & 0x01) == 0x01;
}

void InterruptButton::setMenuLevel(uint8_t level) {
    m_menuLevel = level;
}

uint8_t   InterruptButton::getMenuLevel(){
  return m_menuLevel;
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
