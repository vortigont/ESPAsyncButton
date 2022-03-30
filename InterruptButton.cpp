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


//-- Method to monitor button, called by button change and various timer interrupts ----------------------
void IRAM_ATTR InterruptButton::readButton(void *arg){
  if (!q_action)    // quit if no Q
    return;

  InterruptButton* btn = static_cast<InterruptButton*>(arg);

  BaseType_t xCntxSwitch = pdFALSE;
  btntrigger_t t = { btn->m_pin, btn->m_menuLevel, event_t::KeyDown};

  switch(btn->m_state){
    case pinState_t::Released:                                              // Was sitting released but just detected a signal from the button
      gpio_intr_disable(btn->m_pin);                            // Ignore change inputs while we poll for a valid press
      btn->m_validPolls = 1; btn->m_totalPolls = 1;             // Was released, just detected a change, must be a valid press so count it.
      btn->m_longPress_preventKeyPress = false;
      startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &InterruptButton::readButton, btn, "DB_begin_");  // Begin debouncing the button input
      btn->m_state = pinState_t::ConfirmingPress;
      break;

    case pinState_t::ConfirmingPress:                                       // we get here each time the debounce timer expires (onchange interrupt disabled remember)
      btn->m_totalPolls++;                                                    // Count the number of total reads
      if(gpio_get_level(btn->m_pin) == btn->m_pressedState) btn->m_validPolls++; // Count the number of valid 'PRESSED' reads
      if(btn->m_totalPolls >= m_targetPolls){                                 // If we have checked the button enough times, then make a decision on key state
        if(btn->m_validPolls * 2 <= btn->m_totalPolls) {                      // Then it was a false alarm
          btn->m_state = pinState_t::Released;                                        
          gpio_intr_enable(btn->m_pin);
          return;
        }                                                                     // Otherwise, spill over to "Pressing"
      } else {                                                                // Not yet enough polls to confirm state
        startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &InterruptButton::readButton, btn, "CP2_");  // Keep sampling pin state
        return;
      }
    // Planned spill through here if logic requires, ie keyDown confirmed.
    case pinState_t::Pressing:                                                            // VALID KEYDOWN, assumed pressed if it had valid polls more than half the time
      //btn->action(event_t::KeyDown);                                                   // Fire the asynchronous keyDown event
      t.event = event_t::KeyDown;
      xQueueSendToBackFromISR(q_action, &t, &xCntxSwitch);
      if(btn->m_stateDblClick == pinState_t::DblClickIdle){                               // If not waiting for a double click or timing out, Commence longKeyPress / Autopress timers
        startTimer(btn->m_buttonLPandRepeatTimer, uint64_t(btn->m_longKeyPressMS * 1000), &InterruptButton::longPressEvent, btn, "CP1_");
      }
      btn->m_state = pinState_t::Pressed;
      gpio_intr_enable(btn->m_pin);                                           // Begin monitoring pin again
      break;

    case pinState_t::Pressed:                                                             // Currently pressed until now, but there was a change on the pin
      gpio_intr_disable(btn->m_pin);                                          // Turn off this interrupt to ignore inputs while we wait to check if valid release
      startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &InterruptButton::readButton, btn, "PR_");  // Start timer and start polling the button to debounce it
      btn->m_validPolls = 1; btn->m_totalPolls = 1;                           // This is first poll and it was just released by definition of state
      btn->m_state = pinState_t::WaitingForRelease;
      break;

    case pinState_t::WaitingForRelease: // we get here when debounce timer or doubleclick timeout timer alarms (onchange interrupt disabled remember)
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
    // Intended spill through here to "Releasing" once keyUp confirmed.

    case pinState_t::Releasing:
      killTimer(btn->m_buttonLPandRepeatTimer);
      t.event = event_t::KeyUp;
      xQueueSendToBackFromISR(q_action, &t, &xCntxSwitch);

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
  } // End of SWITCH statement

  if( xCntxSwitch )               // perform a context switch if required
    portYIELD_FROM_ISR ();
} // End of readButton function


//-- Method to handle longKeyPresses (called by timer)----------------------------------------------------
void InterruptButton::longPressEvent(void *arg){
  if (!q_action)
    return;

  InterruptButton* btn = static_cast<InterruptButton*>(arg);

    btntrigger_t t = { btn->m_pin, btn->m_menuLevel, event_t::LongKeyPress};
    btn->m_longPress_preventKeyPress = true;                                          // Used to prevent regular keypress later on in procedure.
    
    //Initiate the autorepeat function
    if(gpio_get_level(btn->m_pin) == btn->m_pressedState) {                           // Sanity check to stop autorepeats in case we somehow missed button release
      startTimer(btn->m_buttonLPandRepeatTimer, uint64_t(btn->m_autoRepeatMS * 1000), &InterruptButton::autoRepeatPressEvent, btn, "LPD_");
    }

    xQueueSendToBack(q_action, (void*)&t, (TickType_t)0);
}

//-- Method to handle autoRepeatPresses (called by timer)-------------------------------------------------
void InterruptButton::autoRepeatPressEvent(void *arg){
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

//-- Method to return to interpret previous keyUp as a keyPress instead of a doubleClick if it times out.
void InterruptButton::doubleClickTimeout(void *arg){
  if (!q_action)
    return;

  InterruptButton* btn = reinterpret_cast<InterruptButton*>(arg);
  btntrigger_t t = { btn->m_pin, btn->m_menuLevel, event_t::KeyPress};
  xQueueSendToBack(q_action, (void*)&t, (TickType_t)0);
  btn->m_stateDblClick = pinState_t::DblClickIdle;
}

//-- Helper method to simplify starting a timer ----------------------------------------------------------
void IRAM_ATTR InterruptButton::startTimer(esp_timer_handle_t &timer, uint32_t duration_US, void (*callBack)(void* arg), InterruptButton* btn, const char *msg){
  esp_timer_create_args_t tmrConfig;
  tmrConfig.arg = static_cast<void*>(btn);
  tmrConfig.callback = callBack;
  tmrConfig.dispatch_method = ESP_TIMER_TASK;
  tmrConfig.name = msg;
  killTimer(timer);
  esp_timer_create(&tmrConfig, &timer);
  esp_timer_start_once(timer, duration_US);
}

//-- Helper method to kill a timer -----------------------------------------------------------------------
void IRAM_ATTR InterruptButton::killTimer(esp_timer_handle_t &timer){
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
                                 m_doubleClickMS(doubleClickMS) {

  // gpio number sanity check
  if (GPIO_IS_VALID_GPIO(pin))
    m_pin = static_cast<gpio_num_t>(pin);
  else {
    ESP_LOGW(TAG, "%d is not valid gpio on this platform", pin);
    m_pin = static_cast<gpio_num_t>(-1);    //GPIO_NUM_NC (enum not showing up as defined);
  }

  m_pollIntervalUS = (debounceUS / m_targetPolls > 65535) ? 65535 : debounceUS / m_targetPolls;
}

// Destructor --------------------------------------------------------------------
InterruptButton::~InterruptButton() {
  gpio_isr_handler_remove(m_pin);
  killTimer(m_buttonPollTimer); killTimer(m_buttonLPandRepeatTimer); killTimer(m_buttonDoubleClickTimer);
  
  unbindall();
  gpio_reset_pin(m_pin);
}

// Initialiser -------------------------------------------------------------------
void InterruptButton::begin(void){
    startBtnTask();             // enshure we have a Q to work on and event consumer Task

    gpio_config_t gpio_conf = {};
      gpio_conf.mode = m_pinMode;
      gpio_conf.pin_bit_mask = BIT64(m_pin);
      gpio_conf.pull_down_en = (m_pressedState) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
      gpio_conf.pull_up_en =   (m_pressedState) ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
      gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
      gpio_config(&gpio_conf);                                                  //configure GPIO with the given settings

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);            // it' OK to call this function multiple times
    ESP_LOGD(TAG, "GPIO ISR service installed with exit status: %d", err);

    gpio_isr_handler_add(m_pin, InterruptButton::readButton, static_cast<void*>(this));

    m_state = (gpio_get_level(m_pin) == m_pressedState) ? pinState_t::Pressed : pinState_t::Released;    // Set to current state when initialising
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
  if (event == event_t::DoubleClick ){
    enableEvent(event_t::DoubleClick);
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
/*
void InterruptButton::setMenuCount(uint8_t numberOfMenus){           // This can only be set before initialising first button
  if(!m_firstButtonInitialised && numberOfMenus > 1) m_numMenus = numberOfMenus;
}
uint8_t InterruptButton::getMenuCount(void) {
  return m_numMenus;
}
*/

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
