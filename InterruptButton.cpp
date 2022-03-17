#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "InterruptButton.h"
#include "LList.h"

#ifdef ARDUINO
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
#endif

#define ESP_INTR_FLAG_DEFAULT 0
#define Q_DEPTH               5
#define EVT_TASK_PRIO         2             // a bit higher than arduino's loop()
#define EVT_TASK_STACK        4096
#define EVT_TASK_NAME         "BTN_ACTN"

static const char* TAG = "IBTN";    // IDF log tag

/* ToDo
  Need to confirm if any ISR's need to blocked/disabled from other ISR entry, ie portMUX highlevel/lowlevel, etc.
  Confirm all necessary varibables are volatile (noting timers throw an error when defined as volatile)

Consider Adding button modes such as momentary, latching, etc.
Consider a static member queue common across all instances to maintain order of all sync events across all buttons (and simplify processing the event actions in the main loop)
Consider an RTOS queue.
Consider adding chord combinations (2 or more buttons pressed conccurently) as a event.  Added to static class member, when any one button is pushed.
corresponding buttons are checked to see if they are in waiting for release state (will be a factorial type check to minimise redundant checks)
considered vortigont's note on thread safety:
    "Thinking forward to thread safety, it might be even better to keep those methods not as static class members
    but private classless functions (or namespace members). Passing pointers to dynamically created class instances
    always has a tiny chance to hit the dangling pointer :))) Although it's not that critical in current implementation."
*/

QueueHandle_t   q_action = nullptr;
TaskHandle_t    t_action = nullptr;
LList<btnaction_t> btn_actions;

QueueHandle_t* getActionQ(){
  if (q_action)
    return &q_action;

  q_action = xQueueCreate( Q_DEPTH, sizeof(btntrigger_t) ); // make q for button actions

  return &q_action;
}

void actionsTask(void* pvParams){
  btntrigger_t t;

  for (;;){
    if (xQueueReceive(q_action, (void*)&t, (portTickType)portMAX_DELAY)) {
      for (auto i = btn_actions.cbegin(); i != btn_actions.cend(); ++i){    // search for registered callback function
        if (i->t == t){                                                     // execute callback if btntrigger_t matches the one provided via Q
          i->cb();
          break;
        }
      }
    }
  }
  vTaskDelete(NULL);
}


bool startBtnActions(){
  if (!t_action && getActionQ())
      return xTaskCreate(actionsTask, EVT_TASK_NAME, EVT_TASK_STACK, NULL, EVT_TASK_PRIO, &t_action) == pdPASS;
  else
      return true;  
}


//-- STATIC CLASS MEMBERS AND METHODS (COMMON ACROSS ALL INSTANCES TO SAVE MEMORY) -----------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------

//-- Initialise Static Member Variables ------------------------------------------------------------------
//bool    InterruptButton::m_firstButtonInitialised { false };
//uint8_t InterruptButton::m_numMenus               { 1 };
//uint8_t InterruptButton::m_menuLevel              { 0 };

//-- Method to monitor button, called by button change and various timer interrupts ----------------------
void IRAM_ATTR InterruptButton::readButton(void *arg){
  InterruptButton* btn = reinterpret_cast<InterruptButton*>(arg);

  BaseType_t xCntxSwitch = pdFALSE;
  btntrigger_t t = { btn->m_pin, btn->m_menuLevel, event_t::KeyUp};

  switch(btn->m_state){
    case Released:                                              // Was sitting released but just detected a signal from the button
      gpio_intr_disable(btn->m_pin);                            // Ignore change inputs while we poll for a valid press
      btn->m_validPolls = 1; btn->m_totalPolls = 1;             // Was released, just detected a change, must be a valid press so count it.
      btn->m_longPress_preventKeyPress = false;
      startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &readButton, btn, "DB_begin_");  // Begin debouncing the button input
      btn->m_state = ConfirmingPress;
      break;

    case ConfirmingPress:                                       // we get here each time the debounce timer expires (onchange interrupt disabled remember)
      btn->m_totalPolls++;                                                    // Count the number of total reads
      if(gpio_get_level(btn->m_pin) == btn->m_pressedState) btn->m_validPolls++; // Count the number of valid 'PRESSED' reads
      if(btn->m_totalPolls >= m_targetPolls){                                 // If we have checked the button enough times, then make a decision on key state
        if(btn->m_validPolls * 2 <= btn->m_totalPolls) {                      // Then it was a false alarm
          btn->m_state = Released;                                        
          gpio_intr_enable(btn->m_pin);
          return;
        }                                                                     // Otherwise, spill over to "Pressing"
      } else {                                                                // Not yet enough polls to confirm state
        startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &readButton, btn, "CP2_");  // Keep sampling pin state
        return;
      }
    // Planned spill through here if logic requires, ie keyDown confirmed.
    case Pressing:                                                            // VALID KEYDOWN, assumed pressed if it had valid polls more than half the time
      //btn->action(event_t::KeyDown);                                                   // Fire the asynchronous keyDown event
      t.event = event_t::KeyDown;
      xQueueSendToBackFromISR(*getActionQ(), &t, &xCntxSwitch);
      if(btn->m_stateDblClick == DblClickIdle){                               // If not waiting for a double click or timing out, Commence longKeyPress / Autopress timers
        //btn->m_longKeyPressMenuLevel = m_menuLevel;                           // (will be cancelled if released before timer expires)
        startTimer(btn->m_buttonLPandRepeatTimer, uint64_t(btn->m_longKeyPressMS * 1000), &longPressEvent, btn, "CP1_");
      }
      btn->m_state = Pressed;
      gpio_intr_enable(btn->m_pin);                                           // Begin monitoring pin again
      break;

    case Pressed:                                                             // Currently pressed until now, but there was a change on the pin
      gpio_intr_disable(btn->m_pin);                                          // Turn off this interrupt to ignore inputs while we wait to check if valid release
      startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &readButton, btn, "PR_");  // Start timer and start polling the button to debounce it
      btn->m_validPolls = 1; btn->m_totalPolls = 1;                           // This is first poll and it was just released by definition of state
      btn->m_state = WaitingForRelease;
      break;

    case WaitingForRelease: // we get here when debounce timer or doubleclick timeout timer alarms (onchange interrupt disabled remember)
                            // stay in this state until released, because button could remain locked down if release missed.
      btn->m_totalPolls++;
      if(gpio_get_level(btn->m_pin) != btn->m_pressedState){
        btn->m_validPolls++;
        if(btn->m_totalPolls < m_targetPolls || btn->m_validPolls * 2 <= btn->m_totalPolls) {           // If we haven't polled enough or not high enough success rate
          startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &readButton, btn, "W4R_polling_");  // Then keep sampling pin state until release is confirmed
          return;
        }                                                                                 // Otherwise, spill through to "Releasing"
      } else {
        if(btn->m_validPolls > 0) {
          btn->m_validPolls--;
        } else {
          btn->m_totalPolls = 0;                                                          // Key is being held down, don't let total polls get too far ahead.
        }
        startTimer(btn->m_buttonPollTimer, btn->m_pollIntervalUS, &readButton, btn, "W4R_invalidPoll"); // Keep sampling pin state until released
      }
    // Intended spill through here to "Releasing" once keyUp confirmed.

    case Releasing:
      killTimer(btn->m_buttonLPandRepeatTimer);
      //btn->action(event_t::KeyUp);
      t.event = event_t::KeyUp;
      xQueueSendToBackFromISR(*getActionQ(), &t, &xCntxSwitch);

      // If double-clicks are enabled and either defined
      if(btn->eventEnabled(event_t::DoubleClick) || btn->eventEnabled(event_t::SyncDoubleClick)) {
         //(btn->eventEnabled(event_t::SyncDoubleClick) && btn->eventEnabled(event_t::SyncEvents) && btn->eventActions[m_menuLevel][static_cast<uint8_t>(event_t::SyncDoubleClick)] != nullptr)) {

        if(btn->m_stateDblClick == DblClickIdle && !btn->m_longPress_preventKeyPress) {   // Commence detection process
          btn->m_stateDblClick = DblClickWaiting;
          //btn->m_keyPressMenuLevel = m_menuLevel;                                         // Save menuLevel in case this is converted to a keyPress later
          startTimer(btn->m_buttonDoubleClickTimer, uint64_t(btn->m_doubleClickMS * 1000), &doubleClickTimeout, btn, "W4R_DCsetup_");

        } else if (btn->m_stateDblClick == DblClickWaiting) {                             // VALID DOUBLE-CLICK (second keyup without a timeout)
          killTimer(btn->m_buttonDoubleClickTimer);
          btn->m_stateDblClick = DblClickIdle;
          //btn->m_doubleClickMenuLevel = m_menuLevel;
          //btn->action(event_t::DoubleClick); btn->doubleClickOccurred = true;
          t.event = event_t::DoubleClick;
          xQueueSendToBackFromISR(*getActionQ(), &t, &xCntxSwitch);
        }
      } else if(!btn->m_longPress_preventKeyPress) {                                      // Otherwise, treat as a basic keyPress
        //btn->action(event_t::KeyPress);                                                   // Then treat as a normal keyPress
        //btn->keyPressOccurred = true; btn->m_keyPressMenuLevel = m_menuLevel;
        t.event = event_t::KeyPress;
        xQueueSendToBackFromISR(*getActionQ(), &t, &xCntxSwitch);
      } 
      btn->m_state = Released;
      gpio_intr_enable(btn->m_pin);
      break;
  } // End of SWITCH statement

  if( xCntxSwitch )               // perform a context switch if required
    portYIELD_FROM_ISR ();
} // End of readButton function


//-- Method to handle longKeyPresses (called by timer)----------------------------------------------------
void InterruptButton::longPressEvent(void *arg){
  InterruptButton* btn = reinterpret_cast<InterruptButton*>(arg);

    btntrigger_t t = { btn->m_pin, btn->m_menuLevel, event_t::LongKeyPress};
    //btn->action(event_t::LongKeyPress);                                                        // Action the Async LongKeyPress Event
    //btn->longKeyPressOccurred = true;                                                 // Setup for Sync LongKeyPress Event
    btn->m_longPress_preventKeyPress = true;                                          // Used to prevent regular keypress later on in procedure.
    
    //Initiate the autorepeat function
    if(gpio_get_level(btn->m_pin) == btn->m_pressedState) {                           // Sanity check to stop autorepeats in case we somehow missed button release
      startTimer(btn->m_buttonLPandRepeatTimer, uint64_t(btn->m_autoRepeatMS * 1000), &autoRepeatPressEvent, btn, "LPD_");
    }

    xQueueSendToBack(*getActionQ(), (void*)&t, (TickType_t)0);
}

//-- Method to handle autoRepeatPresses (called by timer)-------------------------------------------------
void InterruptButton::autoRepeatPressEvent(void *arg){
  InterruptButton* btn = reinterpret_cast<InterruptButton*>(arg);

  btntrigger_t t = { btn->m_pin, btn->m_menuLevel, event_t::KeyPress};

  if(btn->eventEnabled(event_t::DoubleClick)) {
    //btn->action(event_t::AutoRepeatPress);                                                     // Action the Async Auto Repeat KeyPress Event if defined
    t.event = event_t::AutoRepeatPress;
  }

  //btn->autoRepeatPressOccurred = true; btn->m_autoRepeatPressMenuLevel = m_menuLevel; // Setup for Sync AutoRepeatePress Event
  if(gpio_get_level(btn->m_pin) == btn->m_pressedState) {                             // Sanity check to stop autorepeats in case we somehow missed button release
    startTimer(btn->m_buttonLPandRepeatTimer, uint64_t(btn->m_autoRepeatMS * 1000), &autoRepeatPressEvent, btn, "ARPE_");
  }
  xQueueSendToBack(*getActionQ(), (void*)&t, (TickType_t)0);
}

//-- Method to return to interpret previous keyUp as a keyPress instead of a doubleClick if it times out.
void InterruptButton::doubleClickTimeout(void *arg){
  InterruptButton* btn = reinterpret_cast<InterruptButton*>(arg);
  btntrigger_t t = { btn->m_pin, btn->m_menuLevel, event_t::KeyPress};
  xQueueSendToBack(*getActionQ(), (void*)&t, (TickType_t)0);
  //btn->action(event_t::KeyPress);                                                  // Then treat as a normal keyPress
  //btn->keyPressOccurred = true; btn->m_keyPressMenuLevel = m_menuLevel;   // Note, this timer is never started if previous press was a longpress
  btn->m_stateDblClick = DblClickIdle;
}

//-- Helper method to simplify starting a timer ----------------------------------------------------------
void IRAM_ATTR InterruptButton::startTimer(esp_timer_handle_t &timer, uint32_t duration_US, void (*callBack)(void* arg), InterruptButton* btn, const char *msg){
  esp_timer_create_args_t tmrConfig;
    tmrConfig.arg = reinterpret_cast<void*>(btn);
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
    //if(!m_firstButtonInitialised) m_firstButtonInitialised = true;
/*
    eventActions = new func_ptr*[m_numMenus];
    for(int menu = 0; menu < m_numMenus; menu++){
      eventActions[menu] = new func_ptr[event_t::NumEventTypes];
      for(int evt = 0; evt < static_cast<uint8_t>(event_t::NumEventTypes); evt++){
        eventActions[menu][evt] = nullptr;
      }
    }
*/
    gpio_config_t gpio_conf = {};
      gpio_conf.mode = m_pinMode;
      gpio_conf.pin_bit_mask = BIT64(m_pin);
      gpio_conf.pull_down_en = (m_pressedState) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
      gpio_conf.pull_up_en =   (m_pressedState) ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
      gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
      gpio_config(&gpio_conf);                                                  //configure GPIO with the given settings

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);            // it' OK to call this function multiple times
    ESP_LOGD(TAG, "GPIO ISR service installed with exit status: %d", err);

    gpio_isr_handler_add(m_pin, InterruptButton::readButton, (void*)this);

    m_state = (gpio_get_level(m_pin) == m_pressedState) ? Pressed : Released;    // Set to current state when initialising
}


//-- TIMING INTERVAL GETTERS AND SETTERS -----------------------------------------------------------------
void      InterruptButton::setLongPressInterval(uint16_t intervalMS)    { m_longKeyPressMS = intervalMS; }
uint16_t  InterruptButton::getLongPressInterval(void)                   { return m_longKeyPressMS;       }
void      InterruptButton::setAutoRepeatInterval(uint16_t intervalMS)   { m_autoRepeatMS = intervalMS;   }
uint16_t  InterruptButton::getAutoRepeatInterval(void)                  { return m_autoRepeatMS;         }
void      InterruptButton::setDoubleClickInterval(uint16_t intervalMS)  { m_doubleClickMS = intervalMS;  }
uint16_t  InterruptButton::getDoubleClickInterval(void)                 { return m_doubleClickMS;        }


// -- FUNCTIONS RELATED TO EXTERNAL ACTIONS --------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------

void InterruptButton::bind(event_t event, btn_callback_t action){
  bind(event, m_menuLevel, action);
}

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
void InterruptButton::bind(event_t event, uint8_t menuLevel, btn_callback_t action){
  btnaction_t a = { {m_pin, menuLevel, event}, action};
  btn_actions.add(a);
  if (event == event_t::DoubleClick || event == event_t::SyncDoubleClick){
    enableEvent(event_t::DoubleClick);
    enableEvent(event_t::SyncDoubleClick);
  }

  if (event == event_t::AutoRepeatPress || event == event_t::SyncAutoKeyPress){
    enableEvent(event_t::AutoRepeatPress);
    enableEvent(event_t::SyncAutoKeyPress);
  }
}

void InterruptButton::unbind(event_t event){
  unbind(event, m_menuLevel);
}
/*
void InterruptButton::unbind(event_t event, uint8_t menuLevel){
  if(!m_firstButtonInitialised){
    ESP_LOGE(TAG, "You must call the 'begin()' function prior to unbinding external fuctions from a button!");
  } else if(menuLevel >= m_numMenus) {
    ESP_LOGE(TAG, "Specified menu level is greater than the number of menus!");
  } else if(event >= event_t::NumEventTypes) {
    ESP_LOGE(TAG, "Specified event is invalid!");
  } else {
    eventActions[menuLevel][static_cast<uint8_t>(event)] = nullptr;
    if(eventEnabled(event)) disableEvent(event);
  }
  return;
}
*/
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
  //if(event >= event_t::SyncKeyPress && event <= event_t::SyncDoubleClick || event == event_t::SyncEvents) clearSyncInputs();
  if(event <= event_t::SyncEvents && event != event_t::NumEventTypes) eventMask |= (1UL << static_cast<uint8_t>(event));    // Set the relevant bit
}
void InterruptButton::disableEvent(event_t event){
  if(event <= event_t::SyncEvents && event != event_t::NumEventTypes) eventMask &= ~(1UL << static_cast<uint8_t>(event));   // Clear the relevant bit
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


//-- SYNCHRONOUS EVENT ROUTINES --------------------------------------------------------------------------
bool InterruptButton::inputOccurred(void){
  return (keyPressOccurred || longKeyPressOccurred || autoRepeatPressOccurred || doubleClickOccurred); 
}
/*
void InterruptButton::clearSyncInputs(void){
  keyPressOccurred = false; longKeyPressOccurred = false; autoRepeatPressOccurred = false; doubleClickOccurred = false;
}

void InterruptButton::processSyncEvents(void){
  if(eventEnabled(event_t::SyncEvents)){
    if(keyPressOccurred) {
      action(event_t::SyncKeyPress, m_keyPressMenuLevel);                            keyPressOccurred = false; 
    }        
    if(longKeyPressOccurred){
      action(event_t::SyncLongKeyPress, m_longKeyPressMenuLevel);                    longKeyPressOccurred = false; 
    }
    if(autoRepeatPressOccurred){
      event_t event = (eventActions[m_autoRepeatPressMenuLevel][static_cast<uint8_t>(event_t::SyncAutoKeyPress)]) ? event_t::SyncAutoKeyPress : event_t::SyncKeyPress;
      action(event, m_autoRepeatPressMenuLevel);                            autoRepeatPressOccurred = false;
    }
    if(doubleClickOccurred){
      action(event_t::SyncDoubleClick, m_doubleClickMenuLevel);                      doubleClickOccurred = false; 
    }
  }
}
*/

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
