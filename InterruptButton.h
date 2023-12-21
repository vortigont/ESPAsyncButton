#pragma once

#include "driver/gpio.h"
#include "esp_timer.h"
#include <functional>     // Necessary to use std::bind to bind arguments to ISR in attachInterrupt()
#include <bitset>

#define IBTN_LONG_PRESS_TIME_MS     750
#define IBTN_AUTO_REPEAT_TIME_MS    250
#define IBTN_DOUBLE_CLICK_TIME_MS   350
#define IBTN_DEBOUNCE_TIME_US       5000      // polling interval for debouncer
#define IBTN_DEBOUNCE_CNT           5         // number of consecutive readouts with the same value for the pin to be considered as debounced
#ifndef IBTN_MAX_MENU_DEPTH
#define IBTN_MAX_MENU_DEPTH         8         // max menu level depth (flags bitvector length)
#endif

typedef std::function<void ()> btn_callback_t;

enum class event_t:uint8_t {
  none = 0,
  press,
  release,
  click,
  longClick,
  autoRepeat,
  multiClick,
  anyClick                 // any of 'click', 'repeat', 'longClick'
};

// momentary button states
enum class btnState_t:uint8_t {                     // Enumeration to assist with program flow at state machine for reading button
  undefined=0,
  released,                 // key is idle
  pressDebounce,            // key press triggered, waiting for debounce confirmation
  pressed,                // key press debounced and confirmed
  onHold,              // key is held in a pressed state for time < LongPress
  onLongHold,          // key is held in a pressed state for time > LongPress
  releaseDebounce,          // received release interrput, but need to debounce and confirm
  releaseLongDebounce,      // received release interrput after a LongHold, but need to debounce and confirm
  pressRelease,             // key has been released and debounced
  pressLongRelease
};

enum class timer_event_t:uint8_t {
  debounce = 0,
  click,
  longpress
};


/**
 * @brief button trigger event structure
 * gpio - gpio num triggered the event
 * menulvl - button's menulevel at the time of event
 * event - event type
 * param - optional parameter (default is 0)
 * NOTE: this struct is 32 bit aligned
 */
struct btntrigger_t {
  gpio_num_t gpio;
  uint8_t menulvl;
  event_t event;
  int16_t param;

  bool operator== (const btntrigger_t& b) const { return (gpio == b.gpio && menulvl == b.menulvl && event == b.event); };
};

struct btnaction_t {
  btntrigger_t t;
  btn_callback_t cb;
};


struct btnmenu_t {
  uint8_t level{0};
  std::bitset<IBTN_MAX_MENU_DEPTH> click{0x0};                       // bit vector for tracking clicks at specific menulevel
  std::bitset<IBTN_MAX_MENU_DEPTH> longpress{0x0};                   // bit vector for tracking longpress at specific menulevel
  std::bitset<IBTN_MAX_MENU_DEPTH> repeat{0x0};                      // bit vector for tracking repeats at specific menulevel

  void eventSet(event_t evt, bool state, uint8_t menulvl);
  bool eventGet(event_t evt, uint8_t menulvl) const;
  inline void eventSet(event_t evt, bool state){ eventSet(evt, state, level); }
  inline bool eventGet(event_t evt) const { return eventGet(evt, level); }

};

/**
 * @brief run Button event queue task
 * 
 * @return true on success
 * @return false of error
 */
bool startBtnTask();

/**
 * @brief stop Button event queue task
 * 
 */
void stopBtnTask();

// -- Interrupt Button and Debouncer ---------------------------------------------------------------------------------------
// -- ----------------------------------------------------------------------------------------------------------------------
class InterruptButton {

    struct Counters
    {
      int8_t debounce{0};     // debounce counter
      int8_t click{0};        // click counter
      int8_t repeat{0};       // repeat counter
    };
  

    /**
     * @brief GPIO ISR handler wrapper
     * 
     * @param arg - an object instance reference
     */
    static void isr_handler(void* arg);

    /**
     * @brief process the ISR for the object instance
     * 
     */
    void gpio_update_from_isr();

    void readButton();

    /**
     * @brief LongPress and Autorepeat handler
     * called via m_LongPressTimer and reuses the same timer to perform 'autorepeat on hold'
     * 
     */
    void longPressTimeout();                                // timer callback to excecute a longPress event

    void clickTimeout();                                    // Used to separate double-clicks from regular keyPress's

    /**
     * @brief helper method to create esp soft timer
     * 
     * @param timer
     * @param tevent
     */
    void createTimer(timer_event_t tevent, esp_timer_handle_t &timer);

    // Helper function to kill a timer
    void killTimer(esp_timer_handle_t &timer);

    btnmenu_t m_menu;                                                       // feature set

    volatile btnState_t m_state = btnState_t::undefined;

    esp_timer_handle_t m_DebounceTimer = nullptr;                           // Instance specific timer for button debouncing
    esp_timer_handle_t m_LongPressTimer = nullptr;                          // Instance specific timer for button longPress and autoRepeat timing
    esp_timer_handle_t m_ClickTimer = nullptr;                              // timer for counting consecutive clicks

    uint8_t m_pressedState;                                                 // State of button when it is pressed (LOW or HIGH)
    gpio_num_t m_pin;                                                       // Button gpio
    gpio_mode_t m_pinMode;                                                  // GPIO mode: IDF's input/output mode

    uint16_t m_gpio_debounceUS;
    uint16_t m_longKeyPressMS;
    uint16_t m_autoRepeatMS;
    uint16_t m_MultiClickMS;

    Counters _ctr;

    /**
     * @brief remove all action bindings
     * 
     */
    void unbindall();


    public:
    // Class Constructor
    InterruptButton(uint8_t pin, uint8_t pressedState,
                    gpio_mode_t pinMode = GPIO_MODE_INPUT,
                    uint16_t longKeyPressMS = IBTN_LONG_PRESS_TIME_MS,
                    uint16_t autoRepeatMS = IBTN_AUTO_REPEAT_TIME_MS,
                    uint16_t MultiClickMS = IBTN_DOUBLE_CLICK_TIME_MS,
                    uint32_t debounceUS = IBTN_DEBOUNCE_TIME_US);
    ~InterruptButton();                                               // Class Destructor


    /**
     * @brief configure gpio and attach interrupt monitor
     */
    void enable();                                                     // Instance initialiser

    /**
     * @brief disable gpio interrupt handling
     * deactivates button
     */
    void disable();

    /**
     * @brief switch current menulevel for the button
     * changing menulevel affects handling of LongPress/multiClick events, etc...
     * 
     * @param level
     */
    void setMenuLevel(uint8_t level);

    inline uint8_t getMenuLevel() const { return m_menu.level; }                 // Retrieves menu level

    /**
     * @brief Enable/Disable particalar event type at specific menulevel
     * 
     * @param event - event type
     * @param menulvl - menu level
     * @param enable - enable/disable state
     */
    inline void setEventState(event_t event, uint8_t menulvl, bool enable){ m_menu.eventSet( event, menulvl, enable); };

    /**
     * @brief Get event type state at specific menulevel
     * 
     * @param event - event type
     * @param menulvl - menu level
     */
    inline bool getEventState(event_t event, uint8_t menulvl) const { return m_menu.eventGet( event, menulvl); };


    void      setLongPressInterval(uint16_t intervalMS);              // Updates LongPress Interval
    inline uint16_t  getLongPressInterval(void) const { return m_longKeyPressMS; };
    void      setAutoRepeatInterval(uint16_t intervalMS);             // Updates autoRepeat Interval
    inline uint16_t  getAutoRepeatInterval(void) const { return m_autoRepeatMS; };
    void      setMultiClickInterval(uint16_t intervalMS);            // Updates autoRepeat Interval
    inline uint16_t  getMultiClickInterval(void) const { return m_MultiClickMS; };


    void bind(  event_t event, btn_callback_t action, uint8_t menuLevel = 0);   // Used to bind/unbind action to an event at specified menu level
    void unbind(event_t event, uint8_t menuLevel = 0);

    /**
     * @brief return current Button logical state
     * 
     * @return btnState_t 
     */
    btnState_t getState() const { return m_state; };

};

