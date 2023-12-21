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

namespace ESPButton {

  // Events that button generate on operation
  enum class event_t:uint8_t {
    none = 0,               // nothig
    press,                  // button was just pressed, it is not yet know how long it will reside in that "pressed" state
    release,                // button was just released after shot press
    click,                  // short press+release event, i.e. a "mouse click"
    longPress,              // button was pressed and was held in a pressed state long enough that it can't be treaded as 'click'
    longRelease,            // button was releases after being long-pressed for quite some time
    autoRepeat,             // button is being held down, autorepeat events are generated periodically while button is in pressed state
    multiClick,             // button has been clicked many times consecutevily (a number of clicks is also reported within the event)
    anyClick                // any of 'click', 'autoRepeat', 'longPress', multiClick
  };

  /**
   * @brief button trigger event structure
   * gpio - gpio num triggered the event
   * menulvl - button's menulevel at the time of event
   * event - event type
   * param - optional parameter (default is 0)
   */
  struct btntrigger_t {
    gpio_num_t gpio;
    uint8_t menulvl;
    ESPButton::event_t event;
    int32_t param;

    bool operator== (const btntrigger_t& b) const { return (gpio == b.gpio && menulvl == b.menulvl && event == b.event); };
  };

  struct btnaction_t {
    btntrigger_t t;
    btn_callback_t cb;
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
} //namespace ESPButton


// -- Interrupt Button and Debouncer ---------------------------------------------------------------------------------------
// -- ----------------------------------------------------------------------------------------------------------------------
class InterruptButton {
  enum class timer_event_t:uint8_t {
    debounce = 0,
    click,
    longpress
  };

  // momentary button states
  enum class btnState_t:uint8_t {
    undefined=0,
    idle,                 // key is idle
    pressDebounce,            // key press triggered, waiting for debounce confirmation
    pressed,                  // key press debounced and confirmed
    onHold,                   // key is held in a pressed state for time < LongPress
    onLongHold,               // key is held in a pressed state for time > LongPress
    releaseDebounce,          // received release interrput, but need to debounce and confirm
    releaseLongDebounce,      // received release interrput after a LongHold, but need to debounce and confirm
    released,                 // key has just been released and debounced, but pending event processing
    releasedLong              // key has just been released and debounced from a long-hold, but pending event processing
  };

  struct btnmenu_t {
    uint8_t level{0};
    std::bitset<IBTN_MAX_MENU_DEPTH> click{0x0};                       // bit vector for tracking clicks at specific menulevel
    std::bitset<IBTN_MAX_MENU_DEPTH> longpress{0x0};                   // bit vector for tracking longpress at specific menulevel
    std::bitset<IBTN_MAX_MENU_DEPTH> repeat{0x0};                      // bit vector for tracking repeats at specific menulevel

    void eventSet(ESPButton::event_t evt, bool state, uint8_t menulvl);
    bool eventGet(ESPButton::event_t evt, uint8_t menulvl) const;
    inline void eventSet(ESPButton::event_t evt, bool state){ eventSet(evt, state, level); }
    inline bool eventGet(ESPButton::event_t evt) const { return eventGet(evt, level); }

  };

  struct Counters
  {
    int8_t debounce{0};     // debounce counter
    int8_t click{0};        // click counter
    int8_t repeat{0};       // repeat counter
  };

  struct Times
  {
    uint16_t debounce;            // gpio debounce timeout, us
    uint16_t longPress;           // how long to hold a button to consider it as a longPress, ms
    uint16_t autoRepeat;          // autorepeat interval, ms
    uint16_t multiClick;          // time between consecutive keypress when it's considered as multiple clicks, ms
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

    /**
     * @brief remove all action bindings
     * 
     */
    void unbindall();


    // event counters
    Counters _ctr;
    // timeouts and threshold times
    Times _t;

    btnmenu_t m_menu;                                                       // feature set

    volatile btnState_t m_state = btnState_t::undefined;

    esp_timer_handle_t m_DebounceTimer = nullptr;                           // Instance specific timer for button debouncing
    esp_timer_handle_t m_LongPressTimer = nullptr;                          // Instance specific timer for button longPress and autoRepeat timing
    esp_timer_handle_t m_ClickTimer = nullptr;                              // timer for counting consecutive clicks

    bool _logicLevel;                                                       // Logic Level of a button when it is pressed (LOW or HIGH)
    gpio_num_t m_pin;                                                       // Button gpio
    gpio_mode_t m_pinMode;                                                  // GPIO mode: IDF's input/output mode


    public:
    // Class Constructor
    InterruptButton(uint8_t pin, bool logicLevel,
                    gpio_mode_t pinMode = GPIO_MODE_INPUT);
    ~InterruptButton();                                               // Class Destructor


    /**
     * @brief configure gpio and attach interrupt monitor
     */
    void enable();

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
    inline void setEventState(ESPButton::event_t event, uint8_t menulvl, bool enable){ m_menu.eventSet( event, menulvl, enable); };

    /**
     * @brief Get event type state at specific menulevel
     * 
     * @param event - event type
     * @param menulvl - menu level
     */
    inline bool getEventState(ESPButton::event_t event, uint8_t menulvl) const { return m_menu.eventGet( event, menulvl); };


    void      setLongPressInterval(uint16_t intervalMS);              // Updates LongPress Interval
    inline uint16_t  getLongPressInterval(void) const { return _t.longPress; };
    void      setAutoRepeatInterval(uint16_t intervalMS);             // Updates autoRepeat Interval
    inline uint16_t  getAutoRepeatInterval(void) const { return _t.autoRepeat; };
    void      setMultiClickInterval(uint16_t intervalMS);            // Updates autoRepeat Interval
    inline uint16_t  getMultiClickInterval(void) const { return _t.multiClick; };


    void bind(  ESPButton::event_t event, btn_callback_t action, uint8_t menuLevel = 0);   // Used to bind/unbind action to an event at specified menu level
    void unbind(ESPButton::event_t event, uint8_t menuLevel = 0);

    /**
     * @brief return current Button logical state
     * 
     * @return btnState_t 
     */
    btnState_t getState() const { return m_state; };

};

