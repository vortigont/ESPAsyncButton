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

#define IBTN_MAX_MENU_DEPTH         8         // max menu level depth (flags bitvector length)

typedef std::function<void ()> btn_callback_t;

enum class event_t:uint8_t {
  KeyDown = 0,          // These first 6 events are asycnchronous unless explicitly stated and actioned immediately in an ISR ...
  KeyUp,                // ... Asynchronous events should be defined with the IRAM_ATTR attributed so that ...
  KeyPress,             // ... Their code is stored in the RAM so as to not rely on SPI bus for timing to get from flash memory
  LongKeyPress, 
  AutoRepeatPress, 
  MultiClick, 
  NumEventTypes,         // Not an event, but this value used to size the number of columns in event/action array.
  AsyncEvents
};

// momentary button states
enum class pinState_t:uint8_t {                     // Enumeration to assist with program flow at state machine for reading button
  Released,                 // key is idle
  PressDebounce,            // key press triggered, waiting for debounce confirmation
  PressDown,                // key press debounced and confirmed
  PressOnHold,              // key is held in a pressed state
  ReleaseDebounce,          // received release interrput, but need to debounce and confirm
  PressRelease,             // key has been released and debounced
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
 * NOTE: this struct is 32 bit ligned
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
  private:

    /**
     * @brief GPIO ISR handler wrapper
     * 
     * @param arg - an object instance reference
     */
    static void isr_handler(void* arg);

    /**
     * @brief timer's callback wrapper
     * 
     * @param cbt - an event type to distinguish the proper timer
     */
    void timers_handler(timer_event_t cbt);

    /**
     * @brief process the ISR for the object instance
     * 
     */
    void gpio_update_from_isr();




    // Static class members shared by all instances of this object (common across all instances of the class)
    // ------------------------------------------------------------------------------------------------------
    void readButton();                                      // Static function to read button state (must be static to bind to GPIO and timer ISR)

    /**
     * @brief LongPress and Autorepeat handler
     * called via m_LongPressTimer and reuses the same timer to perform 'autorepeat on hold'
     * 
     */
    void longPressEvent();                                  // Wrapper / callback to excecute a longPress event

    //static void autoRepeatPressEvent(void *arg);                            // Wrapper / callback to excecute a autoRepeatPress event
    void clickTimeout();                              // Used to separate double-clicks from regular keyPress's
    //static void startTimer(esp_timer_handle_t &timer, uint32_t duration_US, void (*callBack)(void* arg), InterruptButton* btn, const char *msg);
    void killTimer(esp_timer_handle_t &timer);                       // Helper function to kill a timer


    // Non-static instance specific member declarations
    // ------------------------------------------------
    uint8_t m_menuLevel = 0;                                                // Current object's menulevel
    std::bitset<IBTN_MAX_MENU_DEPTH> menu_dblclck{0x0};                     // bit vector for tracking clicks at specific menulevel
    std::bitset<IBTN_MAX_MENU_DEPTH> menu_longpress{0x0};                   // bit vector for tracking longpress at specific menulevel
    std::bitset<IBTN_MAX_MENU_DEPTH> menu_repeat{0x0};                      // bit vector for tracking repeats at specific menulevel

    volatile pinState_t m_state = pinState_t::Released;
    esp_timer_handle_t m_DebounceTimer = nullptr;                           // Instance specific timer for button debouncing
    esp_timer_handle_t m_LongPressTimer = nullptr;                          // Instance specific timer for button longPress and autoRepeat timing
    esp_timer_handle_t m_ClickTimer = nullptr;                              // timer for counting consecutive clicks

    uint8_t m_pressedState;                                                 // State of button when it is pressed (LOW or HIGH)
    gpio_num_t m_pin;                                                       // Button gpio
    gpio_mode_t m_pinMode;                                                  // GPIO mode: IDF's input/output mode

    uint16_t m_gpio_debounceUS;
    uint16_t m_longKeyPressMS;
    uint16_t m_autoRepeatMS;
    uint16_t m_doubleClickMS;
     
    int8_t debounce_ctr{0};                                                  // debounce counter
    int8_t click_ctr{0};                                                     // click counter
    int16_t repeat_ctr{0};                                                   // repeat counter

    uint16_t eventMask = 0b10000111;        // Default to keyUp, keyDown, and keyPress enabled
                                            // When binding functions, longKeyPress, autoKeyPresses, and double clicks are automatically enabled.

    /**
     * @brief remove all action bindings
     * 
     */
    void unbindall();


  public:
    // Static class members shared by all instances of this object -----------------------
    void    setMenuLevel(uint8_t level);                       // Sets menu level across all buttons (ie buttons mean something different each page)
    uint8_t getMenuLevel();                                    // Retrieves menu level

    // Non-static instance specific member declarations ----------------------------------
    // Class Constructor
    InterruptButton(uint8_t pin, uint8_t pressedState,
                    gpio_mode_t pinMode = GPIO_MODE_INPUT,
                    uint16_t longKeyPressMS = IBTN_LONG_PRESS_TIME_MS,
                    uint16_t autoRepeatMS = IBTN_AUTO_REPEAT_TIME_MS,
                    uint16_t doubleClickMS = IBTN_DOUBLE_CLICK_TIME_MS,
                    uint32_t debounceUS = IBTN_DEBOUNCE_TIME_US);
    ~InterruptButton();                                               // Class Destructor


    /**
     * @brief configure gpio and attach interrupt monitor
     */
    void begin();                                                     // Instance initialiser

    /**
     * @brief stop interrupt and unconfigure gpio
     * 
     */
    void stop();

    void enableEvent(event_t event);
    void disableEvent(event_t event);
    bool eventEnabled(event_t event);

    void      setLongPressInterval(uint16_t intervalMS);              // Updates LongPress Interval
    inline uint16_t  getLongPressInterval(void) const { return m_longKeyPressMS; };
    void      setAutoRepeatInterval(uint16_t intervalMS);             // Updates autoRepeat Interval
    inline uint16_t  getAutoRepeatInterval(void) const { return m_autoRepeatMS; };
    void      setDoubleClickInterval(uint16_t intervalMS);            // Updates autoRepeat Interval
    inline uint16_t  getDoubleClickInterval(void) const { return m_doubleClickMS; };


    // Routines to manage interface with external action functions associated with each event ---
    // Any functions bound to Asynchronous (ISR driven) events should be defined with IRAM_ATTR attribute and be as brief as possible
    // Any functions bound to Synchronous events (Actioned by loop call of "button.processSyncEvents()") may be longer and also may be defined as Lambda functions

    void bind(  event_t event, btn_callback_t action, uint8_t menuLevel = 0);   // Used to bind/unbind action to an event at specified menu level
    void unbind(event_t event, uint8_t menuLevel = 0);
};

