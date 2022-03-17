#ifndef InterruptButton_h
#define InterruptButton_h

#include "driver/gpio.h"
#include "esp_timer.h"
#include <functional>     // Necessary to use std::bind to bind arguments to ISR in attachInterrupt()

typedef std::function<void ()> btn_callback_t;

enum class event_t:uint8_t {
  KeyDown = 0,          // These first 6 events are asycnchronous unless explicitly stated and actioned immediately in an ISR ...
  KeyUp,                // ... Asynchronous events should be defined with the IRAM_ATTR attributed so that ...
  KeyPress,             // ... Their code is stored in the RAM so as to not rely on SPI bus for timing to get from flash memory
  LongKeyPress, 
  AutoRepeatPress, 
  DoubleClick, 
  NumEventTypes,         // Not an event, but this value used to size the number of columns in event/action array.
  AsyncEvents
};


struct btntrigger_t
{
  gpio_num_t gpio;
  uint8_t menulvl;
  event_t event;

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
    enum pinState_t {                     // Enumeration to assist with program flow at state machine for reading button
      Released, 
      ConfirmingPress,
      Pressing,
      Pressed, 
      WaitingForRelease,
      Releasing,
      DblClickIdle,
      DblClickWaiting,
      DblClickTimeout
    };

    // Static class members shared by all instances of this object (common across all instances of the class)
    // ------------------------------------------------------------------------------------------------------
    static void readButton(void* arg);                                      // Static function to read button state (must be static to bind to GPIO and timer ISR)
    static void longPressEvent(void *arg);                                  // Wrapper / callback to excecute a longPress event
    static void autoRepeatPressEvent(void *arg);                            // Wrapper / callback to excecute a autoRepeatPress event
    static void doubleClickTimeout(void *arg);                              // Used to separate double-clicks from regular keyPress's
    static void startTimer(esp_timer_handle_t &timer, uint32_t duration_US, void (*callBack)(void* arg), InterruptButton* btn, const char *msg);
    static void killTimer(esp_timer_handle_t &timer);                       // Helper function to kill a timer

    static const uint8_t m_targetPolls = 10;                                // Desired number of polls required to debounce a button

    // Non-static instance specific member declarations
    // ------------------------------------------------
    uint8_t m_menuLevel = 0;                                                // Current menulevel for all buttons (global in class so common across all buttons)
    volatile pinState_t m_state, m_stateDblClick = DblClickIdle;            // Instance specific state machine variable (intialised when intialising button)
    esp_timer_handle_t m_buttonPollTimer = nullptr;                         // Instance specific timer for button debouncing
    esp_timer_handle_t m_buttonLPandRepeatTimer = nullptr;                  // Instance specific timer for button longPress and autoRepeat timing
    esp_timer_handle_t m_buttonDoubleClickTimer = nullptr;                  // Instance specific timer for policing double-clicks

    uint8_t m_pressedState;                                                 // State of button when it is pressed (LOW or HIGH)
    gpio_num_t m_pin;                                                       // Button gpio
    gpio_mode_t m_pinMode;                                                  // GPIO mode: IDF's input/output mode

    uint16_t m_pollIntervalUS, m_longKeyPressMS, m_autoRepeatMS,            // Timing variables
             m_doubleClickMS;
     
    volatile bool m_longPress_preventKeyPress;                              // Boolean flag to prevent firing a keypress if a long press occurred (outside of polling fuction)
    volatile uint16_t m_validPolls = 0, m_totalPolls = 0;                   // Variables to conduct debouncing algoritm

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
    InterruptButton(uint8_t pin, uint8_t pressedState, gpio_mode_t pinMode = GPIO_MODE_INPUT,     // Class Constructor
                    uint16_t longKeyPressMS = 750, uint16_t autoRepeatMS = 250,
                    uint16_t doubleClickMS = 200, uint32_t debounceUS = 8000);
    ~InterruptButton();                                               // Class Destructor
    void begin();                                                     // Instance initialiser
    void enableEvent(event_t event);
    void disableEvent(event_t event);
    bool eventEnabled(event_t event);

    void      setLongPressInterval(uint16_t intervalMS);              // Updates LongPress Interval
    uint16_t  getLongPressInterval(void);
    void      setAutoRepeatInterval(uint16_t intervalMS);             // Updates autoRepeat Interval
    uint16_t  getAutoRepeatInterval(void);
    void      setDoubleClickInterval(uint16_t intervalMS);            // Updates autoRepeat Interval
    uint16_t  getDoubleClickInterval(void);


    // Routines to manage interface with external action functions associated with each event ---
    // Any functions bound to Asynchronous (ISR driven) events should be defined with IRAM_ATTR attribute and be as brief as possible
    // Any functions bound to Synchronous events (Actioned by loop call of "button.processSyncEvents()") may be longer and also may be defined as Lambda functions

    void bind(  event_t event, uint8_t menuLevel, btn_callback_t action);   // Used to bind/unbind action to an event at specified menu level
    inline void bind(  event_t event, btn_callback_t action){ bind(event, m_menuLevel, action); }    // Used to bind/unbind action to an event at current m_menuLevel
    void unbind(event_t event, uint8_t menuLevel);
    inline void unbind(event_t event){ unbind(event, m_menuLevel); };
};

#endif
