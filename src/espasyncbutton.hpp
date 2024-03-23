#pragma once

#include "driver/gpio.h"
#include "esp_timer.h"
#include <functional>
#include "esp_event.h"
#include <bitset>
#include <list>
#include <vector>

#ifdef ARDUINO
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
#endif

#define IBTN_LONG_PRESS_TIME_MS       750
#define IBTN_LONG_PRESS_MIN_TIME_MS   250       // minimal time for longPress
#define IBTN_AUTO_REPEAT_TIME_MS      250
#define IBTN_AUTO_REPEAT_MIN_TIME_MS  50        // minimal time between autorepeat events
#define IBTN_MULTI_CLICK_TIME_MS      350
#define IBTN_MULTI_CLICK_MIN_TIME_MS  100
#define IBTN_DEBOUNCE_TIME_US         5000      // polling interval for debouncer
#define IBTN_DEBOUNCE_MIN_TIME_US     1000      // min interval for debouncer
#define IBTN_DEBOUNCE_MAX_TIME_US     20000     // max interval for debouncer
#define IBTN_DEBOUNCE_CNT             5         // number of consecutive readouts with the same value for the pin to be considered as debounced

#define IBTN_EVENT_T_SIZE             8         // size of the bitvector for button options (must be >= max of ESPButton::event_t)

#ifndef ESP_INTR_FLAG_DEFAULT
#define ESP_INTR_FLAG_DEFAULT         0
#endif


ESP_EVENT_DECLARE_BASE(EBTN_EVENTS);

static const char* EBTN_TAG = "EBTN";           // IDF log tag


namespace ESPButton {

  // Events that button generate on operation
  enum class event_t {
    press = 0,              // button was just pressed, it is not yet know how long it will reside in that "pressed" state
    release,                // button was just released after shot press
    click,                  // short press+release event that completes faster then longPress time, i.e. a "mouse click"
    longPress,              // button was pressed and was held in a pressed state long enough that it can't be treaded as 'click'
    longRelease,            // button was releases after being long-pressed for quite some time
    autoRepeat,             // button is being held down longer than longPress interval, autorepeat events are generated periodically while button is in long-pressed state
    multiClick,             // button has been clicked many times consecutevily (a number of clicks is also reported within the event)
    undefined               // unknown/undefined event
  };

  /**
   * @brief get ESPButton event loop handler
   * if if was set previously, otherwise return null
   * if handler is not set, than events are sent to default system loop
   * @return esp_event_loop_handle_t 
   */
  esp_event_loop_handle_t get_event_loop_hndlr();

  /**
   * @brief Set ESP Event loop hndlr where to post Button events
   * by default handler is NULL and events are posted to system default loop
   * 
   */
  void set_event_loop_hndlr( esp_event_loop_handle_t handler);

  /**
   * @brief function to convert integer to event_t
   * 
   * @return event_t 
   */
  event_t int2event_t(int32_t e);

  /**
   * @brief run Button event Queue and  task
   * 
   * @return true on success
   * @return false of error
   */
  //bool startBtnQTask();

  /**
   * @brief stop Button event queue task
   * 
   */
  //void stopBtnQTask();

  /**
   * @brief unary predicate for EventCallback structs matching
   * 
   * @tparam T 
   */
  template <class T>
  class MatchEventCallback : public std::unary_function<T, bool>{
      int32_t _gpio;
      uint32_t _menu;

  public:
      explicit MatchEventCallback(uint32_t gpio, uint32_t menu) : _gpio(gpio), _menu(menu) {}
      bool operator() ( T val ){
          // T is struct EventCallback
          return val.gpio == _gpio && val.menulvl == _menu;
      }
  };

} //namespace ESPButton


  // momentary button states
  enum class btnState_t {
    undefined=0,
    idle,                     // key is idle
    pressDebounce,            // key press triggered, waiting for debounce confirmation
    pressed,                  // key press debounced and confirmed (generates and event )
    onHold,                   // key is held in a pressed state for time < LongPress
    onLongHold,               // key is held in a pressed state for time > LongPress
    releaseDebounce,          // received release interrput, but need to debounce and confirm
    releaseLongDebounce,      // received release interrput after a LongHold, but need to debounce and confirm
    released,                 // key has just been released and debounced (generates and event )
    releasedLong              // key has just been released and debounced from a long-hold (generates and event )
  };

  /**
   * @brief Button event message structure
   * holds information about event for a specific button instance
   */
  struct EventMsg {
    // gpio that produced and event, could be real gpio or virtual 
    int32_t gpio;
    // counter for an event, optional filed, i.e. could be a number of multiclicks, etc
    int32_t cntr;
  };


using btn_callback_t = std::function< void (ESPButton::event_t event, const EventMsg* msg)>;

class TimeOuts
{
  uint32_t debounce{IBTN_DEBOUNCE_TIME_US};           // gpio debounce timeout, us
  uint32_t longPress{IBTN_LONG_PRESS_TIME_MS};        // how long to hold a button to consider it as a longPress, ms
  uint32_t autoRepeat{IBTN_AUTO_REPEAT_TIME_MS};      // autorepeat interval, ms
  uint32_t multiClick{IBTN_MULTI_CLICK_TIME_MS};      // time between consecutive keypress when it's considered as multiple clicks, ms

public:
  uint32_t getDebounce() const { return debounce; };
  uint32_t getLongPress() const { return longPress; };
  uint32_t getAutoRepeat() const { return autoRepeat; };
  uint32_t getMultiClick() const { return multiClick; };

  void setDebounce( uint32_t us );
  void setLongPress( uint32_t ms ){ longPress = ms < IBTN_LONG_PRESS_MIN_TIME_MS ? IBTN_LONG_PRESS_MIN_TIME_MS : ms; };
  void setAutoRepeat( uint32_t ms ){ autoRepeat = ms < IBTN_AUTO_REPEAT_MIN_TIME_MS ? IBTN_AUTO_REPEAT_MIN_TIME_MS : ms; };
  void setMultiClick( uint32_t ms ){ multiClick = ms < IBTN_MULTI_CLICK_MIN_TIME_MS ? IBTN_MULTI_CLICK_MIN_TIME_MS : ms; };

};

/**
 * @brief abstract class that implements "Momentary Button" and it's states
 * it contains timers and variables to track button state changes but does not has
 * a bound to any real gpio or other source, it is to be implemented in derived classes
 */
template<class EventPolicy>
class GenericButton {

  // a template policy that will implement button events generation
  EventPolicy _eventPolicy;

  // Instance specific timer for button longPress and autoRepeat timing
  esp_timer_handle_t longPressTimer_h = nullptr;
  // timer for counting consecutive clicks
  esp_timer_handle_t multiclickTimer_h = nullptr;
  // long-press timer instance count
  uint32_t _lpcnt{0};
  // multiclick timer instance count
  uint32_t _mccnt{0};

  /**
   * @brief helper method to create esp HPET timer
   * 
   * @param timer
   * @param tevent
   */
  void _createTimer(ESPButton::event_t tevent);

  // Helper function to kill a timer
  void _deleteTimer(ESPButton::event_t tevent);

  /**
   * @brief LongPress and Autorepeat handler
   * called via longPressTimer_h and reuses the same timer to perform 'autorepeat on hold'
   * 
   */
  void longPressTimeout();                                // timer callback to excecute a longPress event

  void multiclickTimeout();                                    // Used to separate double-clicks from regular keyPress's

protected:

  struct Counters
  {
    int8_t click{0};        // click counter
    int8_t repeat{0};       // repeat counter
  };

  /**
   * @brief virtual gpio
   * a virtual gpio for the button, could be mapped to real gpio or arbitraty gpio id
   */
  int32_t vgpio{-1};

  /**
   * @brief Button's logical state
   * 
   */
  btnState_t _state{btnState_t::undefined};

  // event counters
  Counters ctr;

  /**
   * @brief Button options
   * by default 0B11, i.e. only 'press' and 'release' events are enabled
   */
  std::bitset<IBTN_EVENT_T_SIZE> options{0x3};

  /**
   * @brief method is called when button's state changes
   * it might generate and event in responce to a state change
   * 
   */
  void checkState();

  /**
   * @brief abstract method that generates and sends button's event
   * implementation is specific to derived class
   * 
   */
  void sendButtonEvent(ESPButton::event_t e, EventMsg *msg){ _eventPolicy.event(e, msg); };

public:
  GenericButton(const EventPolicy& policy = EventPolicy()) : _eventPolicy(policy) {}
  virtual ~GenericButton();

  // timeouts and threshold times
  TimeOuts timeouts;

  /**
   * @brief enable button and start sending events
   * 
   * @return esp_err_t 
   */
  virtual esp_err_t enable() = 0;

  /**
   * @brief disable button and stop sending events
   * 
   */
  virtual void disable() = 0;

  /**
   * @brief return current Button's logical state
   * 
   * @return btnState_t 
   */
  btnState_t getState() const { return _state; };

  /**
   * @brief return virtual GPIO number
   * virtual gpio is just an identifier for the button instance, it migh (or might not) be a real gpio
   * 
   * @return int32_t 
   */
  int32_t getGPIO() const { return vgpio; };

  /**
   * @brief enable/disable generation of specific event types
   * by default only event_t::press and event_t::release are generated
   * @param e - event type
   * @param state - true of false to enable/disable events
   */
  void enableEvent(ESPButton::event_t e, bool state = true);

  /**
   * @brief enable/disable generation of specific event types
   * by default only event_t::press and event_t::release are generated
   * 
   * @param bitsset bitset with ESPButton::event_t
   */
  //void enableEvent(uint8_t bitsset);


  /**
   * @brief Get status if specific event generation is enabled/disabled 
   * 
   * @param opt 
   * @return true 
   * @return false 
   */
  bool checkEvent(ESPButton::event_t e) const { return options.test(static_cast<std::size_t>(e)); };
};


/**
 * @brief a policy 
 * 
 */
struct ESPEventPolicy {
  /**
   * @brief send button event via
   * implementation is specific to derived class
   * 
   */
  void event(ESPButton::event_t e, EventMsg *msg);

};


/**
 * @brief GPIO-based Buttons with or w/o debouncer
 * 
 */
template<class EventPolicy = ESPEventPolicy>
class GPIOButton : public GenericButton<EventPolicy> {

  gpio_num_t _gpio = GPIO_NUM_NC;                                         // Button gpio
  bool _gpio_ll;                                                          // Logic Level of a button when it is pressed (LOW or HIGH)
  gpio_pull_mode_t _gpioPull;                                             // gpio pull mode
  gpio_mode_t _gpioMode;                                                  // GPIO mode: IDF's input/output mode
  bool _debounce;                                                         // enable debounce for GPIO
  uint32_t _ctr_debounce;                                                 // debounce counter

  esp_timer_handle_t debounceTimer_h = nullptr;                            // Instance specific timer for button debouncing

  /**
   * @brief GPIO ISR handler wrapper
   * 
   * @param arg - an object instance reference
   * @note the pin ISR handlers no longer need to be declared with IRAM_ATTR
   * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv420gpio_isr_handler_add10gpio_num_t10gpio_isr_tPv
   */
  static void isr_handler(void* arg){ static_cast<GPIOButton*>(arg)->_gpio_isr(); };

  /**
   * @brief process the ISR for the object instance
   * 
   */
  void _gpio_isr();

  /**
   * @brief debounce timer callback
   * 
   */
  void _debounceCheck();

  // timer create helper
  void _createDebounceTimer();

public:
    /**
     * @brief Construct a new GPIOButton object
     * 
     * @param pin - gpio to attach button to
     * @param logicLevel - active LogicLevel for a button, i.e. gpio's level when button is pressed - 'false' for LOW, 'true' for HIGH
     * @param pull - gpio pull mode configuration, see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv416gpio_pull_mode_t
     * @param mode - gpio mode configuration, see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv411gpio_mode_t
     */
    GPIOButton(gpio_num_t gpio, bool logicLevel, gpio_pull_mode_t pull = GPIO_PULLUP_ONLY, gpio_mode_t mode = GPIO_MODE_INPUT, bool debounce = true);
    // Class Destructor
    virtual ~GPIOButton();


    /**
     * @brief configure gpio and attach interrupt monitor
     */
    esp_err_t enable() override;

    /**
     * @brief disable gpio interrupt handling
     * deactivates button
     */
    void disable() override;

  /**
   * @brief Set/change gpio
   * 
   * @return esp_err_t 
   */
  esp_err_t setGPIO(gpio_num_t gpio, bool logicLevel, gpio_pull_mode_t pull = GPIO_PULLUP_ONLY, gpio_mode_t mode = GPIO_MODE_INPUT);

  void setDebounce(bool debounce){ disable(); _debounce = debounce; enable(); };

  bool getDebounce(){ return _debounce; };

};

/**
 * @brief A generic class that provides callback registration and handling
 * for events recieved from a button direvied classes
 * 
 */
class ButtonCallbackMenu {
  // current menulevel
  uint32_t _level{0};

  /**
   * @brief button event callback structure
   * menulvl - button's menulevel at the time of event
   * event - event type
   * cb - functional callback
   */
  struct EventCallback {

    uint32_t menulvl;
    int32_t gpio;
    btn_callback_t cb;

    EventCallback(uint32_t lvl, int32_t gpio, btn_callback_t c) : menulvl(lvl), gpio(gpio), cb(c) {};
    //bool operator== (const btntrigger_t& b) const { return (gpio == b.gpio && menulvl == b.menulvl && event == b.event); };
  };

  // a set of assigned callbacks
  std::list<EventCallback> callbacks;

public:
  //virtual ~ButtonCallbackMenu(){};

  /**
   * @brief switch current menulevel for the button callbacks
   * 
   * @param level
   */
  void setMenuLevel(uint32_t level){ _level = level; };

  // Get current menu level
  uint32_t getMenuLevel() const { return _level; }

  /**
   * @brief assign functional callback
   * 
   * @param gpio - gpio to bind events to
   * @param menuLevel - menulevel to handle
   * @param callback - functional callback
   */
  void assign(int32_t gpio, uint32_t menuLevel, btn_callback_t callback);

  /**
   * @brief remove callback for specific gpio/menu
   * 
   * @param gpio 
   * @param menuLevel 
   */
  void deassign(int32_t gpio, uint32_t menuLevel);

  /**
   * @brief remove all action bindings
   * 
   */
  void reset() { callbacks.clear(); };

  /**
   * @brief Handle ESPButton::event_t Events and execute mapped callback based on current menulevel
   * 
   * @param event - ESPButton::event_t 
   * @param m - event message
   */
  void handleEvent(ESPButton::event_t event, const EventMsg* m);
};

// Templates implementations ----------------------------------------------------

// === GenericButton implementation ===
template<class EventPolicy>
GenericButton<EventPolicy>::~GenericButton(){
  esp_timer_stop(longPressTimer_h);
  esp_timer_delete(longPressTimer_h);
  esp_timer_stop(multiclickTimer_h);
  esp_timer_delete(multiclickTimer_h);
}

template<class EventPolicy>
void GenericButton<EventPolicy>::checkState(){
//ESP_LOGW(EBTN_TAG, "chck");
  switch(_state){
    // press-down event
    case btnState_t::pressed: {
      // generate press event if enabled
      if ( options.test(static_cast<std::size_t>(ESPButton::event_t::press)) ){
        EventMsg m{ vgpio, 0};
        sendButtonEvent(ESPButton::event_t::press, &m);
      }
      // run log-press timer once if LogPress or AutoRepeat actions are enabled
      if ( options.test(static_cast<std::size_t>(ESPButton::event_t::longPress)) || options.test(static_cast<std::size_t>(ESPButton::event_t::autoRepeat)) ){
        esp_timer_start_once(longPressTimer_h, timeouts.getLongPress() * 1000);
      }

      break;
    }

    case btnState_t::onHold: {
      // do nothing here, wait for either release or longpress timers
      break;
    }

    case btnState_t::released: {
      if(longPressTimer_h){                                                   // cancel longpress and autorepeat action on release
        esp_timer_stop(longPressTimer_h);
        ctr.repeat = 0;
      }

      EventMsg m{ vgpio, 0};
      // send 'release' event if enabled regardless of any 'clicks' options
      if ( options.test(static_cast<std::size_t>(ESPButton::event_t::release)) )
        sendButtonEvent(ESPButton::event_t::release, &m);

      // check if multiclicks are enabled
      if ( options.test(static_cast<std::size_t>(ESPButton::event_t::multiClick)) ){
        esp_timer_stop(multiclickTimer_h);                                    // (re)start click timer
        esp_timer_start_once(multiclickTimer_h, timeouts.getMultiClick() * 1000);
        ++ctr.click;                                                          // increment click counter
        // go wait for either more clicks or multiclick timer expired
      } else {
        // send click event if clicks are enabled
        if ( options.test(static_cast<std::size_t>(ESPButton::event_t::click)) )
          sendButtonEvent(ESPButton::event_t::click, &m);
      }
      _state = btnState_t::idle;
      break;
    }

    case btnState_t::releasedLong: {
      if(longPressTimer_h){                                                   // cancel longpress and autorepeat action on release
        esp_timer_stop(longPressTimer_h);
        ctr.repeat = 0;
      }

      EventMsg m{ vgpio, 0};
      if ( options.test(static_cast<std::size_t>(ESPButton::event_t::longRelease)) )
        sendButtonEvent(ESPButton::event_t::longRelease, &m);

      _state = btnState_t::idle;                                              // change button state to 'idle' and wait for next press/click
      break;
    }
    default:;
  }
}

//-- Helper method to simplify starting a timer ----------------------------------------------------------
template<class EventPolicy>
void GenericButton<EventPolicy>::_createTimer(ESPButton::event_t tevent){
  esp_timer_create_args_t tmrConfig;

  tmrConfig.arg = static_cast<void*>(this);
  tmrConfig.dispatch_method = ESP_TIMER_TASK;
  tmrConfig.skip_unhandled_events = true;

  switch (tevent)  {
    case ESPButton::event_t::multiClick :
      if (!_mccnt){
        tmrConfig.callback = [](void* self) { static_cast<GenericButton<EventPolicy>*>(self)->multiclickTimeout(); };
        ESP_ERROR_CHECK(esp_timer_create(&tmrConfig, &multiclickTimer_h));
      }
      ++_mccnt;
      break;

    case ESPButton::event_t::longPress :
      if (!_lpcnt){
        tmrConfig.callback = [](void* self) { static_cast<GenericButton<EventPolicy>*>(self)->longPressTimeout(); };
        ESP_ERROR_CHECK(esp_timer_create(&tmrConfig, &longPressTimer_h));
      }
      ++_lpcnt;
      break;

    default:;
  }
}

//-- Helper method to kill a timer -----------------------------------------------------------------------
template<class EventPolicy>
void GenericButton<EventPolicy>::_deleteTimer(ESPButton::event_t tevent){
  switch (tevent)  {
    case ESPButton::event_t::multiClick :
      if (_mccnt){
        --_mccnt;
        if (!_mccnt){
          esp_timer_stop(multiclickTimer_h);
          esp_timer_delete(multiclickTimer_h);
        }
      }
      break;

    case ESPButton::event_t::longPress :
      if (_lpcnt){
        --_lpcnt;
        if (!_lpcnt){
          esp_timer_stop(longPressTimer_h);
          esp_timer_delete(longPressTimer_h);
        }
      }
      break;

    default:;
  }
}

//-- Method to handle longKeyPresses (called by timer)----------------------------------------------------
template<class EventPolicy>
void GenericButton<EventPolicy>::longPressTimeout(){
  // check if we are in proper state
  //if (_state != btnState_t::onHold || _state != btnState_t::onLongHold) return;
  _state = btnState_t::onLongHold;
  EventMsg m{ vgpio, 0};

  ESPButton::event_t e = ctr.repeat ? ESPButton::event_t::autoRepeat : ESPButton::event_t::longPress;          // first event is 'LongPress', all consecutive is 'Autorepeat';

  // first longpress event
  if (!ctr.repeat && options.test(static_cast<std::size_t>(ESPButton::event_t::longPress)) )
    sendButtonEvent(e, &m);

  // quit if no autorepeats enabled
  if (!options.test(static_cast<std::size_t>(ESPButton::event_t::autoRepeat)) )
    return;

  // check if repeat-on-Hold is activated, then run periodic timer
  if( !ctr.repeat ){
    esp_timer_start_periodic(longPressTimer_h, timeouts.getAutoRepeat() * 1000);
    ++ctr.repeat;   // increment repeat counter
    return;
  }

  m.cntr = ctr.repeat++;    // postincrement repeat counter
  sendButtonEvent(e, &m);
}

//-- multiclick timer callback
template<class EventPolicy>
void GenericButton<EventPolicy>::multiclickTimeout(){
  EventMsg m{ vgpio, ctr.click };

  // check click counter, if there was only one click, than report it as 'click', otherwise report as 'MiltipleClick'
  sendButtonEvent(ctr.click > 1 ? ESPButton::event_t::multiClick : ESPButton::event_t::click, &m);
  ctr.click = 0;
}

template<class EventPolicy>
void GenericButton<EventPolicy>::enableEvent(ESPButton::event_t e, bool state){
  options.set(static_cast<std::size_t>(e), state);

  switch (e){
    case ESPButton::event_t::longPress :
    case ESPButton::event_t::longRelease :
    case ESPButton::event_t::autoRepeat :
      if (state)
        _createTimer(ESPButton::event_t::longPress);
      else
        _deleteTimer(ESPButton::event_t::longPress);
      break;
    case ESPButton::event_t::multiClick :
      if (state)
        _createTimer(ESPButton::event_t::multiClick);
      else
        _deleteTimer(ESPButton::event_t::multiClick);
      break;
  }
}



// Constructor ------------------------------------------------------------------
template<class EventPolicy>
GPIOButton<EventPolicy>::GPIOButton(gpio_num_t gpio, bool logicLevel, gpio_pull_mode_t pull, gpio_mode_t mode, bool debounce) :
                      _gpio(gpio),
                      _gpio_ll(logicLevel),
                      _gpioPull(pull),
                      _gpioMode(mode),
                      _debounce(debounce) {
  // gpio number sanity check
  if (!GPIO_IS_VALID_GPIO(gpio))
    _gpio = GPIO_NUM_NC;
  else
    this->vgpio = static_cast<int>(_gpio);
}

// Destructor --------------------------------------------------------------------
template<class EventPolicy>
GPIOButton<EventPolicy>::~GPIOButton() {
  disable();
  gpio_reset_pin(_gpio);
}

// Initialiser -------------------------------------------------------------------
template<class EventPolicy>
esp_err_t GPIOButton<EventPolicy>::enable(){
  esp_err_t err = ESP_ERR_INVALID_ARG;

  if (_gpio == GPIO_NUM_NC)
    return err;

  gpio_config_t gpio_conf = {};
  gpio_conf.mode = _gpioMode;
  gpio_conf.pin_bit_mask = BIT64(_gpio);
  gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
  err = gpio_config(&gpio_conf);
  if (err != ESP_OK)
    return err;

  gpio_set_pull_mode(_gpio, _gpioPull);

  this->_state = (gpio_get_level(_gpio) == _gpio_ll) ? btnState_t::onHold : btnState_t::idle;    // Set to gpio's level state when initialising

  err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);              // it' OK to call this function multiple times
  ESP_LOGD(EBTN_TAG, "GPIO ISR service installed with exit status: %d", err);

  _ctr_debounce = this->ctr.click = this->ctr.repeat = 0;             // clear counters

  gpio_isr_handler_add(_gpio, GPIOButton::isr_handler, static_cast<void*>(this));

  if (_debounce) _createDebounceTimer();                                      // preallocate debounce timer

  return err;
}

template<class EventPolicy>
void GPIOButton<EventPolicy>::_gpio_isr(){
  if (_debounce){
    switch(this->_state){
      case btnState_t::idle:                                            // Was sitting idle but just detected a signal from the button
        this->_state = btnState_t::pressDebounce;
        break;
      case btnState_t::onHold:
      case btnState_t::pressed:
        this->_state = btnState_t::releaseDebounce;
        break;
      case btnState_t::onLongHold:
        this->_state = btnState_t::releaseLongDebounce;
        break;
      default:
        // have an interrupt while in a wrong state, disable interrupts and leave it
        ESP_LOGW(EBTN_TAG, "isr err state %u", static_cast<unsigned>(this->_state));
        return;
    }
    // proceed with debounce polling
    gpio_intr_disable(_gpio);                                         // disable gpio ISR while we poll for a valid press (debouncer)
    _ctr_debounce = 0;
    if (debounceTimer_h)
      esp_timer_start_periodic(debounceTimer_h, this->timeouts.getDebounce() / IBTN_DEBOUNCE_CNT);

  } else {
    // if no debounce required
    if (gpio_get_level(_gpio) == _gpio_ll){
      this->_state = btnState_t::pressed;
    } else {
      this->_state = this->_state == btnState_t::onLongHold ? btnState_t::releasedLong : btnState_t::released;
    }
    this->checkState();
  }
}

template<class EventPolicy>
void GPIOButton<EventPolicy>::_createDebounceTimer(){
  if (debounceTimer_h != nullptr)
    return;         // timer already exist

  esp_timer_create_args_t tmrConfig;

  tmrConfig.arg = static_cast<void*>(this);
  tmrConfig.dispatch_method = ESP_TIMER_TASK;
  tmrConfig.skip_unhandled_events = true;
  tmrConfig.callback = [](void* self) { static_cast<GPIOButton<EventPolicy>*>(self)->_debounceCheck(); };
  esp_timer_create(&tmrConfig, &debounceTimer_h);
}

template<class EventPolicy>
void GPIOButton<EventPolicy>::_debounceCheck(){
  //ESP_LOGD(EBTN_TAG, "dbnc");
  switch(this->_state){
    case btnState_t::pressDebounce: {                                         // we get here each time the debounce timer expires
      if (gpio_get_level(_gpio) == _gpio_ll)
        ++_ctr_debounce;
      else
        --_ctr_debounce;

      // if debounce counter reached negative threshold => false press or noise
      if (_ctr_debounce == -1*IBTN_DEBOUNCE_CNT){
        this->_state = btnState_t::idle;
        esp_timer_stop(debounceTimer_h);                                      // stop debounce timer
        gpio_intr_enable(_gpio);                                              // enable gpio interrupt
        return;
      }

      if (_ctr_debounce < IBTN_DEBOUNCE_CNT)
        return;                                                               // keep debouncing

      // if debounce counter reached positive threshold => confirmed press
      this->_state = btnState_t::pressed;                                     // change button state to confirmed press-down
      esp_timer_stop(debounceTimer_h);                                        // cancel debounce polling
      gpio_intr_enable(_gpio);                                                // Begin monitoring pin again
        //ESP_LOGD(EBTN_TAG, "okpress");
      break;
    }

    case btnState_t::releaseDebounce :
    case btnState_t::releaseLongDebounce : {
      if(gpio_get_level(_gpio) == _gpio_ll)
        --_ctr_debounce;
      else
        ++_ctr_debounce;

      // if debounce counter reached negative threshold => false release or noise
      if (_ctr_debounce == -1*IBTN_DEBOUNCE_CNT){
        if (this->_state == btnState_t::releaseLongDebounce)
          this->_state = btnState_t::onLongHold;
        else
          this->_state = btnState_t::onHold;

        esp_timer_stop(debounceTimer_h);                                      // stop debounce timer
        gpio_intr_enable(_gpio);                                              // enable gpio interrupt
        return;
      }

      // if debounce counter reached positive threshold => confirmed KeyRelease
      if (_ctr_debounce < IBTN_DEBOUNCE_CNT)                                   // keep debouncing or monitor for gpio interrupt
        return;

      if (this->_state == btnState_t::releaseLongDebounce)
        this->_state = btnState_t::releasedLong;
      else
        this->_state = btnState_t::released;                                 // change button state to confirmed keyrelease

      // stop debounce timer
      esp_timer_stop(debounceTimer_h);
      // enable interrupt for gpio
      gpio_intr_enable(_gpio);
      break;
    }
    default:;
  } // End of SWITCH statement

  //ESP_LOGW(EBTN_TAG, "debounce OK");
  // process state change
  this->checkState();
}

template<class EventPolicy>
void GPIOButton<EventPolicy>::disable(){
  gpio_isr_handler_remove(_gpio);
  if(debounceTimer_h){
    esp_timer_stop(debounceTimer_h);
    esp_timer_delete(debounceTimer_h);
    debounceTimer_h = nullptr;
  }
  this->_state = btnState_t::undefined;
}

template<class EventPolicy>
esp_err_t GPIOButton<EventPolicy>::setGPIO(gpio_num_t gpio, bool logicLevel, gpio_pull_mode_t pull, gpio_mode_t mode){
  // gpio number sanity check
  if (!GPIO_IS_VALID_GPIO(gpio)){
    ESP_LOGW(EBTN_TAG, "%d is not valid gpio on this platform", gpio);
    return ESP_ERR_INVALID_ARG;
  }

  if (_gpio == GPIO_NUM_NC)
    disable();

  this->vgpio = static_cast<int>(_gpio);
  _gpio_ll = logicLevel;
  _gpioPull = pull;
  _gpioMode = mode;

  return enable();
}


