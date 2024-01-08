#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "InterruptButton.h"
#include "LList.h"


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

using ESPButton::event_t;

ESP_EVENT_DEFINE_BASE(EBTN_EVENTS);

/*
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
*/
namespace ESPButton {

/**
 * @brief event loop handler where to post button events
 * if not set, then post events to system default event loop
 */
static esp_event_loop_handle_t ebtn_hndlr = nullptr;

esp_event_loop_handle_t get_evthndlr(){ return ebtn_hndlr; };

event_t int2event_t(int32_t e){
  return (e < 0 || e >= IBTN_EVENT_T_SIZE) ? event_t::undefined : static_cast<event_t>(e);
}

/*
bool startBtnQTask(){
  if (!getActionQ())
    return false;

  if (!t_action)
      return xTaskCreate(actionsTask, IBUTTON_EVT_TASK_NAME, IBUTTON_EVT_TASK_STACK, NULL, IBUTTON_EVT_TASK_PRIO, &t_action) == pdPASS;
  else
      return true;  
}
void stopBtnQTask(){
  if (t_action){
    vTaskDelete(t_action);
    t_action = nullptr;    
  }

  if (q_action){
    vQueueDelete(q_action);
    q_action = nullptr;
  }
}
*/

} // namespace ESPButton


/*
void BtnMenu::eventSet(event_t evt, bool state, uint8_t menulvl){
  if (menulvl >= menu.size())
    menu.resize(menulvl+1);

  menu.at(menulvl).set( static_cast<std::size_t>(evt), state );
}

bool BtnMenu::eventGet(event_t evt, uint8_t menulvl) const {
  if (menulvl >= menu.size())
    return false;

  return menu.at(menulvl).test( static_cast<std::size_t>(evt));
}
*/


// === GPIOButton methods ===
//-- Method to monitor button, called by button change and various timer interrupts ----------------------
#ifdef NOTHING
void ButtonCallbacks::onEvent(event_t event, btn_callback_t cb, uint8_t menuLevel){
  auto it = std::find_if(callbacks.begin(), callbacks.end(), MatchEventCallback<EventCallback>(menuLevel, event));
  if (it != callbacks.end()){
    it.cb = cb;
  } else
    callbacks.emplace_back(EventCallback(event, cb, menuLevel));

/*
  btnaction_t a = { {_gpio, menuLevel, event}, action};
  btn_actions.add(a);
  _menuopts.eventSet(event, true, menuLevel);

  if (_menuopts.eventGet(event_t::autoRepeat) || _menuopts.eventGet(event_t::longPress))
    createTimer(timer_t::longpress, longPressTimer_h);

  if (_menuopts.eventGet(event_t::multiClick))
    createTimer(timer_t::click, multiclickTimer_h);
*/
}

void ButtonCallbacks::unbind(event_t event, uint8_t menuLevel){
  callbacks.remove_if(  MatchEventCallback<EventCallback>(menuLevel, event) );
/*
  if (!btn_actions.size())    // action list is empty
    return;

  for (int i = 0; i != btn_actions.size(); ++i){
    if (btn_actions.get(i).t.gpio == _gpio && btn_actions.get(i).t.event == event && btn_actions.get(i).t.menulvl == menuLevel){
      _menuopts.eventSet(event, false, menuLevel);
      btn_actions.remove(i);
      return;
    }
  }
*/
}

void ButtonCallbacks::unbindall(){
  callbacks.clear();
/*
  if (!btn_actions.size())    // action list is empty
    return;

  int size = btn_actions.size();
  for (int i = 0; i != size; ++i){
    if (btn_actions.get(i).t.gpio == _gpio){
      btn_actions.remove(i);
      --size;
      --i;
    }
  }
*/
}

void ButtonCallbacks::setMenuLevel(uint32_t level){
  if (level > clicks.size())
    return;

  _menuopts.level = level;

  if (_menuopts.eventGet(event_t::autoRepeat) || _menuopts.eventGet(event_t::longPress))
    createTimer(timer_t::longpress, longPressTimer_h);
  else
    killTimer(longPressTimer_h);

  if (_menuopts.eventGet(event_t::multiClick))
    createTimer(timer_t::click, multiclickTimer_h);
  else
    killTimer(multiclickTimer_h);

}
#endif  //NOTHING

// === ESPEventPolicy methods ===

void ESPEventPolicy::event(ESPButton::event_t e, EventMsg *msg){
  if (ESPButton::ebtn_hndlr)
    esp_event_post_to(ESPButton::ebtn_hndlr, EBTN_EVENTS, static_cast<int32_t>(e), msg, sizeof(EventMsg), portMAX_DELAY);
  else
    esp_event_post(EBTN_EVENTS, static_cast<int32_t>(e), msg, sizeof(EventMsg), portMAX_DELAY);
}


