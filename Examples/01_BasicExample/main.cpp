#include "Arduino.h"
#include "InterruptButton.h"

#define BUTTON_1  0                 // Top Left or Bottom Right button
#define BUTTON_2  35                // Bottom Left or Top Right button

//== FUNCTION DECLARATIONS (Necessary if using PlatformIO as it doesn't do function hoisting) ======
//==================================================================================================
void menu0Button1press(void);
void menu0Button1release(void);
void menu0Button1keyPress(void);                 
void menu0Button1longKeyPress(void); 
void menu0Button1AutoRepeatKeyPress(void); 
void menu0Button1MultiClick(void); 

void menu0Button2press(void);
void menu0Button2release(void);
void menu0Button2keyPress(void);                 
void menu0Button2longKeyPress(void); 
void menu0Button2AutoRepeatKeyPress(void); 
void menu0Button2MultiClick(void); 

void menu1Button1press(void);
void menu1Button1release(void);
void menu1Button1keyPress(void);                 
void menu1Button1longKeyPress(void); 
void menu1Button1AutoRepeatKeyPress(void); 
void menu1Button1MultiClick(void); 

void menu1Button2press(void);
void menu1Button2release(void);
void menu1Button2keyPress(void);                 
void menu1Button2longKeyPress(void); 
void menu1Button2AutoRepeatKeyPress(void); 
void menu1Button2MultiClick(void); 


//-- BUTTON VARIABLES -----------------------------------------
InterruptButton button1(BUTTON_1, LOW);
InterruptButton* button2;   // object placeholder

using ESPButton::event_t;   // Import enum from a namespace

void setup() {
  delay(1000); // 3 second delay for recovery
  Serial.begin(115200);

  button2 = new InterruptButton(BUTTON_2, LOW);

  // SETUP THE BUTTONS -----------------------------------------------------------------------------
  button1.enable();
  button2->enable();

  button1.setMultiClickInterval(350);

  // -- Menu/UI Page 0 Functions --------------------------------------------------
  // ------------------------------------------------------------------------------

  // Asynchronous, fires as an interrupt so should be IRAM_ATTR and be FAST!
  button1.bind(event_t::press,          &menu0Button1press);
  button1.bind(event_t::release,            &menu0Button1release);
  button1.bind(event_t::click,         &menu0Button1keyPress);
  button1.bind(event_t::longPress,     &menu0Button1longKeyPress);
  button1.bind(event_t::autoRepeat,  &menu0Button1AutoRepeatKeyPress);
  button1.bind(event_t::multiClick,       &menu0Button1MultiClick);

  // Asynchronous, fires as an interrupt so should be IRAM_ATTR and be FAST!
  button2->bind(event_t::press,          &menu0Button2press);
  button2->bind(event_t::release,            &menu0Button2release);
  button2->bind(event_t::click,         [](){ Serial.printf("Menu 0, Button 2: click:              [%lu ms]\n", millis()); });  
  button2->bind(event_t::longPress,     [](){ Serial.printf("Menu 0, Button 2: longPress:          [%lu ms]\n", millis()); });  
  button2->bind(event_t::autoRepeat,  [](){ Serial.printf("Menu 0, Button 2: AutoRepeat Press:      [%lu ms]\n", millis()); });  
  button2->bind(event_t::multiClick,      [](){ Serial.printf("Menu 0, Button 2: multiClick:           [%lu ms]\n", millis()); });  


  // -- Menu/UI Page 1 Functions --------------------------------------------------
  // ------------------------------------------------------------------------------

  // Asynchronous, fires as an interrupt so should be IRAM_ATTR and be FAST!
  button1.bind(event_t::press,          &menu1Button1press, 1);
  button1.bind(event_t::release,            &menu1Button1release, 1);
  button1.bind(event_t::click,         [](){ Serial.printf("Menu 1, Button 1: click:              [%lu ms]\n", millis()); }, 1);
  button1.bind(event_t::longPress,     [](){ Serial.printf("Menu 1, Button 1: longPress:          [%lu ms]\n", millis()); }, 1);
  button1.bind(event_t::autoRepeat,  [](){ Serial.printf("Menu 1, Button 1: AutoRepeat Press:      [%lu ms]\n", millis()); }, 1);
  button1.bind(event_t::multiClick,      &menu1Button1MultiClick);

  // Asynchronous, fires as an interrupt so should be IRAM_ATTR and be FAST!
  button2->bind(event_t::press,         &menu1Button2press, 1);
  button2->bind(event_t::release,           &menu1Button2release, 1);
  button2->bind(event_t::multiClick,     &menu1Button2MultiClick, 1);
  button2->bind(event_t::click,        [](){ Serial.printf("Menu 1, Button 2: click:              [%lu ms]\n", millis()); }, 1);
  button2->bind(event_t::longPress,    [](){ Serial.printf("Menu 1, Button 2: longPress:          [%lu ms]\n", millis()); }, 1);
  button2->bind(event_t::autoRepeat, [](){ Serial.printf("Menu 1, Button 2: AutoRepeat Press :     [%lu ms]\n", millis()); }, 1);
  //button2.bind(InterruptButton::SyncMultiClick,  1, [](){ Serial.printf("Menu 1, Button 2: SYNC multiClick:           [%lu ms]\n", millis()); });  

}

//== INTERRUPT SERVICE ROUTINES ====================================================================
//==================================================================================================
// Button1 ISR's (Top left).
void IRAM_ATTR menu0Button1press(void)         { Serial.printf("Menu 0, Button 1:  Key Down:              %lu ms\n", millis()); }
void IRAM_ATTR menu0Button1release(void)           { Serial.printf("Menu 0, Button 1:  Key Up:                %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button1keyPress(void)        { Serial.printf("Menu 0, Button 1:  Key Press:             %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button1longKeyPress(void)    { Serial.printf("Menu 0, Button 1:  Long Key Press:        %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button1AutoRepeatKeyPress(void) { Serial.printf("Menu 0, Button 1:  Auto Repeat Key Press: %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button1MultiClick(void)  {
  Serial.printf("Menu 0, Button 1:  Double Click:          %lu ms - Disabling Sync events and changing to Menu Level ", millis());
  //button1.disableEvent(event_t::SyncEvents); button2.disableEvent(event_t::SyncEvents); 
  button1.setMenuLevel(1);
  Serial.println(button1.getMenuLevel());
} 

void IRAM_ATTR menu1Button1press(void)         { Serial.printf("Menu 1, Button 1:  Key Down:              %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button1release(void)           { Serial.printf("Menu 1, Button 1:  Key Up:                %lu ms\n", millis()); }      
void IRAM_ATTR menu1Button1keyPress(void)        { Serial.printf("Menu 1, Button 1:  Key Press:             %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button1longKeyPress(void)    { Serial.printf("Menu 1, Button 1:  Long Key Press:        %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button1AutoRepeatKeyPress(void) { Serial.printf("Menu 1, Button 1:  Auto Repeat Key Press: %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button1MultiClick(void)  { 
  Serial.printf("Menu 1, Button 1:  Double Click:          %lu ms - Changing Back to Menu Level ", millis());
  button1.setMenuLevel(0);
  Serial.println(button1.getMenuLevel());
} 

// Button2 ISR's (Bottom Left).
void IRAM_ATTR menu0Button2press(void)         { Serial.printf("Menu 0, Button 2:  Key Down:              %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button2release(void)           { Serial.printf("Menu 0, Button 2:  Key Up:                %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button2keyPress(void)        { Serial.printf("Menu 0, Button 2:  Key Press:             %lu ms\n", millis()); }         
void IRAM_ATTR menu0Button2longKeyPress(void)    { Serial.printf("Menu 0, Button 2:  Long Key Press:        %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button2AutoRepeatKeyPress(void) { Serial.printf("Menu 0, Button 2:  Auto Repeat Key Press: %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button2MultiClick(void)  { 
  Serial.printf("Menu 0, Button 2:  Double Click:          %lu ms - Enabling Sync events and changing to Menu Level ", millis());
  //button1.enableEvent(event_t::SyncEvents); button2->enableEvent(event_t::SyncEvents); 
  button2->setMenuLevel(1);
  Serial.println(button1.getMenuLevel());
} 

void IRAM_ATTR menu1Button2press(void)         { Serial.printf("Menu 1, Button 2:  Key Down:              %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button2release(void)           { Serial.printf("Menu 1, Button 2:  Key Up:                %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button2keyPress(void)        { Serial.printf("Menu 1, Button 2:  Key Press:             %lu ms\n", millis()); }            
void IRAM_ATTR menu1Button2longKeyPress(void) { 
  Serial.printf("Menu 1, Button 2:  Long Press:            %lu ms - [NOTE FASTER click RESPONSE IF multiClick NOT DEFINED] Changing Back to Menu Level ", millis());
  button2->setMenuLevel(0);
  Serial.println(button2->getMenuLevel());
 }
void IRAM_ATTR menu1Button2AutoRepeatKeyPress(void) { Serial.printf("Menu 1, Button 2:  Auto Repeat Key Press: %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button2MultiClick(void) { 
  Serial.print("Menu 1, Button 2:  Double Click - Changing Back to Menu Level ");
  button2->setMenuLevel(0);
  Serial.println(button2->getMenuLevel());
}

void loop()
{

delay(100);
}
