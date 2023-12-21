#include <Arduino.h>
#include "InterruptButton.h"

#define BUTTON_1  0                 // Top Left or Bottom Right button
#define BUTTON_2  35                // Bottom Left or Top Right button

//== FUNCTION DECLARATIONS (Necessary if using PlatformIO as it doesn't do function hoisting) ======
//==================================================================================================
void menu0Button1keyDown(void);
void menu0Button1keyUp(void);
void menu0Button1keyPress(void);                 
void menu0Button1longKeyPress(void); 
void menu0Button1autoRepeatPress(void); 
void menu0Button1doubleClick(void); 

void menu0Button2keyDown(void);
void menu0Button2keyUp(void);
void menu0Button2keyPress(void);                 
void menu0Button2longKeyPress(void); 
void menu0Button2autoRepeatPress(void); 
void menu0Button2doubleClick(void); 

void menu1Button1keyDown(void);
void menu1Button1keyUp(void);
void menu1Button1keyPress(void);                 
void menu1Button1longKeyPress(void); 
void menu1Button1autoRepeatPress(void); 
void menu1Button1doubleClick(void); 

void menu1Button2keyDown(void);
void menu1Button2keyUp(void);
void menu1Button2keyPress(void);                 
void menu1Button2longKeyPress(void); 
void menu1Button2autoRepeatPress(void); 
void menu1Button2doubleClick(void); 


//-- BUTTON VARIABLES -----------------------------------------
InterruptButton button1(BUTTON_1, LOW);
InterruptButton* button2;   // object placeholder

//== MAIN SETUP FUNCTION ===========================================================================
//==================================================================================================

void setup() {
  Serial.begin(115200);                               // Remember to match platformio.ini setting here
  while(!Serial);                                     // Wait for serial port to start up

  button2 = new InterruptButton(BUTTON_2, LOW);

  // SETUP THE BUTTONS -----------------------------------------------------------------------------
  button1.begin();
  button2->begin();

  button1.setDoubleClickInterval(350);

  // -- Menu/UI Page 0 Functions --------------------------------------------------
  // ------------------------------------------------------------------------------

  // Asynchronous, fires as an interrupt so should be IRAM_ATTR and be FAST!
  button1.bind(event_t::KeyDown,          0, &menu0Button1keyDown);
  button1.bind(event_t::KeyUp,            0, &menu0Button1keyUp);
  button1.bind(event_t::KeyPress,         0, &menu0Button1keyPress);
  button1.bind(event_t::LongKeyPress,     0, &menu0Button1longKeyPress);
  button1.bind(event_t::AutoRepeatPress,  0, &menu0Button1autoRepeatPress);
  button1.bind(event_t::DoubleClick,      0, &menu0Button1doubleClick);
/*
  // Synchronous, so fires when triggered in main loop, can be Lamda
  button1.bind(event_t::SyncKeyPress,     0, [](){ Serial.printf("Menu 0, Button 1: SYNC KeyPress:              [%lu ms]\n", millis()); });  
  button1.bind(event_t::SyncLongKeyPress, 0, [](){ Serial.printf("Menu 0, Button 1: SYNC LongKeyPress:          [%lu ms]\n", millis()); });  
  button1.bind(event_t::SyncAutoRepeatPress, 0, [](){ Serial.printf("Menu 0, Button 1: SYNC AutoRepeat Press:      [%lu ms]\n", millis()); });  
  button1.bind(event_t::SyncDoubleClick,  0, [](){ Serial.printf("Menu 0, Button 1: SYNC DoubleClick:           [%lu ms]\n", millis()); });  
*/

  // Asynchronous, fires as an interrupt so should be IRAM_ATTR and be FAST!
  button2->bind(event_t::KeyDown,          0, &menu0Button2keyDown);
  button2->bind(event_t::KeyUp,            0, &menu0Button2keyUp);
  button2->bind(event_t::KeyPress,     0, [](){ Serial.printf("Menu 0, Button 2: KeyPress:              [%lu ms]\n", millis()); });  
  button2->bind(event_t::LongKeyPress, 0, [](){ Serial.printf("Menu 0, Button 2: LongKeyPress:          [%lu ms]\n", millis()); });  
  button2->bind(event_t::AutoRepeatPress, 0, [](){ Serial.printf("Menu 0, Button 2: AutoRepeat Press:      [%lu ms]\n", millis()); });  
  button2->bind(event_t::DoubleClick,  0, [](){ Serial.printf("Menu 0, Button 2: DoubleClick:           [%lu ms]\n", millis()); });  


  // -- Menu/UI Page 1 Functions --------------------------------------------------
  // ------------------------------------------------------------------------------

  // Asynchronous, fires as an interrupt so should be IRAM_ATTR and be FAST!
  button1.bind(event_t::KeyDown,          1, &menu1Button1keyDown);
  button1.bind(event_t::KeyUp,            1, &menu1Button1keyUp);
  button1.bind(event_t::KeyPress,     1, [](){ Serial.printf("Menu 1, Button 1: KeyPress:              [%lu ms]\n", millis()); });  
  button1.bind(event_t::LongKeyPress, 1, [](){ Serial.printf("Menu 1, Button 1: LongKeyPress:          [%lu ms]\n", millis()); });  
  button1.bind(event_t::AutoRepeatPress, 1, [](){ Serial.printf("Menu 1, Button 1: AutoRepeat Press:      [%lu ms]\n", millis()); });  
  button1.bind(event_t::DoubleClick,  1,    &menu1Button1doubleClick);

  // Asynchronous, fires as an interrupt so should be IRAM_ATTR and be FAST!
  button2->bind(event_t::KeyDown,          1, &menu1Button2keyDown);
  button2->bind(event_t::KeyUp,            1, &menu1Button2keyUp);
  button2->bind(event_t::DoubleClick,      1, &menu1Button2doubleClick);
  button2->bind(event_t::KeyPress,     1, [](){ Serial.printf("Menu 1, Button 2: KeyPress:              [%lu ms]\n", millis()); });  
  button2->bind(event_t::LongKeyPress, 1, [](){ Serial.printf("Menu 1, Button 2: LongKeyPress:          [%lu ms]\n", millis()); });  
  button2->bind(event_t::AutoRepeatPress, 1, [](){ Serial.printf("Menu 1, Button 2: AutoRepeat Press :     [%lu ms]\n", millis()); });  
  //button2.bind(InterruptButton::SyncDoubleClick,  1, [](){ Serial.printf("Menu 1, Button 2: SYNC DoubleClick:           [%lu ms]\n", millis()); });  

}

//== INTERRUPT SERVICE ROUTINES ====================================================================
//==================================================================================================
// Button1 ISR's (Top left).
void IRAM_ATTR menu0Button1keyDown(void)         { Serial.printf("Menu 0, Button 1:  Key Down:              %lu ms\n", millis()); }
void IRAM_ATTR menu0Button1keyUp(void)           { Serial.printf("Menu 0, Button 1:  Key Up:                %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button1keyPress(void)        { Serial.printf("Menu 0, Button 1:  Key Press:             %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button1longKeyPress(void)    { Serial.printf("Menu 0, Button 1:  Long Key Press:        %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button1autoRepeatPress(void) { Serial.printf("Menu 0, Button 1:  Auto Repeat Key Press: %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button1doubleClick(void)  {
  Serial.printf("Menu 0, Button 1:  Double Click:          %lu ms - Disabling Sync events and changing to Menu Level ", millis());
  //button1.disableEvent(event_t::SyncEvents); button2.disableEvent(event_t::SyncEvents); 
  button1.setMenuLevel(1);
  Serial.println(button1.getMenuLevel());
} 

void IRAM_ATTR menu1Button1keyDown(void)         { Serial.printf("Menu 1, Button 1:  Key Down:              %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button1keyUp(void)           { Serial.printf("Menu 1, Button 1:  Key Up:                %lu ms\n", millis()); }      
void IRAM_ATTR menu1Button1keyPress(void)        { Serial.printf("Menu 1, Button 1:  Key Press:             %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button1longKeyPress(void)    { Serial.printf("Menu 1, Button 1:  Long Key Press:        %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button1autoRepeatPress(void) { Serial.printf("Menu 1, Button 1:  Auto Repeat Key Press: %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button1doubleClick(void)  { 
  Serial.printf("Menu 1, Button 1:  Double Click:          %lu ms - Changing Back to Menu Level ", millis());
  button1.setMenuLevel(0);
  Serial.println(button1.getMenuLevel());
} 

// Button2 ISR's (Bottom Left).
void IRAM_ATTR menu0Button2keyDown(void)         { Serial.printf("Menu 0, Button 2:  Key Down:              %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button2keyUp(void)           { Serial.printf("Menu 0, Button 2:  Key Up:                %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button2keyPress(void)        { Serial.printf("Menu 0, Button 2:  Key Press:             %lu ms\n", millis()); }         
void IRAM_ATTR menu0Button2longKeyPress(void)    { Serial.printf("Menu 0, Button 2:  Long Key Press:        %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button2autoRepeatPress(void) { Serial.printf("Menu 0, Button 2:  Auto Repeat Key Press: %lu ms\n", millis()); }     
void IRAM_ATTR menu0Button2doubleClick(void)  { 
  Serial.printf("Menu 0, Button 2:  Double Click:          %lu ms - Enabling Sync events and changing to Menu Level ", millis());
  //button1.enableEvent(event_t::SyncEvents); button2->enableEvent(event_t::SyncEvents); 
  button2->setMenuLevel(1);
  Serial.println(button1.getMenuLevel());
} 

void IRAM_ATTR menu1Button2keyDown(void)         { Serial.printf("Menu 1, Button 2:  Key Down:              %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button2keyUp(void)           { Serial.printf("Menu 1, Button 2:  Key Up:                %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button2keyPress(void)        { Serial.printf("Menu 1, Button 2:  Key Press:             %lu ms\n", millis()); }            
void IRAM_ATTR menu1Button2longKeyPress(void) { 
  Serial.printf("Menu 1, Button 2:  Long Press:            %lu ms - [NOTE FASTER KEYPRESS RESPONSE IF DOUBLECLICK NOT DEFINED] Changing Back to Menu Level ", millis());
  button2->setMenuLevel(0);
  Serial.println(button2->getMenuLevel());
 }
void IRAM_ATTR menu1Button2autoRepeatPress(void) { Serial.printf("Menu 1, Button 2:  Auto Repeat Key Press: %lu ms\n", millis()); }     
void IRAM_ATTR menu1Button2doubleClick(void) { 
  Serial.print("Menu 1, Button 2:  Double Click - Changing Back to Menu Level ");
  button2->setMenuLevel(0);
  Serial.println(button2->getMenuLevel());
}

//== MAIN LOOP FUNCTION =====================================================================================
//===========================================================================================================

void loop() { 
  // Button lib does not need loop()
  vTaskDelete(NULL);
}
