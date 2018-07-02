/* =======================================================================================
    USB Keyboard & Mouse Interface for Fairlight CMI Series III
    (Keyboard interface may also be used with Series I / II / IIX)

    Teensy 3.6 used as core of interface.
    Note that you *must* use a Teensy 3.6!

    Copyright 2018, Joe Britt (britt@shapetable.com)
    
    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
    associated documentation files (the "Software"), to deal in the Software without restriction, 
    including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
    and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in all copies or substantial 
    portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
    NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    USB code based on public domain example "Mouse" from USBHost_t36 by Paul Stoffregren / PJRC.
 */

/* ---------------------------------------------------------------------------------------
    U S B
    Stuff
 */
 
#include "USBHost_t36.h"

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
KeyboardController keyboard1(myusb);
KeyboardController keyboard2(myusb);
USBHIDParser hid1(myusb);
MouseController mouse1(myusb);
uint32_t buttons_prev = 0;

USBDriver *drivers[] = { &hub1, &hub2, &keyboard1, &keyboard2, &hid1 };

#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))

const char * driver_names[CNT_DEVICES] = { "Hub1","Hub2", "KB1", "KB2", "HID1" };
bool driver_active[CNT_DEVICES] = { false, false, false, false };

USBHIDInput *hiddrivers[] = { &mouse1 };

#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))

const char * hid_driver_names[CNT_DEVICES] = { "Mouse1" };
bool hid_driver_active[CNT_DEVICES] = { false, false };
bool show_changed_only = false; 

// Turn these on to see internal state as keys are pressed
#if 0
  #define DEBUG(_x)         {Serial.print(_x);}
  #define DEBUG_HEX(_x)     {Serial.print(_x,HEX);}
#else
  #define DEBUG(_x)
  #define DEBUG_HEX(_x)
#endif

char OEMKey() {
  char oem;
  
  if (keyboard1) {
    oem = keyboard1.getOemKey();
  } else {
    oem = keyboard2.getOemKey();
  }

  return oem;
}

void OnRelease( int key ) {
  clear_cmi_key();
}

void OnPress( int key )
{
  char cmi_val = key;                                                 // assume it is an alpha/num/symbol
  
  DEBUG("key '");
  switch (key) {
    case KEYD_UP:         cmi_val = 0x1c;   DEBUG("UP");    break;    // CMI up arrow
    case KEYD_DOWN:       cmi_val = 0x1d;   DEBUG("DN");    break;    // CMI down arrow
    case KEYD_LEFT:       cmi_val = 0x1f;   DEBUG("LEFT");  break;    // CMI left arrow
    case KEYD_RIGHT:      cmi_val = 0x1e;   DEBUG("RIGHT"); break;    // CMI right arrow
    
    case KEYD_INSERT:                       DEBUG("Ins");   break;    //
    case KEYD_DELETE:     cmi_val = 0x0b;   DEBUG("Del");   break;    // CMI sub
    
    case KEYD_PAGE_UP:    cmi_val = 0x0e;   DEBUG("PUP");   break;    // CMI add
    case KEYD_PAGE_DOWN:  cmi_val = 0x19;   DEBUG("PDN");   break;    // CMI set
    
    case KEYD_HOME:       cmi_val = 0x18;   DEBUG("HOME");  break;    // CMI home
    case KEYD_END:        cmi_val = 0x0c;   DEBUG("END");   break;    // CMI clear

    case 0x0a:            cmi_val = 0x0d;   DEBUG("ENTER"); break;    // CMI return
    
    case KEYD_F1:         cmi_val = 0x81;   DEBUG("F1");    break;    // CMI F1
    case KEYD_F2:         cmi_val = 0x82;   DEBUG("F2");    break;    // CMI F2
    case KEYD_F3:         cmi_val = 0x83;   DEBUG("F3");    break;    // CMI F3
    case KEYD_F4:         cmi_val = 0x84;   DEBUG("F4");    break;    // CMI F4
    case KEYD_F5:         cmi_val = 0x85 ;  DEBUG("F5");    break;    // CMI F5
    case KEYD_F6:         cmi_val = 0x86;   DEBUG("F6");    break;    // CMI F6
    case KEYD_F7:         cmi_val = 0x87;   DEBUG("F7");    break;    // CMI F7
    case KEYD_F8:         cmi_val = 0x88;   DEBUG("F8");    break;    // CMI F8
    case KEYD_F9:         cmi_val = 0x89;   DEBUG("F9");    break;    // CMI F9
    case KEYD_F10:        cmi_val = 0x8a;   DEBUG("F10");   break;    // CMI F10
    case KEYD_F11:        cmi_val = 0x8b;   DEBUG("F11");   break;    // CMI F11
    case KEYD_F12:        cmi_val = 0x8c;   DEBUG("F12");   break;    // CMI F12

    case 0x00:            switch( OEMKey() ) {
                            case 0x46:      cmi_val = 0x8d;   DEBUG("PrtScr");    break;    // CMI F13
                            case 0x47:      cmi_val = 0x8e;   DEBUG("ScrLk");     break;    // CMI F14
                            case 0x48:      cmi_val = 0x8f;   DEBUG("Pause");     break;    // CMI F15
                            case 0x2a:      cmi_val = 0x7f;   DEBUG("Bksp");      break;    // CMI Rub Out
                          }
                          break;
                          
    default:              DEBUG((char)key); break;
  }
  DEBUG("'  ");
  DEBUG_HEX(key);
  DEBUG(" MOD: ");
  
  if (keyboard1) {
    DEBUG_HEX(keyboard1.getModifiers());
    DEBUG(" OEM: ");
    DEBUG_HEX(keyboard1.getOemKey());
    DEBUG(" LEDS: ");
    DEBUG_HEX(keyboard1.LEDS());
  } else {
    DEBUG_HEX(keyboard2.getModifiers());
    DEBUG(" OEM: ");
    DEBUG_HEX(keyboard2.getOemKey());
    DEBUG(" LEDS: ");
    DEBUG_HEX(keyboard2.LEDS());
  }

  DEBUG("\n");

  if( cmi_val ) {
    send_cmi_key( cmi_val );
  }

  //Serial.print("key ");
  //Serial.print((char)keyboard1.getKey());
  //Serial.print("  ");
  //Serial.print((char)keyboard2.getKey());
  //Serial.println();
}


/* ---------------------------------------------------------------------------------------
    C M I
    Stuff

    The original (I/II/IIX) and III keyboards both send ASCII for keypresses.
    No key-up messages, just simple ASCII sent on key-down.

    The keyboard does to delay-until-repeat and auto-repeat (Typematic), we emulate that behavior.

    The Series III keyboard G-Pad sends 6-byte movement packets, with the first byte always = 0x80.

    The Series III keyboard F1 - F15 keys send key values of 0x81 - 0x8f, respectively.

    The Series I / II / IIX keyboard only sends uppercase ASCII.

    A GPIO input is used to select Series I/II/IIX or Series III mode.
    An LED is blinked when data (keyboard or mouse) is sent to the CMI.
 */
 
#define CMI_SERIAL      Serial1

#define MODE_SEL        2                 // input selects Series I/II/IIX or III mode
#define ACT_LED         13                // activity/power LED

int loopcnt;                              // # of passes thru loop() before turning the LED back on
char last_cmi_key;                        // set to the last key char we sent, cleared on key up, used for typematic
bool key_repeating = false;               // if true, we are repeating

#define DELAY_UNTIL_REPEAT    350         // typematic (auto repeat) millisecond delay until repeat
#define REPEAT_TIME           100         // milliseconds between repeat chars

#include <elapsedMillis.h>
elapsedMillis keyTimeElapsed;             // measure elapsed time for key autorepeat (typematic)

bool is_series_3() {
  return true;                            // XXX TODO: read GPIO
}

bool repeating_cmi_key( char c ) {
  if( isprint( c ) || (c == 0x7f) )       // printable char OR rub out / backspace
    return true;
  else
    return false;
}

void clear_cmi_key() {
  last_cmi_key = 0;                       // reset typematic state
  key_repeating = false;
}

void send_cmi_key( char c ) {
  if( (c & 0x80) && !is_series_3() ) {      // if it's an SIII-only key and we are in I/II/IIX mode, don't send it
    return;
  } else {
    digitalWrite( ACT_LED, false );         // make LED flicker, will be turned back on in main loop
    loopcnt = 10000;                        // countdown for main loop
  
    if( !is_series_3() )                    // CMI just expects ASCII, no key-ups. Only send lower case to SIII
      c = toupper( c );
  
    DEBUG_HEX( c );
    CMI_SERIAL.write( c );
  
    last_cmi_key = c;
    keyTimeElapsed = 0;                     // start timer for delay-until-repeat
  }
}


#define MAX_GPAD_X    0x3ff
#define MAX_GPAD_Y    0x3ff

short     gpad_x;                         // G-Pad (Series III) cursor position
short     gpad_y;

short     last_gpad_x;                    // previous G-Pad (Series III) cursor position
short     last_gpad_y;                    //  (used to prevent re-sending same position over & over)

bool      show_cursor_gpad = true;
  
bool      mouse_left_button;

/*
  Each mouse movement results in 6 bytes being sent.
  At 9,600 bps, that takes ~1ms/byte
*/

void Send_GPad_Packet() {
  unsigned char b;

  if( is_series_3() ) {                     // only send gpad messages to Series III
    digitalWrite( ACT_LED, false );         // make LED flicker, will be turned back on in main loop
    loopcnt = 3500;  
    
    CMI_SERIAL.write( 0x80 );               // control byte
  
    b = 0xe0;                               // 111p tsc0: p = pen on pad
                                            //            t = touch
                                            //            s = shift key down (keyboard i/f adds)
                                            //        c = ctl key down (keyboard i/f adds)
  
    if ( mouse_left_button )
      b |= 0x08;                            // touch
  
    if ( show_cursor_gpad )
      b |= 0x10;                            // pen on pad
  
    CMI_SERIAL.write( b );
  
    CMI_SERIAL.write( 0xe0 | (gpad_x & 0x001f) );           // low 5 bits
    CMI_SERIAL.write( 0xe0 | ((gpad_x >> 5) & 0x001f ));    // hi 5 bits
  
    CMI_SERIAL.write( 0xe0 | (gpad_y & 0x001f) );           // low 5 bits
    CMI_SERIAL.write( 0xe0 | ((gpad_y >> 5) & 0x001f ));    // hi 5 bits
  }
}


void init_gpad() {
  last_gpad_x = gpad_x = (MAX_GPAD_X / 2);
  last_gpad_y = gpad_y = (MAX_GPAD_Y / 2);

  Send_GPad_Packet();     // get cursor on-screen and centered
}


void cmi_gpad_move( int8_t dx, int8_t dy ) {

  gpad_x += dx;

  if (gpad_x < 0)
    gpad_x = 0;
  else if (gpad_x > MAX_GPAD_X)
    gpad_x = MAX_GPAD_X;
  
  gpad_y -= dy;                       // G-Pad Y axis is reversed

  if (gpad_y < 0)
    gpad_y = 0;
  else if (gpad_y > MAX_GPAD_Y)
    gpad_y = MAX_GPAD_Y;


  Send_GPad_Packet();                 
}


/* ---------------------------------------------------------------------------------------
    setup & loop
 */

void setup()
{
  pinMode( ACT_LED, OUTPUT );
  digitalWrite( ACT_LED, true );

  Serial.begin( 115200 );
  delay( 1500 );                  // wait for serial, but don't block if it's not connected
  Serial.println("Welcome to USB Keyboard & Mouse adapter for CMI");
    
  Serial.println(sizeof(USBHub), DEC);

  CMI_SERIAL.begin( 9600 );       // init output port   

  init_gpad();

  myusb.begin();
  
  keyboard1.attachPress(OnPress);
  keyboard1.attachRelease(OnRelease);

  keyboard2.attachPress(OnPress);
  keyboard2.attachRelease(OnRelease);
}  


void loop()
{
  // ===============================
  // Update Activity LED
  
  if( loopcnt )
    loopcnt--;
  else
    digitalWrite( ACT_LED, true );

  // ===============================
  // Handle Typematic action (key repeat)
  
  if( key_repeating ) {
    if( keyTimeElapsed >= REPEAT_TIME ) {
      send_cmi_key( last_cmi_key );
      keyTimeElapsed = 0;
    }
  } else
    if( repeating_cmi_key(last_cmi_key) ) {                     // is a printing character key held down?
      if( keyTimeElapsed >= DELAY_UNTIL_REPEAT ) {
        key_repeating = true;
        keyTimeElapsed = 0;
      }
    }

  // ===============================
  // Give USB time
  
  myusb.Task();

  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }


  if(mouse1.available()) {
    /*
    Serial.print("Mouse: buttons = ");
    Serial.print(mouse1.getButtons());    
    Serial.print(",  mouseX = ");
    Serial.print(mouse1.getMouseX());
    Serial.print(",  mouseY = ");
    Serial.print(mouse1.getMouseY());
    Serial.print(",  wheel = ");
    Serial.print(mouse1.getWheel());
    Serial.print(",  wheelH = ");
    Serial.print(mouse1.getWheelH());
    Serial.println();
    */

    mouse_left_button = (mouse1.getButtons() & 0x01) ? true : false;

    if( mouse1.getButtons() & 0x02 )
      show_cursor_gpad = !show_cursor_gpad;

    cmi_gpad_move( mouse1.getMouseX(), mouse1.getMouseY() );

    mouse1.mouseDataClear();
  }

}


