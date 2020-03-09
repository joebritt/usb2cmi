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

    3/7/20    v1.5    Fixed problem with mice sometimes not being recognized at boot
    11/30/19  v1.4    Fixed Shift/Control behavior for on-screen icons and Function keys
                      Removed debug code previously left in.
    2/3/19    v1.3    Added support for HDSP-21xx displays, made i2c probe more robust
    
 */

#define   FIRMWARE_VERSION          5
#define   FIRMWARE_VERSION_TEXT     "VERSION 1.5 "

#include <elapsedMillis.h>

/* ---------------------------------------------------------------------------------------
    Hardware Specifics
 */
 
#define CMI_SERIAL          Serial1           // to/from CMI
#define KEYBD_SERIAL        Serial2           // to/from legacy keyboard

#define EXP_SERIAL          Serial3

#define MIDI_SERIAL         Serial4

#define CMI_MIDI_KB_SERIAL  Serial5           // MIDI in from legacy CMI music keyboard

#define MODE_SEL            22                // input selects Series I/II/IIX or III mode

#define R_LED               16                // activity LED
#define G_LED               17

/* ---------------------------------------------------------------------------------------
    LED display support
    Multiple display types are supported.
    Which (if any) is connected is detected at boot.
    These general purpose routines handle detection of and writing characters to the display.
 */

bool init_led_display();

void blank_led_display();
void putc_led_display( char c );
void led_display_string( char *s );

/* ---------------------------------------------------------------------------------------
    Control Red & Green LEDs on toggle switch
 */
 
void led_green( bool led_on ) {
  digitalWrite( G_LED, led_on ? false : true );
}

void led_red( bool led_on ) {
  digitalWrite( R_LED, led_on ? false : true );
}

void led_blink_version( char vers ) {
  led_green( 0 );                             // power LED off

  for( int xxx = 0; xxx != vers; xxx++ ) {    // version shown as # of red blinks
    led_red( 1 );
    delay( 150 );
    led_red( 0 );
    delay( 250 );
  }
}


/* ---------------------------------------------------------------------------------------
    M I D I
    Stuff
 */

#include <MIDI.h>

MIDI_CREATE_INSTANCE(HardwareSerial, MIDI_SERIAL, MIDI);
const int midi_chan = 1;

#define MIDI_CHANNEL    midi_chan

//MIDI_CREATE_INSTANCE(HardwareSerial, CMI_MIDI_KB_SERIAL, MIDI_CMI_KB);


/* ---------------------------------------------------------------------------------------
    U S B
    Stuff
 */
 
#include "USBHost_t36.h"


class KeyboardControllerExt : public USBDriver , public USBHIDInput, public BTHIDInput {
public:
typedef union {
   struct {
        uint8_t numLock : 1;
        uint8_t capsLock : 1;
        uint8_t scrollLock : 1;
        uint8_t compose : 1;
        uint8_t kana : 1;
        uint8_t reserved : 3;
        };
    uint8_t byte;
} KBDLeds_t;
public:
  KeyboardControllerExt(USBHost &host) { init(); }
  KeyboardControllerExt(USBHost *host) { init(); }

  // need their own versions as both USBDriver and USBHIDInput provide
  uint16_t idVendor();
  uint16_t idProduct();
  const uint8_t *manufacturer();
  const uint8_t *product();
  const uint8_t *serialNumber();

  operator bool() { return ((device != nullptr) || (btdevice != nullptr)); }
  // Main boot keyboard functions. 
  uint16_t getKey() { return keyCode; }
  uint8_t  getModifiers() { return modifiers; }
  uint8_t  getOemKey() { return keyOEM; }
  void     attachPress(void (*f)(int unicode)) { keyPressedFunction = f; }
  void     attachRelease(void (*f)(int unicode)) { keyReleasedFunction = f; }
  void     LEDS(uint8_t leds);
  uint8_t  LEDS() {return leds_.byte;}
  void     updateLEDS(void);
  bool     numLock() {return leds_.numLock;}
  bool     capsLock() {return leds_.capsLock;}
  bool     scrollLock() {return leds_.scrollLock;}
  void   numLock(bool f);
  void     capsLock(bool f);
  void   scrollLock(bool f);

  // Added for extras information.
  void     attachExtrasPress(void (*f)(uint32_t top, uint16_t code)) { extrasKeyPressedFunction = f; }
  void     attachExtrasRelease(void (*f)(uint32_t top, uint16_t code)) { extrasKeyReleasedFunction = f; }
  void   forceBootProtocol();
  enum {MAX_KEYS_DOWN=4};


protected:
  virtual bool claim(Device_t *device, int type, const uint8_t *descriptors, uint32_t len);
  virtual void control(const Transfer_t *transfer);
  virtual void disconnect();
  static void callback(const Transfer_t *transfer);
  void new_data(const Transfer_t *transfer);
  void init();

  // Bluetooth data
  virtual bool claim_bluetooth(BluetoothController *driver, uint32_t bluetooth_class, uint8_t *remoteName);
  virtual bool process_bluetooth_HID_data(const uint8_t *data, uint16_t length);
  virtual bool remoteNameComplete(const uint8_t *remoteName);
  virtual void release_bluetooth();


protected:  // HID functions for extra keyboard data. 
  virtual hidclaim_t claim_collection(USBHIDParser *driver, Device_t *dev, uint32_t topusage);
  virtual void hid_input_begin(uint32_t topusage, uint32_t type, int lgmin, int lgmax);
  virtual void hid_input_data(uint32_t usage, int32_t value);
  virtual void hid_input_end();
  virtual void disconnect_collection(Device_t *dev);

private:
  void update();
  uint16_t convert_to_unicode(uint32_t mod, uint32_t key);
  void key_press(uint32_t mod, uint32_t key);
  void key_release(uint32_t mod, uint32_t key);
  void (*keyPressedFunction)(int unicode);
  void (*keyReleasedFunction)(int unicode);
  Pipe_t *datapipe;
  setup_t setup;
  uint8_t report[8];
  uint16_t keyCode;
  uint8_t modifiers;
  uint8_t keyOEM;
  uint8_t prev_report[8];
  KBDLeds_t leds_ = {0};
  Pipe_t mypipes[2] __attribute__ ((aligned(32)));
  Transfer_t mytransfers[4] __attribute__ ((aligned(32)));
  strbuf_t mystring_bufs[1];

  // Added to process secondary HID data. 
  void (*extrasKeyPressedFunction)(uint32_t top, uint16_t code);
  void (*extrasKeyReleasedFunction)(uint32_t top, uint16_t code);
  uint32_t topusage_ = 0;         // What top report am I processing?
  uint8_t collections_claimed_ = 0;
  volatile bool hid_input_begin_ = false;
  volatile bool hid_input_data_ = false;  // did we receive any valid data with report?
  uint8_t count_keys_down_ = 0;
  uint16_t keys_down[MAX_KEYS_DOWN];
  bool  force_boot_protocol;  // User or VID/PID said force boot protocol?
  bool control_queued;
};


USBHost myusb;
USBHub hub1(myusb);

KeyboardControllerExt keyboard1(myusb);

USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);

MouseController mouse1(myusb);

uint32_t buttons_prev = 0;

USBDriver *drivers[] = { &hub1, &keyboard1, &hid1, &hid2 };

#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))

const char * driver_names[CNT_DEVICES] = { "Hub1", "KB1", "HID1" , "HID2" };

bool driver_active[CNT_DEVICES] = { false, false, false, false };

USBHIDInput *hiddrivers[] = { &mouse1 };

#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))

const char * hid_driver_names[CNT_DEVICES] = { "Mouse1" };
bool hid_driver_active[CNT_DEVICES] = { false, false };
bool show_changed_only = false; 

uint8_t lastMods1;                  // previous loop Shift & Control key states
uint8_t newMods1;                   // this loop Shift & Control key states

#define kUpModSendCount   5         // number of meta-key up g-pad packets to send
int modSentCount;                   // number of meta-key up g-pad packets sent


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
  }

  return oem;
}

void OnRelease( int key ) {
  clear_cmi_key();
}


#define USB_MOD_KEY_SHIFT                 0x22
#define USB_MOD_KEY_CONTROL               0x11

#define CMI_MOD_KEY_SHIFT                 0x10
#define CMI_MOD_KEY_CONTROL               0x20

void OnPress( int key )
{
  char cmi_val = key;                                                 // assume it is an alpha/num/symbol  
  uint8_t mods;           

  DEBUG("mods: ");
  DEBUG_HEX(mods);
  DEBUG("\n");
  
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
    mods = keyboard1.getModifiers();
    DEBUG_HEX(mods);
    DEBUG(" OEM: ");
    DEBUG_HEX(keyboard1.getOemKey());
    DEBUG(" LEDS: ");
    DEBUG_HEX(keyboard1.LEDS());
  }

  DEBUG("\n");

  if( ((cmi_val & 0xf0) == 0x80) && is_series_3() ) {                       // SIII jams Shift & Ctl state into Function keys
    if( mods & USB_MOD_KEY_SHIFT )   cmi_val |= CMI_MOD_KEY_SHIFT;          // Function keys care about Shift & Control
    if( mods & USB_MOD_KEY_CONTROL ) cmi_val |= CMI_MOD_KEY_CONTROL;        // Thanks to Graham Johnson for finding this bug!  
  }

  send_cmi_key( cmi_val );

  //Serial.print("key ");
  //Serial.print((char)keyboard1.getKey());
  //Serial.println();
}


/* ---------------------------------------------------------------------------------------
    C M I
    Stuff

    The original (I/II/IIX) and III keyboards both send ASCII for keypresses.
    No key-up messages, just simple ASCII sent on key-down.

    The keyboard does delay-until-repeat and auto-repeat (Typematic), we emulate that behavior.

    The Series III keyboard G-Pad sends 6-byte movement packets, with the first byte always = 0x80.

    The Series III keyboard F1 - F15 keys send key values of 0x81 - 0x8f, respectively.

    The Series I / II / IIX keyboard only sends uppercase ASCII.

    A GPIO input is used to select Series I/II/IIX or Series III mode.
    An LED is blinked when data (keyboard or mouse) is sent to the CMI.
 */

#define ACT_LED               R_LED

int loopcnt;                                // # of passes thru loop() before turning the LED back on
char last_cmi_key;                          // set to the last key char we sent, cleared on key up, used for typematic
bool key_repeating = false;                 // if true, we are repeating

#define DELAY_UNTIL_REPEAT    350           // typematic (auto repeat) millisecond delay until repeat
#define REPEAT_TIME           100           // milliseconds between repeat chars

#include <elapsedMillis.h>
elapsedMillis keyTimeElapsed;               // measure elapsed time for key autorepeat (typematic)

bool is_series_3() {
  return ( digitalRead( MODE_SEL ) ? true : false );
}

void send_cmi_char( char c ) {              // send a char that we want to flicker the LED for
    digitalWrite( ACT_LED, false );         // make LED flicker, will be turned back on in main loop
    loopcnt = 10000;  

    if( !is_series_3() )                    // CMI just expects ASCII, no key-ups. Only send lower case to SIII
      c = toupper( c );
      
    DEBUG_HEX( c );
    CMI_SERIAL.write( c );
}


bool repeating_cmi_key( char c ) {
  if( isprint( c ) || (c == 0x7f) )         // printable char OR rub out / backspace
    return true;
  else
    return false;
}

void clear_cmi_key() {
  last_cmi_key = 0;                         // reset typematic state
  key_repeating = false;
}

void send_cmi_key( char c ) {
  if( (c & 0x80) && !is_series_3() ) {      // if it's an SIII-only key and we are in I/II/IIX mode, don't send it
    return;
  } else {
    send_cmi_char( c );                     // handles flickering status LED and uppercasing if not in SIII mode  
    last_cmi_key = c;
    keyTimeElapsed = 0;                     // start timer for delay-until-repeat
  }
}


#define MAX_GPAD_X    0x3ff
#define MAX_GPAD_Y    0x3ff

short     gpad_x;                           // G-Pad (Series III) cursor position
short     gpad_y;

short     last_gpad_x;                      // previous G-Pad (Series III) cursor position
short     last_gpad_y;                      //  (used to prevent re-sending same position over & over)

bool      show_cursor_gpad = true;
  
bool      mouse_left_button;

/*
  Each mouse movement results in 6 bytes being sent.
  At 9,600 bps, that takes ~1ms/byte
*/

void Send_GPad_Packet( bool keysOnly ) {
  unsigned char b;
  uint8_t mods = lastMods1;

  if( is_series_3() ) {                     // only send gpad messages to Series III
    digitalWrite( ACT_LED, false );         // make LED flicker, will be turned back on in main loop
    loopcnt = 3500;  
    
    CMI_SERIAL.write( 0x80 );               // control byte

    b = 0xe0;                               // 111p tsc0: p = pen on pad
                                            //            t = touch
                                            //            s = shift key down (keyboard i/f adds)
                                            //            c = ctl key down (keyboard i/f adds)

    if( mods & USB_MOD_KEY_SHIFT )
      b |= 0x04;

    if( mods & USB_MOD_KEY_CONTROL )
      b |= 0x02;

    if( !keysOnly ) {
      if ( mouse_left_button )
        b |= 0x08;                            // touch
    
      if ( show_cursor_gpad )
        b |= 0x10;                            // pen on pad
    }
  
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

  Send_GPad_Packet( false );                // get cursor on-screen and centered
}


void cmi_gpad_move( int8_t dx, int8_t dy ) {

  gpad_x += dx;

  if (gpad_x < 0)
    gpad_x = 0;
  else if (gpad_x > MAX_GPAD_X)
    gpad_x = MAX_GPAD_X;
  
  gpad_y -= dy;                             // G-Pad Y axis is reversed

  if (gpad_y < 0)
    gpad_y = 0;
  else if (gpad_y > MAX_GPAD_Y)
    gpad_y = MAX_GPAD_Y;


  Send_GPad_Packet( false );                 
}


/* ---------------------------------------------------------------------------------------
    midi

    CMI Series I/II/IIX use a proprietary protocol between the music keyboard and the CMI.
    Each event message is 3 bytes long:

    Control 1:    0xD5  0x80  val (80-FF, 80 is lowest)
    Control 2:    0xD5  0x81  val
    Control 3:    0xD5  0x82  val

    Switch 1:     0xD3  0x80  val (80 = UP, FF = DOWN)
    Switch 2:     0xD3  0x81  val

    Music Keys
    ----------

    Lowest note on master KB is F0.
    16 velocity values, from 0xF0 (fastest) to 0xFF (slowest)

    F0            Note on: 0xC1 0x80  vel     /       Note off: 0xC0 0x80 0x80
    F#0                    0xC1 0x81  vel     /                 0xC0 0x81 0x80
    G0                     0xC1 0x82  vel     /                 0xC0 0x82 0x80
    ...

    PITCHBEND controller is mappted to CMI Control 1

    
    XXX TODO:

    - change midi channel from keypad
    - other continuous controllers
    - channelizer for SIII
 */

#define CMI_F0_NOTE_VAL         0x80                                        // note F0 on master keyboard
#define MIDI_F0_NOTE_VAL        0x11
#define CMI_NOTE_DIFF           ( CMI_F0_NOTE_VAL - MIDI_F0_NOTE_VAL )


void send_cmi_music_kb( uint8_t c ) {
  digitalWrite( ACT_LED, false );         // make LED flicker, will be turned back on in main loop
  loopcnt = 10000;  
  
  CMI_SERIAL.write( c );
    
  //Serial.print("M: ");
  //Serial.println( c, HEX );
}


void send_cmi_control_slider_val( char cc, char val ) {
  send_cmi_music_kb( 0xD5 );  
  switch( cc ) {
    case 0x00:  send_cmi_music_kb( 0x80 );      break;            
    case 0x01:  send_cmi_music_kb( 0x81 );      break;
    case 0x02:  send_cmi_music_kb( 0x82 );      break;
    case 0x03:  send_cmi_music_kb( 0x83 );      break;
    case 0x04:  send_cmi_music_kb( 0x84 );      break;
    case 0x05:  send_cmi_music_kb( 0x85 );      break;
  }
  send_cmi_music_kb( map( val, 0x00, 0x7F, 0x80, 0xFF ) );
}


              
void send_cmi_control_switch_val( char cc, char val ) {
  send_cmi_music_kb( 0xD3 ); 
  switch( cc ) {
    case 0x00:  send_cmi_music_kb( 0x80 );      break;    
    case 0x01:  send_cmi_music_kb( 0x81 );      break;      
    case 0x02:  send_cmi_music_kb( 0x82 );      break;
    case 0x03:  send_cmi_music_kb( 0x83 );      break;
    case 0x04:  send_cmi_music_kb( 0x84 );      break;
  }
  send_cmi_music_kb( (val<64) ? 0x80 : 0xff );
}


void handle_midi() {
  char c;
  
  if( is_series_3() ) {                                                     // SIII just takes MIDI from music kb
    if( MIDI_SERIAL.available() ) {                                         // XXX TODO: Merge for SIII!
      c = MIDI_SERIAL.read();
      //Serial.println( c, HEX );
      //MIDI_SERIAL.write( MIDI_SERIAL.read() );
      MIDI_SERIAL.write( c );
    }
    
    if( CMI_MIDI_KB_SERIAL.available() ) {
      c = CMI_MIDI_KB_SERIAL.read();
      //Serial.println( c, HEX );
      //MIDI_SERIAL.write( MIDI_SERIAL.read() );
      MIDI_SERIAL.write( c );
    }
  }
  else {
    int note, velocity, channel;
    byte type;
    
    if (MIDI.read()) {                                                      // Is there a MIDI message incoming ?
      type = MIDI.getType();
      channel = MIDI.getChannel();

      if( channel == MIDI_CHANNEL ) {
        switch (type) {
          
          case midi::NoteOn:
            note = MIDI.getData1();
            velocity = MIDI.getData2();
          
            if (velocity > 0) {
              //Serial.println(String("Note On:  ch=") + channel + ", note=" + note + ", velocity=" + velocity);
  
              send_cmi_music_kb( 0xC1 );                                     // note on
              send_cmi_music_kb( note + CMI_NOTE_DIFF );                     // convert MIDI note to CMI note
              send_cmi_music_kb( map( velocity, 0x00, 0x7f, 0xFF, 0xF0 ) );  // CMI has 16 velocity values, reversed sense
              
            } else {
              //Serial.println(String("Note Off: ch=") + channel + ", note=" + note);
  
              send_cmi_music_kb( 0xC0 );                                     // note off
              send_cmi_music_kb( note + CMI_NOTE_DIFF );                     // convert MIDI note to CMI note
              send_cmi_music_kb( 0x80 );                                     // always sent on note off
            
            }
            break;
        
          case midi::NoteOff:
            note = MIDI.getData1();
            velocity = MIDI.getData2();
      
            send_cmi_music_kb( 0xC0 );                                       // note off
            send_cmi_music_kb( note + CMI_NOTE_DIFF );                       // convert MIDI note to CMI note
            send_cmi_music_kb( 0x80 );                                       // always sent on note off
            break;
  
          case midi::PitchBend:  
            send_cmi_control_slider_val( 0x00, MIDI.getData2() );                               // Pitchbend -> Control Slider 1
            break;

          case midi::ControlChange:  
            switch( MIDI.getData1() ) {                                                         // which CC?
              
              // remaining 5 CMI Control slider inputs are mapped to MIDI CC #75-79 (Generic "Sound Controller #6 - 10)
              
              case 75:  send_cmi_control_slider_val( 0x01, MIDI.getData2() );   break;
              case 76:  send_cmi_control_slider_val( 0x02, MIDI.getData2() );   break;
              case 77:  send_cmi_control_slider_val( 0x03, MIDI.getData2() );   break;
              case 78:  send_cmi_control_slider_val( 0x04, MIDI.getData2() );   break;
              case 79:  send_cmi_control_slider_val( 0x05, MIDI.getData2() );   break;                

              // first 4 of 5 CMI Switch inputs are mapped to MIDI CC #80 - 83 (Generic On/Off Switch, val 0-63 = OFF, 64-127 = ON)
              //         last CMI Switch input is mapped to   MIDI CC #16 (General Purpose, we adopt the convention of val = 0-63 = OFF< 64-127 = ON)

              case 80:  send_cmi_control_switch_val( 0x00, MIDI.getData2() );   break;
              case 81:  send_cmi_control_switch_val( 0x01, MIDI.getData2() );   break;
              case 82:  send_cmi_control_switch_val( 0x02, MIDI.getData2() );   break;
              case 83:  send_cmi_control_switch_val( 0x03, MIDI.getData2() );   break;
              case 16:  send_cmi_control_switch_val( 0x04, MIDI.getData2() );   break;
            }
          break;
          
          default:  

            Serial.print("DEF: ");
            Serial.print( type, HEX );           
            Serial.print(" ");
            Serial.print( MIDI.getData1(), HEX );
            Serial.print(" ");
            Serial.println( MIDI.getData2(), HEX );
            
            break;
        }
      }
    }
  }
}


/* ---------------------------------------------------------------------------------------
    setup & loop
 */

void setup()
{
  // ===============================
  // Set up hardware
  
  pinMode( MODE_SEL, INPUT_PULLUP );
  
  pinMode( R_LED, OUTPUT );
  pinMode( G_LED, OUTPUT );

  led_red( 0 );
  led_green ( 1 );              // power on!

  Serial.begin( 115200 );       // go ahead and do this, we'll take advantage of the delays for the LED version blink

  delay( 500 );
  led_blink_version( FIRMWARE_VERSION );
  
  // ===============================
  // Set up console (debug) serial
    
  delay( 250 );                                         // wait for serial, but don't block if it's not connected
  Serial.println("Welcome to USB Keyboard & Mouse adapter for CMI");

  Serial.print("Series ");
  Serial.print( is_series_3() ? "III" : "I / II / IIX" );
  Serial.println(" mode selected.");
    
  Serial.println(sizeof(USBHub), DEC);

  // ===============================
  // Set up alphanumeric LED display
  
  if( init_led_display() ) {
    led_display_string((char*)FIRMWARE_VERSION_TEXT);
    delay( 800 );
    
    if( is_series_3() )
      led_display_string((char*)" SERIES III ");
    else
      led_display_string((char*)" -POWER ON- ");
  }

  led_green( 1 );                                       // back to green (power on)
  
  // ===============================
  // Set up CMI comms
  
  CMI_SERIAL.begin( 9600 );                             // init output port   
  KEYBD_SERIAL.begin( 9600 );                           // init legacy keyboard port   

  init_gpad();

  // ===============================
  // Set up USB

  myusb.begin();

  delay( 100 );
  
  keyboard1.attachPress(OnPress);
  keyboard1.attachRelease(OnRelease);

  // ===============================
  // Set up MIDI
  
  MIDI.begin( MIDI_CHANNEL );
  MIDI_SERIAL.begin( 31250, SERIAL_8N1_TXINV );         // our hardware is inverted on the transmit side only!

  //CMI_MIDI_KB_SERIAL.begin( MIDI_CHANNEL );
  CMI_MIDI_KB_SERIAL.begin( 31250 );

  // ===============================
  // Set up keypad I/Os

  init_keypad();
}  

#define     kPeriodicModKeyUpdate         20

elapsedMillis em;                                       // timer to send shift/ctl g-pad packet every 100ms
    
void loop()
{  
  // ===============================
  // Update Activity LED
  
  if( loopcnt )
    loopcnt--;
  else
    digitalWrite( ACT_LED, true );

  // ===============================
  // Check for characters received from the CMI

  if( CMI_SERIAL.available() ) {
    char c = CMI_SERIAL.read();

    /*
    Serial.print( "LED: ");
    Serial.print( c );
    Serial.print( " " );
    Serial.println( c, HEX );
    */
        
    putc_led_display( c );                              // XXX -- something timing sensitive here. if i send to the keyboard first,
    KEYBD_SERIAL.write( c );                            //        the second char on the legacy LED display is incorrect.
  }

  // ===============================
  // Check for characters received from the legacy keyboard

  if( KEYBD_SERIAL.available() ) {
    char c = KEYBD_SERIAL.read();
    CMI_SERIAL.write( c );

    //Serial.print("K: ");                                      // for figuring out what the music KB sends
    //Serial.println(c, HEX);
  }
  
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

  // ===============================
  // Check for Shift or Control down, send so on-screen SIII UX hints can update

  newMods1 = keyboard1.getModifiers();                          // XXX JOE -- modified class (below) keeps modifiers updated

  if( (lastMods1 ^ newMods1) && !lastMods1 ) {                  // edge(s), and all UP?
    modSentCount = kUpModSendCount;
  }

  lastMods1 = newMods1;
  
  // for an edge where modifier keys are DOWN, packets are sent as long as keys are down, at kPeriodicModKeyUpdate rate
  // for an edge where modifier keys are ALL UP, packets are sent until modSentCount == 0
  
  if( lastMods1 ) {                                             // IF a mod key is down
    if( em >= kPeriodicModKeyUpdate ) {                         // AND it has been at least kPeriodicMetaKeyUpdate ms since last G-Pad packet
      em = 0;                                                   // zero timer
      Send_GPad_Packet( true );                                 // and send it!
    }
  } else {
    if( em >= kPeriodicModKeyUpdate ) {
      if( modSentCount ) {
        em = 0;
        Send_GPad_Packet( true );                               // get on-screen UX updated quick
        modSentCount--;                                         // only send kUpModSendCount of these, enough to update the UX
      }
    }
  }

  // ===============================
  // Handle the mouse / on-screen cursor
  
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

  // ===============================
  // Give MIDI time

  handle_midi();

  // ===============================
  // Scan the keypad

  scan_keypad();
}



/* ---------------------------------------------------------------------------------------
    DL1416-style display
    Connected via 23017 i2c port expander
 */

#include <i2c_t3.h>

#define IO_EXP_RST      12    // pull low to reset, wait, then let high

#define I2C_SDA_PIN     38    // SDA1 / SCL1
#define I2C_SCL_PIN     37

#define IO_EXP_ADDR     0x20  // all 3 external addr lines tied low

// ------------- standard output routines

bool init_led_display_DL1416();
void put_led_char_DL1416( int pos, char c );

/*
  23017 i2c port expander -> DL1416 LED display pin mapping

  GPA0      LED_A0
  GPA1      LED_D6
  GPA2      LED_D3
  GPA3      LED_D2
  GPA4      LED_D1
  GPA5      LED_D0
  GPA6      LED_D4
  GPA7      LED_D5

  GPB0      n/a
  GPB1      n/a
  GPB2      /LED_3_CS
  GPB3      /LED_WR
  GPB4      /LED_CU
  GPB5      /LED_2_CS
  GPB6      /LED_1_CS
  GPB7      LED_A1
*/


/* ---------------------------------------------
   MCP23017 i2c port expander registers
   
   IOCON.BANK = 0 (default)
*/

typedef unsigned char uchar;

void exp_wr( uchar addr, uchar val );
uchar exp_rd( uchar addr );

bool expander_found = false;      // used to bypass i2c operations if we don't find the port expander at boot

#define IODIRA          0x00      // IO Direction (1 = input, 0xff on reset)
#define IODIRB          0x01
#define IOPOLA          0x02      // IO Polarity (1 = invert, 0x00 on reset)
#define IOPOLB          0x03
#define GPINTENA        0x04      // 1 = enable interrupt on change
#define GPINTENB        0x05
#define DEFVALA         0x06      // default compare values for interrupt on change, 0xoo on reset
#define DEFVALB         0x07
#define INTCONA         0x08      // 1 = pin compared to DEFVAL, 0 = pin compare to previous pin value
#define INTCONB         0x09
#define IOCON           0x0a      // see bit definitions below
//#define IOCON           0x0b    // duplicate
#define GPPUA           0x0c      // 1 = pin pullup enabled, 0x00 on reset
#define GPPUB           0x0d
#define INTFA           0x0e      // interrupt flags, 1 = associated pin caused interrupt
#define INTFB           0x0f
#define INTCAPA         0x10      // interrupt capture, holds GPIO port values when interrupt occurred
#define INTCAPB         0x11
#define GPIOA           0x12      // read: value on port, write: modify OLAT (output latch) register
#define GPIOB           0x13
#define OLATA           0x14      // read: value in OLAT, write: modify OLAT
#define OLATB           0x15

// IOCON bit values

#define IOCON_BANK      0x80      // 1: regs in different banks, 0: regs in same bank (addrs are seq, default)
#define IOCON_MIRROR    0x40      // 1: 2 INT pins are OR'ed, 0: 2 INT pins are independent (default)
#define IOCON_SEQOP     0x20      // 1: sequential ops disabled, addr ptr does NOT increment, 0: addr ptr increments (default)
#define IOCON_ODR       0x04      // 1: INT pin is Open Drain, 0: INT pin is active drive (default)
#define IOCON_INTPOL    0x02      // 1: INT pin is active hi, 0: INT pin is active lo (default)
#define IOCON_INTCC     0x01      // 1: reading INTCAP clears interrupt, 0: reading GPIO reg clears interrupt (default)


void exp_wr( uchar addr, uchar val ) {
  byte err;
  
  Wire1.beginTransmission( IO_EXP_ADDR );
  Wire1.write( addr );
  Wire1.write( val );
  err = Wire1.endTransmission();

  if( err == 0 ) {
    expander_found = true;        // excellent, we have the i2c expander!
  }
}

uchar exp_rd( uchar addr ) {
  uchar r;
  
  Wire1.beginTransmission( IO_EXP_ADDR );
  Wire1.write( addr );
  Wire1.endTransmission();  

  Wire1.requestFrom( IO_EXP_ADDR, 1 );
  r = Wire1.read();
  
  return r;
}

/*  Take in a byte in normal bit order, return a byte with data bits swizzled as hardware is connected.

       0  0
       D6 1
       D3 2
       D2 3
       D1 4
       D0 5
       D4 6
       D5 7
*/

char led_data_swizzle( char c ) {
  char r = 0;

  r |=  (c & 0x01) << 5;            // 0
  r |=  ((c & 0x02) >> 1) << 4;     // 1
  r |=  ((c & 0x04) >> 2) << 3;     // 2
  r |=  ((c & 0x08) >> 3) << 2;     // 3
  r |=  ((c & 0x10) >> 4) << 6;     // 4
  r |=  ((c & 0x20) >> 5) << 7;     // 5
  r |=  ((c & 0x40) >> 6) << 1;     // 6

  return r;
}


/*  ---------------------------
    Keypad

    Keypad rows are driven with LED Data lines:

    D4 (GPA6) -> Row 0
    D0 (GPA5) -> Row 1
    D1 (GPA4) -> Row 2
    D2 (GPA3) -> Row 3

    2 column bits are read back on the i2c port expander.
    The other 2 column bits are read back on pin 8 () and pin 5 () of the expansion connector.

    Col 0 -> i2c GPB1
    Col 1 -> i2c GPB0
    Col 2 -> exp pin 5 (digital pin 7)
    Col 3 -> exp pin 8 (digital pin 11)
*/

#define ROW_0_LOW     0b10111111      // GPA6
#define ROW_1_LOW     0b11011111      // GPA5
#define ROW_2_LOW     0b11101111      // GPA4
#define ROW_3_LOW     0b11110111      // GPA3

#define ROW_LOOPS     5000            // still very responsive, but enough time for weak pullups in the 23017 & to debounce

int drive_count_loops;
int cur_row;                          // row we are scanning now. goes from 0->3->0

char matrix_cur[4];                   // current state, only low 4 bits of each byte are used
char matrix_last[4];                  // last state, use xor to figure out if any changed

// key map tables, [row][col]

char keymap[4][4] = { { '1', '4', '7', 0x05 },          // '*' sends 0x05
                      { '2', '5', '8', '0'  },
                      { '3', '6', '9', 0x06 },          // '#' sends 0x06
                      { 'A', 'B', 'C', 'D'  } };


// drive one row low. -1 = all high.

void drive_row( int r ) {
  switch( r ) {
    case 0:   exp_wr( GPIOA, ROW_0_LOW );     break;
    case 1:   exp_wr( GPIOA, ROW_1_LOW );     break;
    case 2:   exp_wr( GPIOA, ROW_2_LOW );     break;
    case 3:   exp_wr( GPIOA, ROW_3_LOW );     break;
    default:  exp_wr( GPIOA, 0xff );          break;
  }
}

// return true (read a 1) or false (read a 0)

bool read_col( int c ) {
  bool r = false;
  
  switch( c ) {
    case 0:     r = (exp_rd( GPIOB ) & 0b00000010);     break;
    case 1:     r = (exp_rd( GPIOB ) & 0b00000001);     break;
    case 2:     r = digitalRead( 7 );                   break;
    case 3:     r = digitalRead( 11 );                  break;
  }

  return r;
}


void keypad_col_pullups_enable( bool en ) {
  if( en ) {
    exp_wr( GPPUB,   0x03 );           // port B[1:0] INPUTS have pullups ENABLED
    pinMode( 7,   INPUT_PULLUP );
    pinMode( 11,  INPUT_PULLUP );
  } else {
    exp_wr( GPPUB,   0x00 );           // port B[1:0] INPUTS have pullups ENABLED
    pinMode( 7,   INPUT );
    pinMode( 11,  INPUT );
  }
}


void init_keypad() {
  
  if( expander_found == false )        // don't try to init if no expander fitted
    return;
      
  Serial.print("initializing keypad...");
  
  keypad_col_pullups_enable( false );

  drive_count_loops = 0;
  cur_row = 0;

  matrix_cur[0] = 0;
  matrix_cur[1] = 0;
  matrix_cur[2] = 0;
  matrix_cur[3] = 0;

  matrix_last[0] = 0;
  matrix_last[1] = 0;
  matrix_last[2] = 0;
  matrix_last[3] = 0;
  
  Serial.println("done!");
}


/*  LED data lines are used as the row drivers, so some caution must be used when scanning the keypad.

    We need the column inputs to normally be just inputs (no pullups) so they don't interfere with the data lines for
     real LED display writes.
     
    But to do a scan, we need to drive a row (data line) low, the others high, wait, and sample the column inputs.
    But we don't want to just spinwait after driving the row. That would introduce latency for handling incoming real-time
     (MIDI, keyboard) events.

    So, the LED and keypad code have to coordinate. Most of the time we are not writing to the LED displays. So, the common
     path through loop() is:

    1. drive a row
    2. wait some number of loop()s
    3. sample the columns for that row
    4. update keypad state change bitmaps
    5. send chars if we just saw key downs (no key up events)
    6. next row, goto 1

    This is driven with a state machine. If an LED write happens, the state machine is reset.

    We implement this interlock/reset at the low level (here), so that otehr implementations without this hardware
     restriction don't have to worry about it.
*/

#define KEY_STATE_IDLE                0           // keypad scan state machine states
#define KEY_STATE_DRIVE_ROW           1
#define KEY_STATE_WAIT_LOOP           2
#define KEY_STATE_SAMPLE_COLS         3

int keyscan_state = KEY_STATE_IDLE;

int scan_row;                                     // row currently driving low


void keypad_send( char c ) {                      // send keypad keypresses to CMI
  //Serial.print("keypad: ");
  //Serial.println( c, HEX );
  send_cmi_char( c );                             // no autorepeat, handle flickering status LED
}


bool went_down( int row, char mask ) {
  return( ((matrix_cur[row] ^ matrix_last[row]) & mask) && (matrix_cur[row] & mask) );          // changed and is now down?
}


void scan_keypad() {
  
  if( expander_found == false )                                                                 // don't try to scan if no expander fitted
    return;
  
  switch( keyscan_state ) {
    case KEY_STATE_IDLE:          keypad_col_pullups_enable( true );                            // turn on pullups for the scan. LED code may turn off.
                                  drive_count_loops = 0;
                                  cur_row++;
                                  if( cur_row == 4 )
                                    cur_row = 0;
                                  keyscan_state = KEY_STATE_DRIVE_ROW;
                                  break;
                                  
    case KEY_STATE_DRIVE_ROW:     drive_row( cur_row ); 
                                  keyscan_state = KEY_STATE_WAIT_LOOP;
                                  break;                            
                                  
    case KEY_STATE_WAIT_LOOP:     if( drive_count_loops++ > ROW_LOOPS )
                                    keyscan_state = KEY_STATE_SAMPLE_COLS;
                                  break;
                                  
    case KEY_STATE_SAMPLE_COLS:   matrix_last[cur_row] = matrix_cur[cur_row];                   // remember what they were
    
                                  matrix_cur[cur_row] =  ( read_col( 0 ) ? (0<<0) : (1<<0) );   // see what they are now
                                  matrix_cur[cur_row] |= ( read_col( 1 ) ? (0<<1) : (1<<1) );
                                  matrix_cur[cur_row] |= ( read_col( 2 ) ? (0<<2) : (1<<2) );
                                  matrix_cur[cur_row] |= ( read_col( 3 ) ? (0<<3) : (1<<3) );
                         
                                  // send ASCII for any keys that are down. no autorepeat.
                                  
                                  if( went_down( cur_row, 0x01 ) ) keypad_send( keymap[cur_row][0] );
                                  if( went_down( cur_row, 0x02 ) ) keypad_send( keymap[cur_row][1] );
                                  if( went_down( cur_row, 0x04 ) ) keypad_send( keymap[cur_row][2] );
                                  if( went_down( cur_row, 0x08 ) ) keypad_send( keymap[cur_row][3] );
                                  
                                  keyscan_state = KEY_STATE_IDLE;
                                  break;  
  }
}



void reset_keypad_scan() {
  keypad_col_pullups_enable( false );       // pullups off so they don't interfere with LED data
  keyscan_state = KEY_STATE_IDLE;           // reset the scanner, will continue after LED write is done
}


// ------------- standard output routines

bool init_led_display_DL1416() {  
  pinMode( IO_EXP_RST, OUTPUT );            // hard reset the part
  digitalWrite( IO_EXP_RST, 0 );  
  delay( 2 );                               // let it out of reset
  digitalWrite( IO_EXP_RST, 1 );
  
  // crank up i2c
  
  pinMode( I2C_SDA_PIN, OUTPUT );           // first drive both lines low
  pinMode( I2C_SCL_PIN, OUTPUT );           // this seems to make probe work better when nothing is connected
  digitalWrite( I2C_SDA_PIN, LOW );
  digitalWrite( I2C_SCL_PIN, LOW );
  
  Wire1.begin();
  Wire1.setOpMode(I2C_OP_MODE_IMM);
  Wire1.resetBus();

  Serial.print("initializing port expander...");  
  
  exp_wr( OLATA,    0xff );                 // LED pins hi, and also probe for the i2c expander

  Serial.println("back");
  
  if( expander_found == true ) {            // do we have one?
    exp_wr( IODIRA,   0x00 );               // all port A pins OUTPUTS
  
    exp_wr( OLATB,    0xff );
    exp_wr( IODIRB,   0x03 );               // port B[7:3] are OUTPUTS, B[1:0] are INPUTS

    Serial.println(" LED/Keypad found, right on! ");
  }
  else {
    Serial.println(" No LED/Keypad found, bummer! ");
  }

  return( expander_found );                 // did we find one?
}

void put_led_char_DL1416( int pos, char c ) {
  uchar portA;
  uchar portB;
  uchar portB_CS_sel;

  if( expander_found == false )             // don't try to talk to LED displayes if no expander fitted
    return;
    
  reset_keypad_scan();                      // make sure the keypad scanner doesn't get in our way
  
  pos ^= 0x03;                              // invert address lines to LED displays, our sense of chars 0->3 is reversed
  
  // ---------------------------
  // Set up the data bus

  portA = led_data_swizzle( c );
  portA |= (pos & 0x01);                    // get LED_A0 into bit 0 of GPIOA
  exp_wr( GPIOA,    portA);                 // set up swizzled data and LED_A0

  // ---------------------------
  // figure out which chip select                                
  
  if( pos < 4 )
    portB_CS_sel = ~0x04;
  else
    if( pos < 8 )
      portB_CS_sel = ~0x20;
    else
      portB_CS_sel = ~0x40;

  // ---------------------------
  // run a bus cycle to write the swizzled char data
  
  portB = 0x7c;                             // A1 = 0, CU = 1, WR = 1, CS = 1
  portB |= ((pos & 0x02) << 6);             // get LED_A1 into bit 7 of GPIOB
  
  exp_wr( GPIOB,    portB );                // A1 = LED_A1, CU =1, WR = 1, CS = 1

  portB &= portB_CS_sel;                    // drop the right /CS
  exp_wr( GPIOB,    portB );                // A1 = LED_A1, CU = 1, WR = 1, CS = 0

  portB &= 0xf4;                            // drop /WR
  exp_wr( GPIOB,    portB );                // A1 = LED_A1, CU = 1, WR = 0, CS = 0
  
  exp_wr( GPIOB,    0xff );                 // A1 = 1, CU = 1, WR = 1, CS = 1 
}



/* ---------------------------------------------------------------------------------------
    HDSP-21xx-style display
    Connected via GPIOs, all driven from Teensy -> display, so no 5V tolerance issues.

    Display is a long shift register.
    Character data from font in sketch (font5x7.h) is loaded to display desired characters.
    
 */

#include "font5x7.h"                           // font definition

// Pins used for the LED display

#define HDSP_LED_DATA                    8     // connects to the display's data in
#define HDSP_LED_REGSEL                 11     // the display's register select pin 
#define HDSP_LED_CLK                    12     // the display's clock pin
#define HDSP_LED_ENABLE                 37     // the display's chip enable pin
#define HDSP_LED_RST                    38     // the display's reset pin        \ tied together
#define HDSP_LED_DETECT_LOOPBACK         7     //                                /   if fitted

#define LED_CHARLEN                     16     // number of characters in the display

void hdsp_init();                                           // init an 8-digit HDSP LED display
void hdsp_putchar( int pos, char c );                       // display ASCII char c at pos, call hdsp_flush when ready to send to display
void hdsp_flush();                                          // copy hdsp_bitmap to the display

void hdsp_set_brightness( char b );                         // 0 (off) -> 15 (brightest)

void hdsp_write_ctl( int reg, char val );                   // write to one of the two HDSP control regs

char hdsp_buf[LED_CHARLEN];                                 // ASCII values on display
char hdsp_bitmap[LED_CHARLEN*5];                            // each digit is 5 bytes

bool hdsp_found = false;                                    // assume we don't have it fitted

// ------------- standard output routines

bool init_led_display_HDSP21XX();                           // probe for display, return true if found
void put_led_char_HDSP21XX( int pos, char c );              // put a char on the display


void hdsp_init() {  
  // init i/o pins
  pinMode( HDSP_LED_DATA,    OUTPUT );
  pinMode( HDSP_LED_REGSEL,  OUTPUT );
  pinMode( HDSP_LED_CLK,     OUTPUT );
  pinMode( HDSP_LED_ENABLE,  OUTPUT );
  pinMode( HDSP_LED_RST,     OUTPUT );

  // reset the display
  digitalWrite( HDSP_LED_RST, LOW );
  delay( 10 );
  digitalWrite( HDSP_LED_RST, HIGH );

  hdsp_flush();                                             // init dot reg, all 0 (blank display)

  hdsp_write_ctl( 1, 0x01 );                                // word 1: OSC freq 1, Dout holds D7
  hdsp_write_ctl( 0, 0x7f );                                // word 0: no sleep, max current, max brightness
}

void hdsp_putchar( int pos, char c ) {
  int bm_pos;
  
  bm_pos = (pos * 5);                                       // each char is 5 bytes, index to the first for this char
  hdsp_buf[pos] = c;                                        // ASCII shadows

  for( int x = 0; x != 5; x++ )
    hdsp_bitmap[bm_pos+x] = pgm_read_byte(&Font5x7[((c-0x20)*5)+x]);

  hdsp_flush();
}


void hdsp_flush() {
  digitalWrite( HDSP_LED_REGSEL, LOW );                     // RS = 0 -> dot reg
  digitalWrite( HDSP_LED_ENABLE, LOW );                     // drop CS

  for( int x = 0; x < (LED_CHARLEN*5); x++ )
    shiftOut( HDSP_LED_DATA, HDSP_LED_CLK, MSBFIRST, hdsp_bitmap[x] );

  digitalWrite( HDSP_LED_ENABLE, HIGH );                    // done
}


void hdsp_set_brightness( char b ) {
  hdsp_write_ctl( 0, (0x70+b) );                            // brightness 0-15, in low nybble 
}


/*
 *  The display has 2 independent 7-bit control registers.
 *  Bit 7 in the written value selects one of the 2 words.
 *  
 *  Control word 0 (b7 = 0) controls PWM brightness, peak pixel current, and sleep mode
 *  Control work 1 (b7 = 1) controls serial in/out mode and ext osc prescaler
 */
void hdsp_write_ctl( int reg, char val ) {
  if(reg)
    val |= 0x80;                                            // register word 0/1 select is b7
  
  digitalWrite( HDSP_LED_REGSEL, HIGH );                    // RS = 1 -> ctl regs
  digitalWrite( HDSP_LED_ENABLE, LOW );                     // drop CS
  shiftOut( HDSP_LED_DATA, HDSP_LED_CLK, MSBFIRST, val );   // shift out the bits
  shiftOut( HDSP_LED_DATA, HDSP_LED_CLK, MSBFIRST, val );   // shift out the bits again (2 displays)
  digitalWrite( HDSP_LED_ENABLE, HIGH );                    // done
}


// ------------- standard output routines

bool init_led_display_HDSP21XX() {

  pinMode( HDSP_LED_DETECT_LOOPBACK, INPUT );
  pinMode( HDSP_LED_RST,             OUTPUT );

  digitalWrite( HDSP_LED_RST, 1 );
  if( digitalRead( HDSP_LED_DETECT_LOOPBACK ) != 1 ) {      // LOOPBACK following RST?
    return false;
  }

  digitalWrite( HDSP_LED_RST, 0 );
  if( digitalRead( HDSP_LED_DETECT_LOOPBACK ) != 0 ) {      // LOOPBACK following RST?
    return false;
  }

  // OK, we are installed!

  hdsp_found = true;

  Serial.println(" HDSP Mini LED display found, good times! ");
  
  hdsp_init();                                              // initialize display

  return( true );
}

void put_led_char_HDSP21XX( int pos, char c ) {
  if( hdsp_found )
    hdsp_putchar( pos, c );
}



/* ---------------------------------------------------------------------------------------
    Alphanumeric LED Displays

    2 Display types are currently supported:

    1. 3x DL1416 4-character intelligent LED displays
       These are the ones used in the CMI music keyboard.
       These have character ROMs, and only need to be given ASCII character codes.

    2. 2x HDSP-21xx 8-character 5x7 matrix LED displays
       These are smaller, more modern, and more easily sourced displays.
       With 2 of them chained together, 16 characters are available, but we only use the middle 12.
       These displays are just dumb bitmaps, a font is in the font5x7.h file. 
       This can be edited to customize the character appearance.

    At boot we probe for each, and use one if detected.
    Detection of the HDSP-21xx displays is done by checking if digital pins 38 and 7 are connected.
    Detection of the DL1416 displays is done by detecting a successful i2c transaction to the port expander.

    The main code receives characters from the CMI and uses these APIs to control the displays:

          bool init_led_display();
          void blank_led_display();
          void led_display_string( char *s );

    Each possible display type implements this API:

          void put_led_char_TYPE( char c );

    Where TYPE is the specific type:

          void put_led_char_DL1416( char c );
          void put_led_char_HDSP21XX( char c );
     
 */

int curpos = 0;

bool init_led_display(){

  if( init_led_display_HDSP21XX() == true )               // Look for the HDSP first, its probe is much faster
    return true;

  if( init_led_display_DL1416() == true )
    return true;

  return false;
}

void putc_led_display( char c ) {
  if( isprint( c ) ) {
    if( curpos == 12 )
      curpos = 0;

    if( hdsp_found )
      put_led_char_HDSP21XX( curpos+2, c );               // hack to center 12 chars in 16 char display

    if( expander_found )
      put_led_char_DL1416( curpos, c );
      
    curpos++;
  } else
    if( (c == 0x0a) || (c == 0x0d) ) {
      curpos = 0;
    }

  //Serial.println(curpos);
}

void blank_led_display() {
  putc_led_display( 0x0d );
  for( int xxx = 0; xxx != 12; xxx++ )
    putc_led_display( ' ' );  
}

void led_display_string( char *s ) {
  blank_led_display();
  for( int xxx = 0; xxx != 12; xxx++ ) {
    if( s[xxx] )
      putc_led_display( s[xxx] );
    else
      break;
  }
}




/* ---------------------------------------------------------------------------------------
  Gross but simple. 
  We needed to modify the KeyboardController class to let us know when Shift and/or Control go up & down,
    even if no other key is pressed.
 */
 

/* USB EHCI Host for Teensy 3.6
 * Copyright 2017 Paul Stoffregen (paul@pjrc.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//#include <Arduino.h>
//#include "USBHost_t36.h"  // Read this header first for key info
#include "keylayouts.h"   // from Teensyduino core library

typedef struct {
  KEYCODE_TYPE code;
  uint8_t    ascii;
} keycode_extra_t;

typedef struct {
  KEYCODE_TYPE code;
  KEYCODE_TYPE codeNumlockOff;
  uint8_t charNumlockOn;    // We will assume when num lock is on we have all characters...
} keycode_numlock_t;

typedef struct {
  uint16_t  idVendor;   // vendor id of keyboard
  uint16_t  idProduct;    // product id - 0 implies all of the ones from vendor; 
} keyboard_force_boot_protocol_t; // list of products to force into boot protocol

#ifdef M
#undef M
#endif
#define M(n) ((n) & KEYCODE_MASK)

static const keycode_extra_t keycode_extras[] = {
  {M(KEY_ENTER), '\n'},
  {M(KEY_ESC), 0x1b},
  {M(KEY_TAB), 0x9 },
  {M(KEY_UP), KEYD_UP },
  {M(KEY_DOWN), KEYD_DOWN },
  {M(KEY_LEFT), KEYD_LEFT },
  {M(KEY_RIGHT), KEYD_RIGHT },
  {M(KEY_INSERT), KEYD_INSERT },
  {M(KEY_DELETE), KEYD_DELETE }, 
  {M(KEY_PAGE_UP), KEYD_PAGE_UP },
  {M(KEY_PAGE_DOWN), KEYD_PAGE_DOWN }, 
  {M(KEY_HOME), KEYD_HOME },
  {M(KEY_END), KEYD_END },   
  {M(KEY_F1), KEYD_F1 },
  {M(KEY_F2), KEYD_F2 },     
  {M(KEY_F3), KEYD_F3 },     
  {M(KEY_F4), KEYD_F4 },     
  {M(KEY_F5), KEYD_F5 },     
  {M(KEY_F6), KEYD_F6 },     
  {M(KEY_F7), KEYD_F7 },     
  {M(KEY_F8), KEYD_F8 },     
  {M(KEY_F9), KEYD_F9  },     
  {M(KEY_F10), KEYD_F10 },    
  {M(KEY_F11), KEYD_F11 },    
  {M(KEY_F12), KEYD_F12 }    
};

// Some of these mapped to key + shift.
static const keycode_numlock_t keycode_numlock[] = {
  {M(KEYPAD_SLASH),   '/', '/'},
  {M(KEYPAD_ASTERIX), '*', '*'},
  {M(KEYPAD_MINUS), '-', '-'},
  {M(KEYPAD_PLUS),  '+', '+'},
  {M(KEYPAD_ENTER),   '\n', '\n'},
  {M(KEYPAD_1),     0x80 | M(KEY_END), '1'},
  {M(KEYPAD_2),     0x80 | M(KEY_DOWN), '2'},
  {M(KEYPAD_3),     0x80 | M(KEY_PAGE_DOWN), '3'},
  {M(KEYPAD_4),     0x80 | M(KEY_LEFT), '4'},
  {M(KEYPAD_5),     0x00, '5'},
  {M(KEYPAD_6),     0x80 | M(KEY_RIGHT), '6'},
  {M(KEYPAD_7),     0x80 | M(KEY_HOME),  '7'},
  {M(KEYPAD_8),     0x80 | M(KEY_UP),  '8'},
  {M(KEYPAD_9),     0x80 | M(KEY_PAGE_UP), '9'},
  {M(KEYPAD_0),     0x80 | M(KEY_INSERT), '0'},
  {M(KEYPAD_PERIOD),  0x80 | M(KEY_DELETE), '.'}
};

static const keyboard_force_boot_protocol_t keyboard_forceBootMode[] = {
  {0x04D9, 0}
};


#define print   USBHost::print_
#define println USBHost::println_




void KeyboardControllerExt::init()
{
  contribute_Pipes(mypipes, sizeof(mypipes)/sizeof(Pipe_t));
  contribute_Transfers(mytransfers, sizeof(mytransfers)/sizeof(Transfer_t));
  contribute_String_Buffers(mystring_bufs, sizeof(mystring_bufs)/sizeof(strbuf_t));
  driver_ready_for_device(this);
  USBHIDParser::driver_ready_for_hid_collection(this);
  BluetoothController::driver_ready_for_bluetooth(this);
  force_boot_protocol = false;  // start off assuming not
}

bool KeyboardControllerExt::claim(Device_t *dev, int type, const uint8_t *descriptors, uint32_t len)
{
  println("KeyboardController claim this=", (uint32_t)this, HEX);

  // only claim at interface level
  if (type != 1) return false;
  if (len < 9+9+7) return false;
  print_hexbytes(descriptors, len);

  uint32_t numendpoint = descriptors[4];
  if (numendpoint < 1) return false;
  if (descriptors[5] != 3) return false; // bInterfaceClass, 3 = HID
  if (descriptors[6] != 1) return false; // bInterfaceSubClass, 1 = Boot Device
  if (descriptors[7] != 1) return false; // bInterfaceProtocol, 1 = Keyboard
  if (descriptors[9] != 9) return false;
  if (descriptors[10] != 33) return false; // HID descriptor (ignored, Boot Protocol)
  if (descriptors[18] != 7) return false;
  if (descriptors[19] != 5) return false; // endpoint descriptor
  uint32_t endpoint = descriptors[20];
  println("ep = ", endpoint, HEX);
  if ((endpoint & 0xF0) != 0x80) return false; // must be IN direction
  endpoint &= 0x0F;
  if (endpoint == 0) return false;
  if (descriptors[21] != 3) return false; // must be interrupt type
  uint32_t size = descriptors[22] | (descriptors[23] << 8);
  println("packet size = ", size);
  if ((size < 8) || (size > 64)) {
    return false; // Keyboard Boot Protocol is 8 bytes, but maybe others have longer... 
  }
#ifdef USBHS_KEYBOARD_INTERVAL 
  uint32_t interval = USBHS_KEYBOARD_INTERVAL;
#else
  uint32_t interval = descriptors[24];
#endif
  println("polling interval = ", interval);
  datapipe = new_Pipe(dev, 3, endpoint, 1, 8, interval);
  datapipe->callback_function = callback;
  queue_Data_Transfer(datapipe, report, 8, this);

  // see if this device in list of devices that need to be set in
  // boot protocol mode
  bool in_forceBoot_mode_list = false;
  for (uint8_t i = 0; i < sizeof(keyboard_forceBootMode)/sizeof(keyboard_forceBootMode[0]); i++) {
    if (dev->idVendor == keyboard_forceBootMode[i].idVendor) {
      if ((dev->idProduct == keyboard_forceBootMode[i].idProduct) ||
          (keyboard_forceBootMode[i].idProduct == 0)) {
        in_forceBoot_mode_list = true;
        break;
      }
    }
  }
  if (in_forceBoot_mode_list) {
    println("SET_PROTOCOL Boot");
    mk_setup(setup, 0x21, 11, 0, 0, 0); // 11=SET_PROTOCOL  BOOT
  } else {
    mk_setup(setup, 0x21, 10, 0, 0, 0); // 10=SET_IDLE
  }
  queue_Control_Transfer(dev, &setup, NULL, this);
  control_queued = true;
  return true;
}

void KeyboardControllerExt::control(const Transfer_t *transfer)
{
  println("control callback (keyboard)");
  control_queued = false;
  print_hexbytes(transfer->buffer, transfer->length);
  // To decode hex dump to human readable HID report summary:
  //   http://eleccelerator.com/usbdescreqparser/
  uint32_t mesg = transfer->setup.word1;
  println("  mesg = ", mesg, HEX);
  if (mesg == 0x00B21 && transfer->length == 0) { // SET_PROTOCOL
    mk_setup(setup, 0x21, 10, 0, 0, 0); // 10=SET_IDLE
    control_queued = true;
    queue_Control_Transfer(device, &setup, NULL, this);
  } else if (force_boot_protocol) {
    forceBootProtocol();  // lets setup to do the boot protocol
    force_boot_protocol = false;  // turn back off
  }
}

void KeyboardControllerExt::callback(const Transfer_t *transfer)
{
  //println("KeyboardController Callback (static)");
  if (transfer->driver) {
    ((KeyboardControllerExt *)(transfer->driver))->new_data(transfer);
  }
}

void KeyboardControllerExt::forceBootProtocol()
{
  if (device && !control_queued) {
    mk_setup(setup, 0x21, 11, 0, 0, 0); // 11=SET_PROTOCOL  BOOT
    control_queued = true;
    queue_Control_Transfer(device, &setup, NULL, this);   
  } else {
    force_boot_protocol = true; // let system know we want to force this.
  }
}

void KeyboardControllerExt::disconnect()
{
  // TODO: free resources
}


// Arduino defined this static weak symbol callback, and their
// examples use it as the only way to detect new key presses,
// so unfortunate as static weak callbacks are, it probably
// needs to be supported for compatibility
extern "C" {
void __keyboardControllerExtEmptyCallback() { }
}
void keyPressed()  __attribute__ ((weak, alias("__keyboardControllerExtEmptyCallback")));
void keyReleased() __attribute__ ((weak, alias("__keyboardControllerExtEmptyCallback")));

static bool contains(uint8_t b, const uint8_t *data)
{
  if (data[2] == b || data[3] == b || data[4] == b) return true;
  if (data[5] == b || data[6] == b || data[7] == b) return true;
  return false;
}

void KeyboardControllerExt::new_data(const Transfer_t *transfer)
{
  println("KeyboardController Callback (member)");
  print("  KB Data: ");
  print_hexbytes(transfer->buffer, 8);

  modifiers = report[0];            // XXX JOE - always set the mod key bits, the CMI main loop
                                    //           code above polls them using getModifiers()
                                    //           so the SIII UI will update properly
  
  for (int i=2; i < 8; i++) {
    uint32_t key = prev_report[i];
    if (key >= 4 && !contains(key, report)) {
      key_release(prev_report[0], key);
    }
  }
  for (int i=2; i < 8; i++) {
    uint32_t key = report[i];
    if (key >= 4 && !contains(key, prev_report)) {
      key_press(report[0], key);
    }
  }
  memcpy(prev_report, report, 8);
  queue_Data_Transfer(datapipe, report, 8, this);
}


void KeyboardControllerExt::numLock(bool f) {
  if (leds_.numLock != f) {
    leds_.numLock = f;
    updateLEDS();
  }
}

void KeyboardControllerExt::capsLock(bool f) {
  if (leds_.capsLock != f) {
    leds_.capsLock = f;
    updateLEDS();
  }
}

void KeyboardControllerExt::scrollLock(bool f) {
  if (leds_.scrollLock != f) {
    leds_.scrollLock = f;
    updateLEDS();
  }
}

void KeyboardControllerExt::key_press(uint32_t mod, uint32_t key)
{
  // TODO: queue events, perform callback from Task
  println("  press, key=", key);
  modifiers = mod;
  keyOEM = key;
  keyCode = convert_to_unicode(mod, key);
  println("  unicode = ", keyCode);
  if (keyPressedFunction) {
    keyPressedFunction(keyCode);
  } else {
    keyPressed();
  }
}

void KeyboardControllerExt::key_release(uint32_t mod, uint32_t key)
{
  // TODO: queue events, perform callback from Task
  println("  release, key=", key);
  modifiers = mod;
  keyOEM = key;

  // Look for modifier keys
  if (key == M(KEY_NUM_LOCK)) {
    numLock(!leds_.numLock);
    // Lets toggle Numlock
  } else if (key == M(KEY_CAPS_LOCK)) {
    capsLock(!leds_.capsLock);

  } else if (key == M(KEY_SCROLL_LOCK)) {
    scrollLock(!leds_.scrollLock);
  } else {
    keyCode = convert_to_unicode(mod, key);
    if (keyReleasedFunction) {
      keyReleasedFunction(keyCode);
    } else {
      keyReleased();
    }
  }
}

uint16_t KeyboardControllerExt::convert_to_unicode(uint32_t mod, uint32_t key)
{
  // WIP: special keys
  // TODO: dead key sequences


  if (key & SHIFT_MASK) {
    // Many of these keys will look like they are other keys with shift mask...
    // Check for any of our mapped extra keys
    for (uint8_t i = 0; i < (sizeof(keycode_numlock)/sizeof(keycode_numlock[0])); i++) {
      if (keycode_numlock[i].code == key) {
        // See if the user is using numlock or not...
        if (leds_.numLock) {
          return keycode_numlock[i].charNumlockOn;
        } else {
          key = keycode_numlock[i].codeNumlockOff;
          if (!(key & 0x80)) return key;  // we have hard coded value
          key &= 0x7f;  // mask off the extra and break out to process as other characters...
          break;
        }
      }
    }
  }

  // Check for any of our mapped extra keys - Done early as some of these keys are 
  // above and some below the SHIFT_MASK value
  for (uint8_t i = 0; i < (sizeof(keycode_extras)/sizeof(keycode_extras[0])); i++) {
    if (keycode_extras[i].code == key) {
      return keycode_extras[i].ascii;
    }
  }

  // If we made it here without doing something then return 0;
  if (key & SHIFT_MASK) return 0;

  if ((mod & 0x02) || (mod & 0x20)) key |= SHIFT_MASK;
  if (leds_.capsLock) key ^= SHIFT_MASK;    // Caps lock will switch the Shift;
  for (int i=0; i < 96; i++) {
    if (keycodes_ascii[i] == key) {
      if ((mod & 1) || (mod & 0x10)) return (i+32) & 0x1f;  // Control key is down
      return i + 32;
    }
  }


#ifdef ISO_8859_1_A0
  for (int i=0; i < 96; i++) {
    if (keycodes_iso_8859_1[i] == key) return i + 160;
  }
#endif
  return 0;
}

void KeyboardControllerExt::LEDS(uint8_t leds) {
  println("Keyboard setLEDS ", leds, HEX);
  leds_.byte = leds;
  updateLEDS();
}

void KeyboardControllerExt::updateLEDS() {
  // Now lets tell keyboard new state.
  if (device != nullptr) {
    // Only do it this way if we are a standard USB device
    mk_setup(setup, 0x21, 9, 0x200, 0, sizeof(leds_.byte)); // hopefully this sets leds
    queue_Control_Transfer(device, &setup, &leds_.byte, this);
  } else {
    // Bluetooth, need to setup back channel to Bluetooth controller. 
  }
}

//=============================================================================
// Keyboard Extras - Combined from other object
//=============================================================================

#define TOPUSAGE_SYS_CONTROL  0x10080
#define TOPUSAGE_CONSUMER_CONTROL 0x0c0001

hidclaim_t KeyboardControllerExt::claim_collection(USBHIDParser *driver, Device_t *dev, uint32_t topusage)
{
  // Lets try to claim a few specific Keyboard related collection/reports
  //USBHDBGSerial.printf("KBH Claim %x\n", topusage);
  if ((topusage != TOPUSAGE_SYS_CONTROL) 
    && (topusage != TOPUSAGE_CONSUMER_CONTROL)
    ) return CLAIM_NO;
  // only claim from one physical device
  //USBHDBGSerial.println("KeyboardController claim collection");
  // Lets only claim if this is the same device as claimed Keyboard... 
  if (dev != device) return CLAIM_NO;
  if (mydevice != NULL && dev != mydevice) return CLAIM_NO;
  mydevice = dev;
  collections_claimed_++;
  return CLAIM_REPORT;
}

void KeyboardControllerExt::disconnect_collection(Device_t *dev)
{
  if (--collections_claimed_ == 0) {
    mydevice = NULL;
  }
}

void KeyboardControllerExt::hid_input_begin(uint32_t topusage, uint32_t type, int lgmin, int lgmax)
{
  //USBHDBGSerial.printf("KPC:hid_input_begin TUSE: %x TYPE: %x Range:%x %x\n", topusage, type, lgmin, lgmax);
  topusage_ = topusage; // remember which report we are processing. 
  hid_input_begin_ = true;
  hid_input_data_ = false;
}

void KeyboardControllerExt::hid_input_data(uint32_t usage, int32_t value)
{
  // Hack ignore 0xff00 high words as these are user values... 
  if ((usage & 0xffff0000) == 0xff000000) return; 
  //USBHDBGSerial.printf("KeyboardController: topusage= %x usage=%X, value=%d\n", topusage_, usage, value);

  // See if the value is in our keys_down list
  usage &= 0xffff;    // only keep the actual key
  if (usage == 0) return; // lets not process 0, if only 0 happens, we will handle it on the end to remove existing pressed items.

  // Remember if we have received any logical key up events.  Some keyboard appear to send them
  // others do no...
  hid_input_data_ = true;

  uint8_t key_index;
  for (key_index = 0; key_index < count_keys_down_; key_index++) {
    if (keys_down[key_index] == usage) {
      if (value) return;    // still down

      if (extrasKeyReleasedFunction) {
        extrasKeyReleasedFunction(topusage_, usage);
      }

      // Remove from list
      count_keys_down_--;
      for (;key_index < count_keys_down_; key_index++) {
        keys_down[key_index] = keys_down[key_index+1];
      }
      return;
    }
  }
  // Was not in list
  if (!value) return; // still 0
  if (extrasKeyPressedFunction) {
    extrasKeyPressedFunction(topusage_, usage);
  }
  if (count_keys_down_ < MAX_KEYS_DOWN) {
    keys_down[count_keys_down_++] = usage;
  }
}

void KeyboardControllerExt::hid_input_end()
{
  //USBHDBGSerial.println("KPC:hid_input_end");
  if (hid_input_begin_) {

    // See if we received any data from parser if not, assume all keys released... 
    if (!hid_input_data_ ) {
      if (extrasKeyReleasedFunction) {
        while (count_keys_down_) {
          count_keys_down_--;
          extrasKeyReleasedFunction(topusage_, keys_down[count_keys_down_]);
        }
      }
      count_keys_down_ = 0;
    }

    hid_input_begin_ = false;
  }   
}

bool KeyboardControllerExt::claim_bluetooth(BluetoothController *driver, uint32_t bluetooth_class, uint8_t *remoteName) 
{
  USBHDBGSerial.printf("Keyboard Controller::claim_bluetooth - Class %x\n", bluetooth_class);
  if ((((bluetooth_class & 0xff00) == 0x2500) || (((bluetooth_class & 0xff00) == 0x500))) && (bluetooth_class & 0x40)) {
    if (remoteName && (strncmp((const char *)remoteName, "PLAYSTATION(R)3", 15) == 0)) {
      USBHDBGSerial.printf("KeyboardController::claim_bluetooth Reject PS3 hack\n");
      return false;
    }
    USBHDBGSerial.printf("KeyboardController::claim_bluetooth TRUE\n");
    //btdevice = driver;
    return true;
  }
  return false;
}

bool KeyboardControllerExt::remoteNameComplete(const uint8_t *remoteName) 
{
  // Real Hack some PS3 controllers bluetoot class is keyboard... 
  if (strncmp((const char *)remoteName, "PLAYSTATION(R)3", 15) == 0) {
    USBHDBGSerial.printf("  KeyboardController::remoteNameComplete %s - Oops PS3 unclaim\n", remoteName);
    return false;
  }
  return true;
}




bool KeyboardControllerExt::process_bluetooth_HID_data(const uint8_t *data, uint16_t length) 
{
  // Example DATA from bluetooth keyboard:
  //                  0  1 2 3 4 5  6 7  8 910 1 2 3 4 5 6 7
  //                           LEN         D
  //BT rx2_data(18): 48 20 e 0 a 0 70 0 a1 1 2 0 0 0 0 0 0 0 
  //BT rx2_data(18): 48 20 e 0 a 0 70 0 a1 1 2 0 4 0 0 0 0 0 
  //BT rx2_data(18): 48 20 e 0 a 0 70 0 a1 1 2 0 0 0 0 0 0 0 
  // So Len=9 passed in data starting at report ID=1... 
  USBHDBGSerial.printf("KeyboardController::process_bluetooth_HID_data\n");
  if (data[0] != 1) return false;
  print("  KB Data: ");
  print_hexbytes(data, length);
  for (int i=2; i < length; i++) {
    uint32_t key = prev_report[i];
    if (key >= 4 && !contains(key, report)) {
      key_release(prev_report[0], key);
    }
  }
  for (int i=2; i < 8; i++) {
    uint32_t key = data[i];
    if (key >= 4 && !contains(key, prev_report)) {
      key_press(data[1], key);
    }
  }
  // Save away the data.. But shift down one byte... Don't need the report number
  memcpy(prev_report, &data[1], 8);
  return true;
}

void KeyboardControllerExt::release_bluetooth() 
{
  //btdevice = nullptr;
}

//*****************************************************************************
// Some simple query functions depend on which interface we are using...
//*****************************************************************************

uint16_t KeyboardControllerExt::idVendor() 
{
  if (device != nullptr) return device->idVendor;
  if (mydevice != nullptr) return mydevice->idVendor;
  if (btdevice != nullptr) return btdevice->idVendor;
  return 0;
}

uint16_t KeyboardControllerExt::idProduct() 
{
  if (device != nullptr) return device->idProduct;
  if (mydevice != nullptr) return mydevice->idProduct;
  if (btdevice != nullptr) return btdevice->idProduct;
  return 0;
}

const uint8_t *KeyboardControllerExt::manufacturer()
{
  if ((device != nullptr) && (device->strbuf != nullptr)) return &device->strbuf->buffer[device->strbuf->iStrings[strbuf_t::STR_ID_MAN]];
  if ((btdevice != nullptr) && (btdevice->strbuf != nullptr)) return &btdevice->strbuf->buffer[btdevice->strbuf->iStrings[strbuf_t::STR_ID_MAN]]; 
  if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_MAN]]; 
  return nullptr;
}

const uint8_t *KeyboardControllerExt::product()
{
  if ((device != nullptr) && (device->strbuf != nullptr)) return &device->strbuf->buffer[device->strbuf->iStrings[strbuf_t::STR_ID_PROD]];
  if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_PROD]]; 
  if ((btdevice != nullptr) && (btdevice->strbuf != nullptr)) return &btdevice->strbuf->buffer[btdevice->strbuf->iStrings[strbuf_t::STR_ID_PROD]]; 
  return nullptr;
}

const uint8_t *KeyboardControllerExt::serialNumber()
{
  if ((device != nullptr) && (device->strbuf != nullptr)) return &device->strbuf->buffer[device->strbuf->iStrings[strbuf_t::STR_ID_SERIAL]];
  if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_SERIAL]]; 
  if ((btdevice != nullptr) && (btdevice->strbuf != nullptr)) return &btdevice->strbuf->buffer[btdevice->strbuf->iStrings[strbuf_t::STR_ID_SERIAL]]; 
  return nullptr;
}
