//UNTZtrument code single button select

//Included librarys, files that define standard reusable functions

#include <Wire.h>                   //Serial communication Library
#include <Adafruit_Trellis.h>
#include <Adafruit_UNTZtrument.h>
#include "MIDIUSB.h"                //Midi Library
#include <avr/pgmspace.h>           //Library for memory support

#define LED 13              // Pin for heartbeat LED (shows code is working)
#define modeSwitch 6       //Pin to read the encoder switch
#define modeSwitchPower 7  //Pin to power the encoder switch

//Standard definitions to setup an UNTZtrument
//Adafuit_Trellis is a standard name used by the Trellis library
//Hardcoded Unztrument to 4 Trellis boards, one Untztrument
Adafruit_Trellis     T[4];
Adafruit_UNTZtrument untztrument(&T[0], &T[1], &T[2], &T[3]);

//Set up the Addresses for the individual Trellis boards
//This is needed for Serial Communnication
const uint8_t        addr[] = { 0x70, 0x71, 0x72, 0x73 };

const byte    Row           = 8,               //number of buttons in each row/column
              Column        = 8;
uint8_t       bpm           = 200,              //Tempo in beats per minute, hardcoded for now
              heart         = 0;                //heartbeat counter, used for debugging 
unsigned long beatInterval  = 60000L / bpm,     // ms/beat
              prevBeatTime = 0L,
              prevReadTime = 0L;                // Keypad polling timer

//define an encoder on pins 4 and 5
//enc is defined in the untztrument library and e is a built in function
enc e(4,5);

//16 buttons per individual trellis
//T is an array defined above
#define N_BUTTONS ((sizeof(T) / sizeof(T[0])) * 16) 

//Setup a matrix in memory to store note values in
//The matrix NoteGrid will have size N_Buttons
//PROGMEM will tell the controller to save this to memory so we can save values 
//when we shut down the controller and use the same notes the next time
const uint8_t NoteGrid[N_BUTTONS] PROGMEM;

//We don't need to save the button press state at shutdown so it doesn't need the PROGMEM tag
// ButtonPressState[N_BUTTONS];
uint8_t ButtonPressState[N_BUTTONS];

//Array for column value bitmask, copied from step seq
const uint8_t PROGMEM bitmask[8] = {   1,  2,  4,  8, 16, 32, 64, 128 },
                      channel[8] = {   1,  1,  1,  1,  1,  1,  1,   1 };

void setup() {
  // put your setup code here, to run once:
  //Turn on outputs to LEDs and begin serial communcations to each Trellis
  pinMode(LED, OUTPUT);
  pinMode(modeSwitch, INPUT);
  pinMode(modeSwitchPower, OUTPUT);
  untztrument.begin(addr[0], addr[1], addr[2], addr[3]);

#ifdef ARDUINO_ARCH_SAMD
  Wire.setClock(400000L);
#endif

  untztrument.clear();
  untztrument.writeDisplay();
  memset(NoteGrid, 0, sizeof(NoteGrid));
  enc::begin();                     // Initialize all encoder pins
  e.setBounds(60 * 4, 480 * 4 + 3); // Set tempo limits
  e.setValue(bpm * 4);              // *4's for encoder detents, based on encoder hardware

  //Turn on power to the mode switch
  digitalWrite(modeSwitchPower, HIGH);
}

//Note On and off Functions, copied form UNTZtrument step seq code, defined in Library
void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();
}

void loop() {
  // put your main code here, to run repeatedly:
  boolean       switchState = digitalRead(modeSwitch);
  boolean       refresh = false;
  unsigned long t = millis();  //define t as time variable in milliseconds
  enc::poll(); // Read encoder(s)

  if((t - prevReadTime) >= 20L) {            // 20ms = min Trellis poll time
    if(untztrument.readSwitches()) {        // Button state change?
      for(uint8_t i=0; i<N_BUTTONS; i++) {  // For each button...
        
        //If the key was just pressed, turn the LED on and turn the note on
        if(untztrument.justPressed(i)) 
        {
          //if the button was not already pressed
          if (ButtonPressState[i] == 0)
          {
            //turn on the LED, send the Midi note, set the button state to on
            untztrument.setLED(i);
            noteOn(1, 64, 127);
            ButtonPressState[i] = 1;
          }
          //otherwise the button is already pressed
          else
          {
            //turn off the LED and Note and set the button state to off
             untztrument.clrLED(i);
             noteOff(1, 64, 127);
             ButtonPressState[i] = 0;
          }
          //refresh the display either way
          refresh = true;
        }
      }
    }
    
    prevReadTime = t;
    
    digitalWrite(LED, ++heart & 32); // Blink = alive
    prevBeatTime = t;
    refresh      = true;
    //bpm          = e.getValue() / 4; // Div for encoder detents
    beatInterval = 60000L / bpm;
    
  }
  
  if(refresh) untztrument.writeDisplay();

}
