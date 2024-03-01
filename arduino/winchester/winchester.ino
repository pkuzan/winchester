/*
    Winchester
    Power Controller Software for :
    32u4 Adafruit Itsy Bitsy 
    Windows Machine - SSR 
    Pushbutton
    Single LED 
    HW sends MIDI note-on / note-off on ch=16 note=32 to signal audio engine start and stop
    Sends MIDI program change on ch-16 program=121 to shut computer down
    Romsey OrganWorks  
    Paul Kuzan
    01/04/2024
*/

#include <MIDIUSB.h>
#include <Bounce2.h>

#define STATE_STANDBY 1
#define STATE_COMPUTER_STARTING 2
#define STATE_HW_STARTING 3
#define STATE_RUNNING 4
#define STATE_COMPUTER_STOPPRING 5

#define STATE_LED_OFF 1
#define STATE_LED_ON 2
#define STATE_LED_FLASH_SLOW 3
#define STATE_LED_FLASH_FAST 4

#define STATE_AUDIO_OFF 1
#define STATE_AUDIO_ON 2

#define SWITCH_NONE 1
#define SWITCH_PRESS_SHORT 2
#define SWITCH_PRESS_LONG 3

//Controls servo to start Mac
const int servoPin = 10;

//TIP120 output - controls GND to drawstops
const int tip120Pin = 9;

//Uses USB bus power to detect when Computer has actually started and shutdown
//Logic is inverted by opto-isolator
const int USBBusPowerPin = 21;

//Console switching power from console
//Logic is inverted by opto-isolator
const int switchingPowerPin = 15;
//OR
//Push button start
const int pushButtonPin = 18;

//Status LED on board
const int ledPin = 7;
//Status LED on pushputton
const int ledSwPin = 0;

//Controls audio - probably via a contactor or relay - last on, first off
const int audioPowerPin = 1;

//Controls aux peripherals - follows USB bus power
const int auxPowerPin = 2;

//Controls power to NUC
const int computerPowerPin = 3;

// Aux Input
const int auxInPin = 19;

//Shuts down computer by sending a MIDI Program Change 121 on Channel 16.
const byte midiChannel = 15;
const byte midiShutdownPC = 120;
//Receieves MIDI Note on / off to indicate if HW audio engine has started or stopped
const byte midiAudioEngineStateNote = 32;

//Servo servo;
Bounce pushButton = Bounce();

//Power LED flash  interval
const unsigned long onFlashInterval = 1000UL;
const unsigned long offFlashInterval = 200UL;

unsigned long previousMillis = 0;
bool ledFlashState;

volatile byte state;
volatile byte audioEngineState;
volatile byte ledState;

bool justTransitioned = false;
bool ledStateJustTransitioned = false;

//Pushbutton hold time
const unsigned long switchHoldTime = 5000UL;
unsigned long switchPressTime = 0;
byte switchState = SWITCH_NONE;
bool switchPressed = false;

void setup() {
  Serial.begin(9600);

  pinMode(pushButtonPin, INPUT_PULLUP);
  pushButton.attach(pushButtonPin);

  pinMode(auxInPin, INPUT_PULLUP);

  pinMode(USBBusPowerPin, INPUT);
  pinMode(switchingPowerPin, INPUT);

  pinMode(audioPowerPin, OUTPUT);
  pinMode(auxPowerPin, OUTPUT);
  pinMode(computerPowerPin, OUTPUT);

  pinMode(ledPin, OUTPUT);
  pinMode(ledSwPin, OUTPUT);

  pinMode(tip120Pin, OUTPUT);

  transitionTo(STATE_STANDBY);
}

void loop() {
  readSwitch();
  readMIDI();
  doStateMachine();
  doLEDStateMachine();
}

void readSwitch() {
  pushButton.update();
  if (pushButton.fell()) {
    switchPressed = true;
    switchPressTime = millis();
  } else if (pushButton.rose()) {
    if (switchPressed) {
      if ((millis() - switchPressTime) > switchHoldTime) {
        //Long press
        switchState = SWITCH_PRESS_LONG;
      } else {
        //Short press
        switchState = SWITCH_PRESS_SHORT;
      }
      switchPressed = false;
    }
  }
}

//The Main State Machine
void doStateMachine() {
  if (switchState == SWITCH_PRESS_LONG) {
    switchState = SWITCH_NONE;
    transitionTo(STATE_STANDBY);
  }

  switch (state) {
    case STATE_STANDBY:
      {
        if (justTransitioned) {
          Serial.print("Standby\n");

          transitionLEDState(STATE_LED_ON);
          switchOffComputerPower();
          switchOffAudio();
          audioEngineState = STATE_AUDIO_OFF;
          switchState = SWITCH_NONE;

          justTransitioned = false;
        }

        if (switchState == SWITCH_PRESS_SHORT) {
          Serial.print("Button pressed\n");
          switchState = SWITCH_NONE;
          switchOnComputerPower();
          transitionTo(STATE_COMPUTER_STARTING);
        }
        break;
      }

    case STATE_COMPUTER_STARTING:
      {
        if (justTransitioned) {
          Serial.print("Waiting for Computer to Start\n");

          justTransitioned = false;
        }

        if (digitalRead(USBBusPowerPin) == LOW) {
          Serial.print("USB Bus Power ON\n");

          transitionLEDState(STATE_LED_FLASH_SLOW);

          transitionTo(STATE_HW_STARTING);
        }
        break;
      }

    case STATE_HW_STARTING:
      {
        if (justTransitioned) {
          Serial.print("Waiting for Hauptwerk to Start\n");

          justTransitioned = false;
        }

        if (audioEngineState == STATE_AUDIO_ON) {
          switchOnAudio();
          transitionTo(STATE_RUNNING);
        }
        break;
      }

    case STATE_RUNNING:
      {
        if (justTransitioned) {
          Serial.print("Hauptwerk Started\n");

          transitionLEDState(STATE_LED_OFF);

          justTransitioned = false;
        }

        if (switchState == SWITCH_PRESS_SHORT) {
          Serial.print("Switching Power OFF\n");

          switchOffAudio();
          delay(2000);
          sendShutdownMidi();

          transitionTo(STATE_COMPUTER_STOPPRING);
        }
        break;
      }

    case STATE_COMPUTER_STOPPRING:
      {
        if (justTransitioned) {
          Serial.print("Computer Stopping\n");

          transitionLEDState(STATE_LED_FLASH_FAST);

          justTransitioned = false;
        }

        if (digitalRead(USBBusPowerPin) == HIGH) {
          Serial.print("Computer OFF\n");

          transitionTo(STATE_STANDBY);
        }
        break;
      }
  }
}

//The State Machine for the Power LED
void doLEDStateMachine() {
  switch (ledState) {
    case STATE_LED_OFF:
      {
        if (ledStateJustTransitioned) {
          updateLED(false);

          ledStateJustTransitioned = false;
        }

        break;
      }
    case STATE_LED_ON:
      {
        if (ledStateJustTransitioned) {
          updateLED(true);

          ledStateJustTransitioned = false;
        }

        break;
      }
    case STATE_LED_FLASH_SLOW:
      {
        if (ledStateJustTransitioned) {
          //Do nothing
          ledStateJustTransitioned = false;
        }

        doFlash(onFlashInterval);

        break;
      }
    case STATE_LED_FLASH_FAST:
      {
        if (ledStateJustTransitioned) {
          //Do nothing
          ledStateJustTransitioned = false;
        }

        doFlash(offFlashInterval);

        break;
      }
  }
}

void transitionTo(byte newState) {
  justTransitioned = true;
  state = newState;
}

void transitionLEDState(byte newLEDState) {
  ledStateJustTransitioned = true;
  ledState = newLEDState;
}

void doFlash(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    updateLED(!ledFlashState);
  }
}

//Actually turn on or off the power to led
void updateLED(bool newLEDFlashState) {
  ledFlashState = newLEDFlashState;

  if (ledFlashState) {
    digitalWrite(ledPin, HIGH);
    digitalWrite(ledSwPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
    digitalWrite(ledSwPin, LOW);
  }
}

void readMIDI() {
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
    switch (rx.header) {
      case 0:
        break;  //No pending events

      case 0x9:
        Serial.print("Note On\n");
        onNoteOn(
          rx.byte1 & 0xF,  //channel
          rx.byte2,        //pitch
          rx.byte3         //velocity
        );
        break;

      case 0x8:
        Serial.print("Note Off\n");
        onNoteOff(
          rx.byte1 & 0xF,  //channel
          rx.byte2,        //pitch
          rx.byte3         //velocity
        );
        break;
    }
  } while (rx.header != 0);
}

void onNoteOn(byte channel, byte note, byte velocity) {
  if (isMidiAudioEngineStateNote(channel, note)) {
    Serial.print("Audio Engine Started\n");
    audioEngineState = STATE_AUDIO_ON;
  }
}

void onNoteOff(byte channel, byte note, byte velocity) {
  if (isMidiAudioEngineStateNote(channel, note)) {
    Serial.print("Audio Engine Stopped\n");
    audioEngineState = STATE_AUDIO_OFF;
  }
}

bool isMidiAudioEngineStateNote(byte channel, byte note) {
  if (channel == midiChannel) {
    if (note == midiAudioEngineStateNote) {
      return true;
    }
  }
  return false;
}

void sendProgramChange(byte channel, byte program) {
  midiEventPacket_t pc = { 0x0C, 0xC0 | channel, program, 0 };
  MidiUSB.sendMIDI(pc);
  MidiUSB.flush();
}

void sendShutdownMidi() {
  sendProgramChange(midiChannel, midiShutdownPC);
}

void switchOnAudio() {
  digitalWrite(tip120Pin, HIGH);
  digitalWrite(audioPowerPin, HIGH);
}

void switchOffAudio() {
  digitalWrite(tip120Pin, LOW);
  digitalWrite(audioPowerPin, LOW);
}

void switchOnAux() {
  digitalWrite(auxPowerPin, HIGH);
}

void switchOffAux() {
  digitalWrite(auxPowerPin, LOW);
}

void switchOnComputerPower() {
  digitalWrite(computerPowerPin, HIGH);
}

void switchOffComputerPower() {
  digitalWrite(computerPowerPin, LOW);
}

