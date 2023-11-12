/*
    Winchester
    Power Controller Software for :
    32u4 Adafruit Itsy Bitsy 
    Windows Machine - SSR 
    Pushbutton
    Single LED 
    HW sends MIDI note-on / note-off on ch=16 note=32 to signal audio engine start and stop
    Sends MIDI program change on ch-16 program=127 to shut computer down
    Romsey OrganWorks  
    Paul Kuzan
    12/11/2023
*/

#include <Servo.h>
#include <MIDIUSB.h>

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

//Controls servo to start Mac
const int servoPin = 10;

//Uses USB bus power to detect when Mac has actually started and shutdown
//Logic is inverted by opto-isolator
const int USBBusPowerPin = 21;

//Console switching power
//Logic is inverted by opto-isolator
const int switchingPowerPin = 15;

//Status LED on board
const int ledPin = 7;
//Status LED on pushputton
const int ledSwPin = 0;

//Controls audio - probably via a contactor or relay - last on, first off
const int audioPowerPin = 1;

//Controls aux peripherals - follows USB bus power
const int auxPowerPin = 2;

// Push button
const int pushButtonPin = 18;

// Aux Input
const int auxInPin = 19;

//Shuts down Mac by sending a MIDI Program Change 127 on Channel 16.
const byte midiChannel = 15;
const byte midiShutdownPC = 120;
//Receieves MIDI Note on / off to indicate if HW audio engine has started or stopped
const byte midiAudioEngineStateNote = 32;

Servo servo;

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

void setup() {
  Serial.begin(9600);

  initServo();

  pinMode(pushButtonPin, INPUT_PULLUP);
  pinMode(auxInPin, INPUT_PULLUP);

  pinMode(USBBusPowerPin, INPUT);
  pinMode(switchingPowerPin, INPUT);

  pinMode(audioPowerPin, OUTPUT);
  pinMode(auxPowerPin, OUTPUT);

  pinMode(ledPin, OUTPUT);
  pinMode(ledSwPin, OUTPUT);

  transitionTo(STATE_STANDBY);
}

void loop() {
  doStateMachine();
  doLEDStateMachine();
}

//The Main State Machine
void doStateMachine() {
  switch (state) {
    case STATE_STANDBY:
      {
        if (justTransitioned) {
          Serial.print("Standby\n");

          transitionLEDState(STATE_LED_ON);
          switchOffAudio();
          switchOffAux();
          audioEngineState = STATE_AUDIO_OFF;

          justTransitioned = false;
        }

        if (digitalRead(switchingPowerPin) == LOW) {
          Serial.print("Switching Power ON\n");

          switchOnAux();
          delay(30000);
          switchOnMac();

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

        readMIDI();

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

        if (digitalRead(switchingPowerPin) == HIGH) {
          Serial.print("Switching Power OFF\n");

          switchOffAudio();
          delay(10000);
          switchOffMac();

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
  midiEventPacket_t rx = MidiUSB.read();
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

void switchOnAudio() {
  digitalWrite(audioPowerPin, HIGH);
}

void switchOffAudio() {
  digitalWrite(audioPowerPin, LOW);
}

void switchOnAux() {
  digitalWrite(auxPowerPin, HIGH);
}

void switchOffAux() {
  digitalWrite(auxPowerPin, LOW);
}

void switchOnMac() {
  //Moves the servo to mechanically switch on Mac
  servo.attach(servoPin);
  servo.write(70);
  delay(200);
  servo.writeMicroseconds(1500);
  delay(200);
  servo.detach();
}

void switchOffMac() {
  sendProgramChange(midiChannel, midiShutdownPC);
}

void initServo() {
  servo.attach(servoPin);
  servo.writeMicroseconds(1500);
  delay(200);
  servo.detach();
}
