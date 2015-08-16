// Modified by Ryo Ishigaki, https://github.com/risgk/tinkerit

// Auduino, the Lo-Fi granular synthesiser
//
// by Peter Knight, Tinker.it http://tinker.it
//
// Help:      http://code.google.com/p/tinkerit/wiki/Auduino
// More help: http://groups.google.com/group/auduino
//
// Digital 3: Audio out (Digital 11 on ATmega8)
//
// Changelog:
// 19 Nov 2008: Added support for ATmega8 boards
// 21 Mar 2009: Added support for ATmega328 boards
// 7 Apr 2009: Fixed interrupt vector for ATmega328 boards
// 8 Apr 2009: Added support for ATmega1280 boards (Arduino Mega)

#include <avr/io.h>
#include <avr/interrupt.h>

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;
uint16_t grain2PhaseAcc;
uint16_t grain2PhaseInc;
uint16_t grain2Amp;
uint8_t grain2Decay;

// MIDI
#define MIDI_CH               (0)
#define SERIAL_SPEED          (38400)

#define DATA_BYTE_MAX         (0x7F)
#define STATUS_BYTE_INVALID   (0x7F)
#define DATA_BYTE_INVALID     (0x80)
#define STATUS_BYTE_MIN       (0x80)
#define NOTE_OFF              (0x80)
#define NOTE_ON               (0x90)
#define CONTROL_CHANGE        (0xB0)
#define SYSTEM_MESSAGE_MIN    (0xF0)
#define SYSTEM_EXCLUSIVE      (0xF0)
#define TIME_CODE             (0xF1)
#define SONG_POSITION         (0xF2)
#define SONG_SELECT           (0xF3)
#define TUNE_REQUEST          (0xF6)
#define EOX                   (0xF7)
#define REAL_TIME_MESSAGE_MIN (0xF8)
#define ACTIVE_SENSING        (0xFE)

#define GRAIN_FREQ_CONTROL    (16)
#define GRAIN_DECAY_CONTROL   (17)
#define GRAIN2_FREQ_CONTROL   (18)
#define GRAIN2_DECAY_CONTROL  (19)
#define ALL_NOTES_OFF         (123)

#define INLINE inline __attribute__((always_inline))

class SerialIn {
public:
  INLINE static void open() {
    UBRR0 = (1000000 / SERIAL_SPEED) - 1;
    UCSR0B = _BV(RXEN0);
  }

  INLINE static boolean available() {
    return UCSR0A & _BV(RXC0);
  }

  INLINE static int8_t read() {
    return UDR0;
  }
};

uint16_t mapPhaseInc(uint16_t input);
uint16_t mapMidi(uint16_t input);

class MIDI {
  static uint8_t m_system_exclusive;
  static uint8_t m_system_data_remaining;
  static uint8_t m_running_status;
  static uint8_t m_first_data;

public:
  INLINE static void initialize() {
    m_system_exclusive = false;
    m_system_data_remaining = 0;
    m_running_status = STATUS_BYTE_INVALID;
    m_first_data = DATA_BYTE_INVALID;
  }

  INLINE static void receive_midi_byte(uint8_t b) {
    if (is_data_byte(b)) {
      if (m_system_exclusive) {
        // do nothing
      } else if (m_system_data_remaining != 0) {
        m_system_data_remaining--;
      } else if (m_running_status == (NOTE_ON | MIDI_CH)) {
        if (!is_data_byte(m_first_data)) {
          m_first_data = b;
        } else if (b == 0) {
          note_off(m_first_data);
          m_first_data = DATA_BYTE_INVALID;
        } else {
          note_on(m_first_data);
          m_first_data = DATA_BYTE_INVALID;
        }
      } else if (m_running_status == (NOTE_OFF | MIDI_CH)) {
        if (!is_data_byte(m_first_data)) {
          m_first_data = b;
        } else {
          note_off(m_first_data);
          m_first_data = DATA_BYTE_INVALID;
        }
      } else if (m_running_status == (CONTROL_CHANGE | MIDI_CH)) {
        if (!is_data_byte(m_first_data)) {
          m_first_data = b;
        } else {
          control_change(m_first_data, b);
          m_first_data = DATA_BYTE_INVALID;
        }
      }
    } else if (is_system_message(b)) {
      switch (b) {
      case SYSTEM_EXCLUSIVE:
        m_system_exclusive = true;
        m_running_status = STATUS_BYTE_INVALID;
        break;
      case EOX:
      case TUNE_REQUEST:
      case 0xF4:
      case 0xF5:
        m_system_exclusive = false;
        m_system_data_remaining = 0;
        m_running_status = STATUS_BYTE_INVALID;
        break;
      case TIME_CODE:
      case SONG_SELECT:
        m_system_exclusive = false;
        m_system_data_remaining = 1;
        m_running_status = STATUS_BYTE_INVALID;
        break;
      case SONG_POSITION:
        m_system_exclusive = false;
        m_system_data_remaining = 2;
        m_running_status = STATUS_BYTE_INVALID;
        break;
      }
    } else if (is_status_byte(b)) {
      m_system_exclusive = false;
      m_running_status = b;
      m_first_data = DATA_BYTE_INVALID;
    }
  }

private:
  INLINE static boolean is_real_time_message(uint8_t b) {
    return b >= REAL_TIME_MESSAGE_MIN;
  }

  INLINE static boolean is_system_message(uint8_t b) {
    return b >= SYSTEM_MESSAGE_MIN;
  }

  INLINE static boolean is_status_byte(uint8_t b) {
    return b >= STATUS_BYTE_MIN;
  }

  INLINE static boolean is_data_byte(uint8_t b) {
    return b <= DATA_BYTE_MAX;
  }

  INLINE static void note_on(uint8_t note_number) {
    // Stepped mapping to MIDI notes: C, Db, D, Eb, E, F...
    syncPhaseInc = mapMidi((127 - note_number) << 3);
  }

  INLINE static void note_off(uint8_t note_number) {
  }

  INLINE static void control_change(uint8_t controller_number, uint8_t controller_value) {
    switch (controller_number) {
    case GRAIN_FREQ_CONTROL:
      grainPhaseInc  = mapPhaseInc(((127 - controller_value) << 3) + 0) / 2;
      break;
    case GRAIN_DECAY_CONTROL:
      grainDecay     = (((127 - controller_value) << 3) + 0) / 8;
      break;
    case GRAIN2_FREQ_CONTROL:
      grain2PhaseInc = mapPhaseInc(((127 - controller_value) << 3) + 4) / 2;
      break;
    case GRAIN2_DECAY_CONTROL:
      grain2Decay    = (((127 - controller_value) << 3) + 4) / 4;
      break;
    default:
      break;
    }
  }
};

uint8_t MIDI::m_system_exclusive;
uint8_t MIDI::m_system_data_remaining;
uint8_t MIDI::m_running_status;
uint8_t MIDI::m_first_data;


// Changing these will also requires rewriting audioOn()

#if defined(__AVR_ATmega8__)
//
// On old ATmega8 boards.
//    Output is on pin 11
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_PIN       11
#define PWM_VALUE     OCR2
#define PWM_INTERRUPT TIMER2_OVF_vect
#elif defined(__AVR_ATmega1280__)
//
// On the Arduino Mega
//    Output is on pin 3
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       7
#define PWM_PIN       3
#define PWM_VALUE     OCR3C
#define PWM_INTERRUPT TIMER3_OVF_vect
#else
//
// For modern ATmega168 and ATmega328 boards
//    Output is on pin 3
//
#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_INTERRUPT TIMER2_OVF_vect
#endif

// Smooth logarithmic mapping
//
uint16_t antilogTable[] = {
  64830,64132,63441,62757,62081,61413,60751,60097,59449,58809,58176,57549,56929,56316,55709,55109,
  54515,53928,53347,52773,52204,51642,51085,50535,49991,49452,48920,48393,47871,47356,46846,46341,
  45842,45348,44859,44376,43898,43425,42958,42495,42037,41584,41136,40693,40255,39821,39392,38968,
  38548,38133,37722,37316,36914,36516,36123,35734,35349,34968,34591,34219,33850,33486,33125,32768
};
uint16_t mapPhaseInc(uint16_t input) {
  return (antilogTable[input & 0x3f]) >> (input >> 6);
}

// Stepped chromatic mapping
//
uint16_t midiTable[] = {
  17,18,19,20,22,23,24,26,27,29,31,32,34,36,38,41,43,46,48,51,54,58,61,65,69,73,
  77,82,86,92,97,103,109,115,122,129,137,145,154,163,173,183,194,206,218,231,
  244,259,274,291,308,326,346,366,388,411,435,461,489,518,549,581,616,652,691,
  732,776,822,871,923,978,1036,1097,1163,1232,1305,1383,1465,1552,1644,1742,
  1845,1955,2071,2195,2325,2463,2610,2765,2930,3104,3288,3484,3691,3910,4143,
  4389,4650,4927,5220,5530,5859,6207,6577,6968,7382,7821,8286,8779,9301,9854,
  10440,11060,11718,12415,13153,13935,14764,15642,16572,17557,18601,19708,20879,
  22121,23436,24830,26306
};
uint16_t mapMidi(uint16_t input) {
  return (midiTable[(1023-input) >> 3]);
}

// Stepped Pentatonic mapping
//
uint16_t pentatonicTable[54] = {
  0,19,22,26,29,32,38,43,51,58,65,77,86,103,115,129,154,173,206,231,259,308,346,
  411,461,518,616,691,822,923,1036,1232,1383,1644,1845,2071,2463,2765,3288,
  3691,4143,4927,5530,6577,7382,8286,9854,11060,13153,14764,16572,19708,22121,26306
};

uint16_t mapPentatonic(uint16_t input) {
  uint8_t value = (1023-input) / (1024/53);
  return (pentatonicTable[value]);
}


void audioOn() {
#if defined(__AVR_ATmega8__)
  // ATmega8 has different registers
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#endif
}


void setup() {
  MIDI::initialize();
  SerialIn::open();

  syncPhaseInc   = mapMidi((127 - 60) << 3);
  grainPhaseInc  = mapPhaseInc(((127 - 64) << 3) + 0) / 2;
  grainDecay     = (((127 - 64) << 3) + 0) / 8;
  grain2PhaseInc = mapPhaseInc(((127 - 64) << 3) + 4) / 2;
  grain2Decay    = (((127 - 64) << 3) + 4) / 4;

  pinMode(PWM_PIN,OUTPUT);
  audioOn();
  pinMode(LED_PIN,OUTPUT);
}

void loop() {
  while(true) {
    // The loop is pretty simple - it just updates the parameters for the oscillators.
    //
    // Avoid using any functions that make extensive use of interrupts, or turn interrupts off.
    // They will cause clicks and poops in the audio.
    if (SerialIn::available()) {
      uint8_t b = SerialIn::read();
      MIDI::receive_midi_byte(b);
    }
  }
}

SIGNAL(PWM_INTERRUPT)
{
  uint8_t value;
  uint16_t output;

  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    // Time to start the next grain
    grainPhaseAcc = 0;
    grainAmp = 0x7fff;
    grain2PhaseAcc = 0;
    grain2Amp = 0x7fff;
    LED_PORT ^= 1 << LED_BIT; // Faster than using digitalWrite
  }
  
  // Increment the phase of the grain oscillators
  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  // Convert phase into a triangle wave
  value = (grainPhaseAcc >> 7) & 0xff;
  if (grainPhaseAcc & 0x8000) value = ~value;
  // Multiply by current grain amplitude to get sample
  output = value * (grainAmp >> 8);

  // Repeat for second grain
  value = (grain2PhaseAcc >> 7) & 0xff;
  if (grain2PhaseAcc & 0x8000) value = ~value;
  output += value * (grain2Amp >> 8);

  // Make the grain amplitudes decay by a factor every sample (exponential decay)
  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  // Scale output to the available range, clipping if necessary
  output >>= 9;
  if (output > 255) output = 255;

  // Output to PWM (this is faster than using analogWrite)  
  PWM_VALUE = output;
}
