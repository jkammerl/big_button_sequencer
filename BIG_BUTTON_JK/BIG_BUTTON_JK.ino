//BIG BUTTON CODE

#define TRIGGER_LENGTH_MS 5
#define NUM_BANKS 2
#define NUM_BARS 1
#define BAR_RES 32
#define NUM_CHANNELS 6
#define MIDI_NOTE_OFFSET 50

//#define SERIAL_DEBUG

struct IOPin {
  int pin;
  unsigned long switch_state_at_time;
  bool inverse;
  bool state;
  bool just_pressed;
};

enum OUT_ID {
  OUT1 = 0,
  OUT2,
  OUT3,
  OUT4,
  OUT5,
  OUT6,
  OUT_SYNC,
  OUT_RESET,
  OUT_BB_LED,
  NUM_OUTS
};

IOPin outs[NUM_OUTS] = {
  {13, 0, false, false, false}, // OUT1
  {12, 0, false, false, false}, // OUT2
  {11, 0, false, false, false}, // OUT3
  {10, 0, false, false, false}, // OUT4
  {9, 0, false, false, false},  // OUT5
  {8, 0, false, false, false},  // OUT6
  {A0, 0, false, false, false}, // OUT_SYNC
  {2, 0, false, false, false},  // OUT_RESET
  {A3, 0, false, false, false}  // OUT_BB_LED
};

enum IN_ID {
  IN_DELETE = 0,
  IN_RAND,
  IN_FILL,
  IN_CLEAR,
  IN_BANK_SELECT,
  IN_BB,
  NUM_INS
};

IOPin ins[NUM_INS] = {
  {PD7, 0, false, false, false},  // IN_DELETE
  {PD6, 0, false, false, false},  // IN_RAND
  {PD5, 0, false, false, false},  // IN_FILL
  {PD4, 0, false, false, false},  // IN_CLEAR
  {PD3, 0, false, false, false},  // IN_BANK_SELECT
  {A5, 0, false, false, false}    // IN_BB
};

// MIDI clock has 24ticks per 4th: 1th, 2th, 4th, 8th, 16th, 32th
static const int kMidiTicksDivs[] = {96, 48, 24, 12, 6, 3};
int kNumMidiTicksDivs = sizeof(kMidiTicksDivs) / sizeof(int);
int current_tick_div = 0;

struct RandVariConfig {
  float prob;
  int tick_div;
};
RandVariConfig rand_variations[NUM_BANKS][NUM_CHANNELS];


int bank_idx[NUM_CHANNELS];
bool sequence[NUM_BANKS][NUM_BARS * BAR_RES][NUM_CHANNELS];

const int kRatePotiPin = 1;
const int kChannelPotiPin = 6;
const int kRandVariationsPotiPin = 2;

int current_channel = 0;

unsigned long midi_tick_counter = 0;

void blink_leds(int led_id) {
  digitalWrite(led_id, HIGH);
  delay(40);
  digitalWrite(led_id, LOW);
}

void setup()
{
  // Midi
#ifndef SERIAL_DEBUG
  Serial.begin(31250);
#else
  Serial.begin(115200);
#endif

  for (int i = 0; i < NUM_OUTS; ++i) {
    pinMode(outs[i].pin, OUTPUT);
    blink_leds(outs[i].pin);
  }

  for (int i = 0; i < NUM_INS; ++i) {
    pinMode(ins[i].pin, INPUT);
  }

  memset(sequence, 0, sizeof(sequence));
  memset(rand_variations, 0, sizeof(rand_variations));
  memset(bank_idx, 0, sizeof(bank_idx));

  digitalWrite(outs[OUT_BB_LED].pin, true);
}

void update_input_buttons() {
  for (int i = 0; i < NUM_INS; ++i) {
    const bool state = digitalRead(ins[i].pin);
    if (state && !ins[i].state) {
      ins[i].just_pressed |= true;
    }
    ins[i].state = state;
  }
}


void MIDINoteOn(int MIDInote) {
  static const int noteON = 144;
  MIDImessage(noteON, MIDInote, 127);
}

void MIDINoteOff(int MIDInote) {
  static const int noteOFF = 128;
  MIDImessage(noteOFF, MIDInote, 0);
}


void MIDImessage(int command, int MIDInote, int MIDIvelocity) {
#ifndef SERIAL_DEBUG
  Serial.write(command);//send note on or note off command
  Serial.write(MIDInote);//send pitch data
  Serial.write(MIDIvelocity);//send velocity data
#endif
}


void trigger_out(int id) {
  if (outs[id].inverse) {
    digitalWrite(outs[id].pin, false);
  } else {
    digitalWrite(outs[id].pin, true);
  }
  if (OUT_BB_LED != id && OUT_SYNC != id ) {
    MIDINoteOn(MIDI_NOTE_OFFSET + id);
  }
  outs[id].switch_state_at_time = millis() + TRIGGER_LENGTH_MS;
  outs[id].state = true;
}

void trigger_off() {
  const unsigned long now = millis();
  // Turn off triggers after TRIGGER_LENGTH_MS;
  for (int i = 0; i < NUM_OUTS; ++i) {
    if (outs[i].state && outs[i].switch_state_at_time > 0 && (now > outs[i].switch_state_at_time)) {
      outs[i].state = false;
      if (outs[i].inverse) {
        digitalWrite(outs[i].pin, HIGH);
      } else {
        digitalWrite(outs[i].pin, LOW);
      }
      if (OUT_BB_LED != i && OUT_SYNC != i ) {
        MIDINoteOff(MIDI_NOTE_OFFSET + i);
      }
    }
  }
}

int update_tick_divisor() {
  int idx = max(0, min(kNumMidiTicksDivs - 1, analogRead(kRatePotiPin) / (1024.0f / kNumMidiTicksDivs)));
  current_tick_div = kMidiTicksDivs[idx];
}

void trigger_out_by_idx(int idx) {
  switch (idx) {
    case 0:
      trigger_out(OUT1);
      break;
    case 1:
      trigger_out(OUT2);
      break;
    case 2:
      trigger_out(OUT3);
      break;
    case 3:
      trigger_out(OUT4);
      break;
    case 4:
      trigger_out(OUT5);
      break;
    case 5:
      trigger_out(OUT6);
      break;
    default:
      break;
  }
}

void clock_trigger(unsigned long midi_ticks, bool fill_pressed) {
  // MIDI clock starts with first tick.
  const bool sync_trigger = (midi_ticks % current_tick_div == 0);
  if (sync_trigger) {
    trigger_out(OUT_SYNC);

    trigger_out(OUT_BB_LED);

    if (fill_pressed) {
      trigger_out_by_idx(current_channel);
    }
  }

  // Detect beginning of new 32th note (every 3rd midi tick).
  if (midi_ticks % 3 == 0) {
    int seq_idx = ( midi_ticks / 3 ) % (NUM_BARS * BAR_RES);
    for (int c = 0; c < NUM_CHANNELS; ++c) {
      int c_bank_idx = bank_idx[c];
      if (rand_variations[c_bank_idx][c].tick_div == 0) {
        rand_variations[c_bank_idx][c].tick_div = current_tick_div;
      }
      const int vari_tick_div = rand_variations[c_bank_idx][c].tick_div;
      const bool vari_trigger = (midi_ticks % vari_tick_div == 0);
      const float probability = rand_variations[c_bank_idx][c].prob;
      // Randomly add triggers based on probability and sync_trigger.
      if (vari_trigger && probability > 0.0f && probability > rand_float()) {
        trigger_out_by_idx(c);
      } else
        // Randomly skip triggers based on probability and sync_trigger.
        if (probability < 0.0f && probability * -1.0f > rand_float()) {
          continue;
        } else if (sequence[c_bank_idx][seq_idx][c])  {
          trigger_out_by_idx(c);
        }
    }
  }
}

bool process_midi() {
#ifndef SERIAL_DEBUG
  if (Serial.available()) {
    int inByte = Serial.read();

    if (inByte == 0xF8) {
      // MIDI clock.
      ++midi_tick_counter;
      return true;
    }
    if (inByte == 0xFA) {
      // MIDI start.
      midi_tick_counter = 0;
      digitalWrite(outs[OUT_RESET].pin, LOW);
      return true;
    }
    if (inByte == 0xFC) {
      // MIDI stop
      midi_tick_counter = 0;
      digitalWrite(outs[OUT_RESET].pin, HIGH);
      return true;
    }
  }
  return false;
#else
  static unsigned long last_tick = 0;
  unsigned long now = millis();
  if (now - last_tick > 33) {
    last_tick = now;
    ++midi_tick_counter;
    return true;
  }
  return false;
#endif
}

void big_button_pressed(unsigned long midi_ticks) {
  // Quantize to current tick rate.
  const int seq_idx =
    static_cast<unsigned long>(floor((midi_ticks + current_tick_div / 2.0) / current_tick_div) * current_tick_div / 3) % (NUM_BARS * BAR_RES);
  sequence[bank_idx[current_channel]][seq_idx][current_channel] = true;
}

void clear_button_pressed(unsigned long midi_ticks) {
  const int seq_idx =
    static_cast<unsigned long>(floor((midi_ticks + current_tick_div / 2.0) / current_tick_div) * current_tick_div / 3) % (NUM_BARS * BAR_RES);
  sequence[bank_idx[current_channel]][seq_idx][current_channel] = false;
}

void delete_button_pressed() {
  for (int i = 0; i < NUM_BARS * BAR_RES; ++i) {
    sequence[bank_idx[current_channel]][i][current_channel] = false;
  }
  rand_variations[bank_idx[current_channel]][current_channel].prob = 0.0f;
}

void rand_button_pressed() {
  delete_button_pressed();
  for (int i = 0; i < NUM_BARS * BAR_RES; i += current_tick_div / 3)
    sequence[bank_idx[current_channel]][i][current_channel] = rand() & 1;
}

float rand_float() {
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}


float get_rand_variations() {
  int value = analogRead(kRandVariationsPotiPin);
  const int kOffRange = 100;
  if (value < 1024 / 2 - kOffRange) {
    return static_cast<float>(1024 / 2 - kOffRange - value) /
           static_cast<float>(1024 / 2 - kOffRange) * -1.0f;
  }
  if (value > 1024 / 2 + kOffRange) {
    return static_cast<float>(value - (1024 / 2 + kOffRange)) /
           static_cast<float>(1024 / 2 - kOffRange);
  }
  return 0.0f;
}


void update_channel() {
  int value = analogRead(kChannelPotiPin);
  //  static const int kPotiValues = {1023,  853, 682, 512,  341,  170,   0, 0 };
  static const int kChannelDiv = 1024 / (NUM_CHANNELS);
  current_channel =   max(0, min(NUM_CHANNELS, NUM_CHANNELS - ((value + kChannelDiv / 2)  / kChannelDiv)));
}

void turing_render(unsigned long midi_ticks) {
  // MIDI clock starts with first tick.
  const bool sync_trigger = (midi_ticks % current_tick_div == 0);
  if (!sync_trigger) {
    return;
  }
  trigger_out(OUT_SYNC);

  const bool bb_light = (midi_ticks / current_tick_div) & 1;
  digitalWrite(outs[OUT_BB_LED].pin, bb_light);

  static byte turing_state = rand();

  byte lsb = (turing_state & 1);
  if (ins[IN_BB].just_pressed) {
    lsb = !lsb;
    ins[IN_BB].just_pressed = false;
  }
  const float probability = get_rand_variations();
  if (probability != 0.0f && fabs(probability) / 2.0f > rand_float()) {
    if (probability > 0) {
      lsb = !lsb;
    } else {
      lsb = false;
    }
  }

  // bit rotation
  turing_state = (lsb << 7) | (turing_state >> 1);

  if ((turing_state & B1000000) && (turing_state & B00010000)) {
    trigger_out_by_idx(0);
  }
  if ((turing_state & B1000000) && (turing_state & B00000001)) {
    trigger_out_by_idx(1);
  }
  if (turing_state & B0100000) {
    trigger_out_by_idx(2);
  }
  if (turing_state & B0001000) {
    trigger_out_by_idx(3);
  }
  if ((turing_state & B0100000) || (turing_state & B00000010)) {
    trigger_out_by_idx(4);
  }
  if ((turing_state & B0010000) || (turing_state & B00010010)) {
    trigger_out_by_idx(5);
  }
}

void loop()
{
  update_channel();
  update_tick_divisor();
  update_input_buttons();
  trigger_off();


  const bool midi_tick_received = process_midi();

  if (midi_tick_counter == 0 || !midi_tick_received) {
    return;
  }

  if (current_channel >= NUM_CHANNELS) {
    turing_render(midi_tick_counter - 1);
    return;
  }

  if (ins[IN_BB].just_pressed) {
    big_button_pressed(midi_tick_counter - 1);
    ins[IN_BB].just_pressed = false;
  }
  if (ins[IN_CLEAR].state) {
    clear_button_pressed(midi_tick_counter - 1);
  }

  if (ins[IN_DELETE].state) {
    delete_button_pressed();
  }
  if (ins[IN_RAND].just_pressed) {
    rand_button_pressed();
    ins[IN_RAND].just_pressed = false;
  }
  if (ins[IN_BANK_SELECT].just_pressed) {
    bank_idx[current_channel] = !bank_idx[current_channel];
    digitalWrite(outs[OUT_BB_LED].pin, !bank_idx);
    ins[IN_BANK_SELECT].just_pressed = false;
  }

  static float last_probability = 0.0f;
  const float current_probability = get_rand_variations();
  const int current_bank_idx = bank_idx[current_channel];
  outs[OUT_BB_LED].inverse = current_bank_idx;
  if ((fabs(last_probability - current_probability) > 0.01) ||
      (rand_variations[current_bank_idx][current_channel].tick_div == 0)) {
    last_probability = current_probability;
    rand_variations[current_bank_idx][current_channel].prob = current_probability;
    rand_variations[current_bank_idx][current_channel].tick_div = current_tick_div;
  }

  // MIDI clock starts with first tick.
  clock_trigger(midi_tick_counter - 1,
                ins[IN_FILL].state);
}
