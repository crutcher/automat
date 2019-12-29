// Style:
// ClassNames.methodNames()
// ClassNames.member_names_
// globalFunctions()
// local_variables
// g_global_variables
// CONSTANT_VALUES


#include <ArduinoOTA.h>

// Shutup FastLED pragma message:
#define FASTLED_INTERNAL
#include <FastLED.h>


// PINS:
// LED_BUILTIN

#define PIN_A0 17
#define PIN_D0 16
#define PIN_D5 14
#define PIN_D6 12
#define PIN_D7 13
#define PIN_D8 15

#define PIN_TX 1
#define PIN_RX 3
#define PIN_D1 5
#define PIN_D2 4
#define PIN_D3 0
#define PIN_D4 2

// FIXME(crutcher): We should be able to use IDE constants (D1, D2, etc).
// But it seems the board-setup-selection is wrong in our env; so
// instead we're redefining them for the Wemos D1

const int CELL_LIGHTS_CLOCK_PIN = PIN_D4;
const int CELL_LIGHTS_DATA_PIN = PIN_D3;

const int VFD_DATA_PIN = PIN_D0;
const int VFD_CLOCK_PIN = PIN_D5;
const int VFD_LATCH_PIN = PIN_D6;
const int VFD_OE_PIN = PIN_D7;


CRGB AUTOMAT_GREEN(22, 164, 40);


String g_cell_control_topic; // "automat/cell/{ID}/control"
String g_automat_cell_id;


#define g_fastled_values_size 4

CRGB g_fastled_values[g_fastled_values_size];

void setLedColor(CRGB color) {
  for (int i = 0; i < g_fastled_values_size; ++i) {
    g_fastled_values[i] = color;
  }    
}

int scaleRange(int val, int in_low, int in_high, int out_low, int out_high) {
  return (val - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}

/**
 * Clamp a value between [low, high].
 *
 * @param val: the value to clamp.
 * @param low: the lowest output, inclusive.
 * @param high: the highest output, inclusive.
 * @returns: the clamped value
 */
int clamp(int val, int low, int high) {
  if (val > high) {
    return high;
  } else if (val < low) {
    return low;
  }
  return val;
}


/**
 * A network syncable millis() timer.
 */
class PhaseTimer {
  public:
    PhaseTimer(
        unsigned long offset,
        unsigned int dwell_weight,
        unsigned int update_weight) {
      offset_ = offset;
      dwell_weight_ = dwell_weight;
      update_weight_ = update_weight;
    }

    unsigned long phase_time() {
      return millis() + offset_;
    }

    /**
     * Update the local idea of time with a remote message.
     *
     * We do not model roundtrip timing here; just average
     * phase time as it reaches us. The assumption is that
     * all nodes are roughly the same 'distance' from the
     * master.
     */
    void update_phase(unsigned long remote_time) {
      // This is what the offset of the remote time is wrt to our clock.
      // This ignores transmission delays.
      unsigned long remote_offset = remote_time - millis();
      
      if (!initialized_) {
        // The very first message is authoritative.
        initialized_ = true;
        offset_ = remote_offset;

      } else {
        // Subsequent messages are averaged.

        unsigned int denom = dwell_weight_ + update_weight_;

        // (a * old + b * new) / (a + b)
        // (a * old / (a + b)) + (b * new / (a + b))

        offset_ = dwell_weight_ * (offset_ / denom) + update_weight_ * (remote_offset / denom);
      }
    }
       
  private:
    unsigned int initialized_ = false;
    unsigned int dwell_weight_;
    unsigned int update_weight_;
    unsigned long offset_;
};

PhaseTimer g_phase_timer(0, 4, 1);


class CycleTimer {
  public:
    CycleTimer(long period_millis, PhaseTimer *phase_timer) {
      period_millis_ = period_millis;
      phase_timer_ = phase_timer;
    }

    long value() {
      return phase_timer_->phase_time() % period_millis_;
    }

    uint8_t byteValue() {
      return (256 * value()) / period_millis_;
    }

  private:
    long period_millis_;
    PhaseTimer *phase_timer_;
};

class PulseGenerator {
  public:
    PulseGenerator(long pulse_time, long delay_ms, PhaseTimer *phase_timer) {
      phase_timer_ = phase_timer;
      pulse_time_ = pulse_time;
      delay_ms_ = delay_ms;
      last_pulse_ = 0;
    }

    bool pulse() {
      bool p = false;
      unsigned long now = phase_timer_->phase_time();

      if (last_pulse_ + pulse_time_ > now) {
        // The previous pulse is still active.
        p = true;
        
      } else if (last_pulse_ + delay_ms_ < now) {
        // Start a new pulse.
        last_pulse_ = now;
        p = true;
      }

      return p;
    }

  private:
    PhaseTimer *phase_timer_;
    long pulse_time_;
    long delay_ms_;
    unsigned long last_pulse_;
};




class CellPower {
  public:
    CellPower(
        PhaseTimer *phase_timer,
        uint8_t power_low,
        uint8_t power_high,
        uint8_t glitch) {
      phase_timer_ = phase_timer;
      heartbeat_ = new CycleTimer(6500, phase_timer);
      power_low_ = power_low;
      power_high_ = power_high;
      glitch_ = glitch;
    }

    void loop() {
      int wave = scaleRange(
        cubicwave8(heartbeat_->byteValue()),
        0, 255,
        power_low_, power_high_);

      wave += scaleRange(
        inoise8(phase_timer_->phase_time()),
        0, 255,
        -glitch_ / 8, glitch_ / 8);

      power_ = clamp(wave, 0, 255);
    }

    uint8_t power() {
      return power_;
    }

    void set_power_high(uint8_t power_high) {
      power_high_ = power_high;
    }

    void set_power_low(uint8_t power_low) {
      power_low_ = power_low;
    }

    uint8_t get_glitch() {
      return glitch_;
    }

    void update_glitch(uint8_t glitch) {
      if (random(10) == 0) {
        glitch_ = (((int) glitch) + ((int) glitch_)) / 2;
      }
    }

    void dump() {
      Serial.print("CellPower(");
      Serial.print(power_low_);
      Serial.print(", ");
      Serial.print(power_high_);
      Serial.print(")[");
      Serial.print(glitch_);
      Serial.println("]");
    }

  private:
    PhaseTimer *phase_timer_;
    CycleTimer *heartbeat_;
    
    uint8_t power_high_;
    uint8_t power_low_;
    uint8_t glitch_;
    uint8_t power_;
};

CellPower g_cell_power(&g_phase_timer, 30, 255, 45);


class VfdBank {
  public:
    uint8_t num_tubes_;
    byte *ghost_bank_;
    byte *solid_bank_;
    byte *flicker_bank_;

    VfdBank(
        uint8_t num_tubes,
        uint8_t data_pin,
        uint8_t clock_pin,
        uint8_t latch_pin,
        uint8_t oe_pin) {
      num_tubes_ = num_tubes;
      data_pin_ = data_pin;
      clock_pin_ = clock_pin;
      latch_pin_ = latch_pin;

      solid_bank_ = new byte[num_tubes];
      ghost_bank_ = new byte[num_tubes];
      flicker_bank_ = new byte[num_tubes];
    }

    void show() {
      digitalWrite(latch_pin_, LOW);
      for (int i = num_tubes_; i > 0; --i) {
        int idx = i - 1;

        byte value = solid_bank_[idx];

        if (random(100) > 95) {
          value |= ghost_bank_[idx];
        }
        
        shiftOut(data_pin_, clock_pin_, MSBFIRST, value);
      }
      digitalWrite(latch_pin_, HIGH);
    }

    void clear_solid() {
      for (int i = 0; i < num_tubes_; ++i) {
        solid_bank_[i] = 0;
      }
    }

    void clear_ghost() {
      for (int i = 0; i < num_tubes_; ++i) {
        ghost_bank_[i] = 0;
      }
    }

    void clear_flicker() {
      for (int i = 0; i < num_tubes_; ++i) {
        flicker_bank_[i] = 0;
      }
    }

    void clear() {
      clear_solid();
      clear_ghost();
      clear_flicker();
    }

  private:
    uint8_t data_pin_;
    uint8_t clock_pin_;
    uint8_t latch_pin_;

};

VfdBank vfd_bank(10, VFD_DATA_PIN, VFD_CLOCK_PIN, VFD_LATCH_PIN, VFD_OE_PIN);

void setup() {
  // Setup serial debugging.
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  delay(10);
  Serial.println("");
  Serial.println("setup");

  pinMode(VFD_DATA_PIN, OUTPUT);
  pinMode(VFD_CLOCK_PIN, OUTPUT);
  pinMode(VFD_LATCH_PIN, OUTPUT);
  pinMode(VFD_OE_PIN, OUTPUT);

  pinMode(CELL_LIGHTS_DATA_PIN, OUTPUT);
  pinMode(CELL_LIGHTS_CLOCK_PIN, OUTPUT);

  vfd_bank.clear();
  vfd_bank.show();

  FastLED.addLeds<
    WS2801,
    CELL_LIGHTS_DATA_PIN,
    CELL_LIGHTS_CLOCK_PIN,
    RGB>(
      g_fastled_values,
      g_fastled_values_size);

}


unsigned long lastDigitMillis = 0;
const long DIGIT_DELAY = 1750;

unsigned long lastFlickerDigitMillis = 0;
const long FLICKER_DIGIT_DELAY = 800;

byte garbage() {
   return (millis() / 100) & 0xFF;
}

void loop() {
  
  // POWER!
  g_cell_power.loop();
  FastLED.setBrightness(g_cell_power.power());
  // wemosd1's analogWrite is 0-1024 (not 0-255).
  // VFD_OE_PIN is active-low.
  analogWrite(VFD_OE_PIN, 1024 - (g_cell_power.power() * 4));

  setLedColor(AUTOMAT_GREEN);
  FastLED.show();


  long now = millis();
  if (now - lastDigitMillis > DIGIT_DELAY) {
    lastDigitMillis = now;

    // Right shift.
    for (int i = vfd_bank.num_tubes_; i > 1; --i) {
      vfd_bank.solid_bank_[i-1] = vfd_bank.solid_bank_[i-2];
    }
   vfd_bank.solid_bank_[0] = garbage();
 }
 
 if (now - lastDigitMillis > FLICKER_DIGIT_DELAY) {
    lastFlickerDigitMillis = now;

    // Right shift.
    for (int i = vfd_bank.num_tubes_; i > 1; --i) {
      vfd_bank.ghost_bank_[i-1] = vfd_bank.ghost_bank_[i-2];
    }
   vfd_bank.ghost_bank_[0] = garbage();
 }

 vfd_bank.show();
}
