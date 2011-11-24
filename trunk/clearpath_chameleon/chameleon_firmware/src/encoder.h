
template <uint16_t PERIOD_MS>
class Encoder {
  public:
    volatile int32_t ticks;
    volatile int16_t ticks_delta;
    
    Encoder(): last_a(0), ticks(0), ticks_delta(0) {}
    
    volatile void tick(int8_t a, int8_t b) {
      if (b == last_a) {
        ticks++; ticks_delta++;
      } else {
        ticks--; ticks_delta--;
      }
      last_a = a;
    }
    
    int16_t get_ticks_per_second() {
      return ticks_delta * (1000 / PERIOD_MS);
    }

    void reset_ticks_per_second() {
      ticks_delta = 0;
    }
      
  private:
    volatile int8_t last_a;
};
