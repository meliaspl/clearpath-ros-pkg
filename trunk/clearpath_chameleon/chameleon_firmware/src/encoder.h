

class Encoder {
  public:
    volatile int32_t ticks;
    int16_t ticks_per_second;
    
    Encoder(): last_a(0), last_time_ms(0), 
               ticks(0), ticks_per_second(0) {}
    
    void tick(int8_t a, int8_t b) {
      if (b == last_a) ticks++; else ticks--;
      last_a = a;
    }
    
    int16_t compute_speed(uint32_t time_ms) {
      ticks_per_second = (ticks - last_ticks) * (1000 / (time_ms - last_time_ms));
      
      last_time_ms = time_ms;
      last_ticks = ticks;
      return ticks_per_second;
    }
      
  private:
    volatile int8_t last_a;
    uint32_t last_time_ms;
    uint32_t last_ticks;    
};
