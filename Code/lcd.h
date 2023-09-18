#ifndef lcd_h__
#define lcd_h__

#include <cstdint>
#include <span>

#include "hardware/pio.h"

class LCD_DOGS164 {
public:
  struct Pins {
    uint CS, SCLK, MOSI, MISO;
  };

  LCD_DOGS164(PIO pio, uint sm, float clock_divisor, Pins pins);

  void display(std::span<const uint8_t> data);
  void display_at(uint8_t offset, std::span<const uint8_t> data);

private:
  class Impl;
  Impl& impl();

  std::aligned_storage_t<sizeof(uint64_t)> impl_[4];
};

#endif // lcd_h__
