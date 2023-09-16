#ifndef lcd_h__
#define lcd_h__

#include <cstdint>
#include <span>

#include "pio_spi.h"

class LCD_DOGS164 {
public:
  struct Pins {
    uint CS, SCLK, MOSI, MISO;
  };

  LCD_DOGS164(PIO pio, uint sm, float clock_divisor, Pins pins);

  void display(std::span<const uint8_t> data);
  void display_at(uint8_t offset, std::span<const uint8_t> data);

private:
  void init();

  pio_spi_inst_t pio_spi_;
};

#endif // lcd_h__
