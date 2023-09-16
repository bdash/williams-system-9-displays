#ifndef lcd_h__
#define lcd_h__

#include <cassert>
#include <cstdint>
#include <span>

void lcd_init(const struct pio_spi_inst *lcd_spi);

void lcd_display(const struct pio_spi_inst *lcd_spi, std::span<const uint8_t> data);

void lcd_set(const struct pio_spi_inst *lcd_spi, uint8_t offset,
                    std::span<const uint8_t> data);

#endif // lcd_h__
