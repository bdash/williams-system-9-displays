#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include <span>

#include "lcd.h"
#include "pio_spi.h"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

const uint LCD_RESET_PIN = 16;
const uint LCD_CS_PIN = 17;
const uint LCD_SCLK_PIN = 18;
const uint LCD_MOSI_PIN = 19;
const uint LCD_MISO_PIN = 20;

int main() {
  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  gpio_init(LCD_RESET_PIN);
  gpio_put(LCD_RESET_PIN, 1);
  gpio_set_dir(LCD_RESET_PIN, GPIO_OUT);

  // Low pulse > 0.2ms to trigger reset of LCD.
  gpio_put(LCD_RESET_PIN, 0);
  sleep_ms(1);
  gpio_put(LCD_RESET_PIN, 1);

  struct pio_spi_inst lcd_spi = {
      .pio = pio0,
      .sm = 0,
      .cs_pin = LCD_CS_PIN,
  };

  uint prog_offset = pio_add_program(pio0, &spi_program);
  // This is too fast on the breadboard:
  // float clkdiv = 31.25f;  // 1 MHz @ 125 clk_sys
  float clkdiv = 125.f;

  pio_spi_cs_init(lcd_spi.pio, lcd_spi.sm, prog_offset,
                  8 /* bits per SPI frame*/, clkdiv, LCD_CS_PIN, LCD_SCLK_PIN,
                  LCD_MOSI_PIN, LCD_MISO_PIN);

  lcd_init(&lcd_spi);
  uint8_t message[] = "0123456789012345XXXX0123456789012345XXXX0123456789012345"
                      "XXXX0123456789012345";
  lcd_display(&lcd_spi, std::span(message, sizeof(message) - 1));

  while (true) {
    sleep_ms(500);
    message[1]++;
    lcd_set(&lcd_spi, 1, std::span(&message[1], 1));
  }

  return 0;
}
