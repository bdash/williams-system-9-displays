#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include <stdio.h>

#include <cstring>
#include <span>

#include "lcd.h"
#include "pio_spi.h"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

const uint LCD_RESET_PIN = 16;
const uint LCD_CS_PIN = 17;
const uint LCD_SCLK_PIN = 18;
const uint LCD_MOSI_PIN = 19;
const uint LCD_MISO_PIN = 20;

const uint GREEN_LED_PIN = 27;
const uint AMBER_LED_PIN = 26;
const uint RED_LED_PIN = 22;

const uint ST_N_0 = 0;
const uint ST_N_1 = 1;
const uint ST_N_2 = 2;
const uint ST_N_3 = 3;
const uint ST_N_VALID = 4;

const uint DIGIT_1_A = 5;
const uint DIGIT_1_B = 6;
const uint DIGIT_1_C = 7;
const uint DIGIT_1_D = 8;

const uint DIGIT_2_A = 9;
const uint DIGIT_2_B = 10;
const uint DIGIT_2_C = 11;
const uint DIGIT_2_D = 12;

const uint COMMA_1_2 = 13;
const uint COMMA_3_4 = 14;
const uint BLANKING = 15;

const uint DISPLAY_BUFFER_COUNT = 4;
const uint DISPLAY_BUFFER_SIZE = 80;

uint8_t display_buffers[DISPLAY_BUFFER_COUNT][DISPLAY_BUFFER_SIZE];
queue_t display_queue;

LCD_DOGS164 *shared_lcd = nullptr;

void update_display() {
  uint8_t last_display_buffer[DISPLAY_BUFFER_SIZE];
  uint8_t update_count = 0;
  while (true) {
    uint i = 0;
    queue_remove_blocking(&display_queue, &i);

    uint8_t *display_buffer = display_buffers[i];
    if (memcmp(display_buffer, last_display_buffer, DISPLAY_BUFFER_SIZE)) {
      display_buffer[72] = update_count++;
      shared_lcd->display_at(0, std::span(display_buffer, DISPLAY_BUFFER_SIZE));
      display_buffer[72] = ' ';
      memcpy(last_display_buffer, display_buffer, DISPLAY_BUFFER_SIZE);
    }

    memset(display_buffer, ' ' , DISPLAY_BUFFER_SIZE);
  }
}

int main() {
  stdio_init_all();

  // We use all GPIOs up to and including BLANKING as inputs.
  gpio_init_mask((1 << (BLANKING + 1)) - 1);
  gpio_set_dir_in_masked((1 << (BLANKING + 1)) - 1);

  // Three LEDs for output, plus the Pico's onboard one.
  gpio_init_mask(1 << LED_PIN | 1 << GREEN_LED_PIN | 1 << AMBER_LED_PIN | 1 << RED_LED_PIN);
  gpio_set_dir_out_masked(1 << LED_PIN | 1 << GREEN_LED_PIN | 1 << AMBER_LED_PIN | 1 << RED_LED_PIN);

  gpio_init(LCD_RESET_PIN);
  gpio_put(LCD_RESET_PIN, 1);
  gpio_set_dir(LCD_RESET_PIN, GPIO_OUT);

  // Low pulse > 0.2ms to trigger reset of LCD.
  gpio_put(LCD_RESET_PIN, 0);
  sleep_ms(1);
  gpio_put(LCD_RESET_PIN, 1);

  // TODO: Why do faster speeds not work?
  // float clock_divisor = 31.25f;  // 1 MHz @ 125 clk_sys
  float clock_divisor = 125.f;

  LCD_DOGS164 lcd(pio0, 0, clock_divisor, {.CS = LCD_CS_PIN, .SCLK = LCD_SCLK_PIN, .MOSI = LCD_MOSI_PIN, .MISO = LCD_MISO_PIN });

  // Initialize state shared with display thread
  shared_lcd = &lcd;
  queue_init(&display_queue, sizeof(uint), DISPLAY_BUFFER_COUNT - 1);
  memset(display_buffers, ' ', sizeof(display_buffers));

  // Kick off the display thread.
  multicore_launch_core1(&update_display);

  // Local state for the main loop.
  bool was_updated = true;
  uint current_display_buffer = 0;
  uint8_t *display_buffer = display_buffers[current_display_buffer];
  bool last_strobes_valid = false;
  uint8_t strobe_transition_count = 0;

  while (true) {
    uint32_t gpios = gpio_get_all();
    bool strobes_valid = !((gpios & 0b10000) >> 4);

    if (strobes_valid != last_strobes_valid) {
      gpio_put(GREEN_LED_PIN, strobes_valid);
      strobe_transition_count++;
      last_strobes_valid = strobes_valid;
    }

    if (strobes_valid) {
      // We have a valid strobe signal. Extract the strobe number
      // to determine which positions we'll be updating, and
      // extract the digits that will be displayed.
      uint32_t strobe = gpios & 0b1111;
      uint32_t digit_1 = (gpios & 0b0000111100000) >> 5;
      uint32_t digit_2 = (gpios & 0b1111000000000) >> 9;

      // TODO: Do something with commas and blanking.
      #if 0
      // uint32_t comma_1_2 = (gpios & 0b110000000000000) >> 13;
      // uint32_t comma_3_4 = (gpios & 0b11000000000000000) >> 15;
      // uint32_t blanking = (gpios & (1 << BLANKING)) >> (BLANKING - 1);
      #endif

      // Reverse the order of the digits so they match viewing order.
      strobe = 15 - strobe;

      // Move credits / ball in play to different lines than scores.
      if (strobe == 0 || strobe == 8) {
        strobe = 40 + bool(strobe & 8);
      }

      // TODO: Why do we get a valid strobe signal with invalid digits?
      // Are we seeing the strobe signal transition before the digit is
      // ready, or do these non-BCD values signify something?
      if (digit_1 < 10) {
        display_buffer[strobe] = '0' + digit_1;
        was_updated = true;
      }

      if (digit_2 < 10) {
        display_buffer[20 + strobe] = '0' + digit_2;
        was_updated = true;
      }

    } else if (was_updated && strobe_transition_count > 1) {
      // Let the display thread know that it has work to do.
      // We block here to avoid mutating data the display thread is
      // working with in the hopefully unlikely event that we
      // are running significantly faster than the display thread.
      bool success = queue_try_add(&display_queue, &current_display_buffer);
      assert(success);

      // Update to the next display buffer
      current_display_buffer = (current_display_buffer + 1) % DISPLAY_BUFFER_COUNT;
      display_buffer = display_buffers[current_display_buffer];

      // Reset update tracking.
      was_updated = false;
      strobe_transition_count = 0;
    }
  }

  return 0;
}
