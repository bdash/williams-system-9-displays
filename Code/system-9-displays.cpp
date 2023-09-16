#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include <span>

#include "pio_spi.h"
#include "spi.pio.h"

const uint LED_PIN = 25;

const uint LCD_RESET_PIN = 16;
const uint LCD_CS_PIN = 17;
const uint LCD_SCLK_PIN = 18;
const uint LCD_MOSI_PIN = 19;
const uint LCD_MISO_PIN = 20;

enum class ReadWrite : bool {
  Read = 1,
  Write = 0,
};

enum class RegisterSelect : bool {
  Instruction = 0,
  Data = 1,
};

inline constexpr uint8_t lcd_packet_header(ReadWrite read_write,
                                           RegisterSelect register_select) {
  return 0x1f | (static_cast<uint8_t>(register_select) << 6) |
         (static_cast<uint8_t>(read_write) << 5);
}

enum class DisplayState : bool {
  Off = 0,
  On = 1,
};

enum class CursorState : bool {
  Off = 0,
  On = 1,
};

enum class CursorBlink : bool {
  Off = 0,
  On = 1,
};

inline constexpr uint8_t lcd_display_on_off_control(DisplayState display_state,
                                                    CursorState cursor_state,
                                                    CursorBlink cursor_blink) {
  return (0b1000 | (static_cast<uint8_t>(display_state) << 2) |
          (static_cast<uint8_t>(cursor_state) << 1) |
          (static_cast<uint8_t>(cursor_blink) << 0));
}

inline constexpr uint8_t lcd_clear_display() { return 1; }

enum class FontWidth : bool {
  FiveDot = 0,
  SixDot = 1,
};

enum class BlackWhiteInversion : bool {
  Off = 0,
  On = 1,
};

enum class FourLineMode : bool {
  Off = 0,
  On = 1,
};

inline constexpr uint8_t
lcd_extended_function_set(FontWidth font_width,
                          BlackWhiteInversion black_white_inversion,
                          FourLineMode four_line_mode) {
  return (0b1000 | (static_cast<uint8_t>(font_width) << 2) |
          (static_cast<uint8_t>(black_white_inversion) << 1) |
          (static_cast<uint8_t>(four_line_mode) << 0));
}

enum class CommonDataShiftDirection : bool {
  Reverse = 0,
  Normal = 1,
};

enum class SegmentDataShiftDirection : bool {
  Reverse = 0,
  Normal = 1,
};

// TODO: This is for when RE=1. What about RE=0?
inline constexpr uint8_t
lcd_entry_mode_set(CommonDataShiftDirection common_data_shift_direction,
                   SegmentDataShiftDirection segment_data_shift_direction) {
  return (0b100 | (static_cast<uint8_t>(common_data_shift_direction) << 1) |
          (static_cast<uint8_t>(segment_data_shift_direction) << 0));
}

enum class DividerCircuit : bool {
  Off = 0,
  On = 1,
};

// Requires IS=1 and RE=0
inline constexpr uint8_t lcd_follower_control(DividerCircuit divider_cicuit,
                                              uint8_t resistor_ratio_flags) {
  assert(resistor_ratio_flags <= 0b111);
  return (0b1100000 | (static_cast<uint8_t>(divider_cicuit) << 3) |
          resistor_ratio_flags);
}

enum class IconDisplay : bool {
  Off = 0,
  On = 1,
};

enum class PowerRegulator : bool {
  Off = 0,
  On = 1,
};

// Requires IS=1 and RE=0
inline constexpr uint8_t
lcd_power_icon_contrast_set(IconDisplay icon_display,
                            PowerRegulator power_regulator, uint8_t contrast) {
  assert(contrast <= 0b11);
  return (0b1010000 | (static_cast<uint8_t>(icon_display) << 3) |
          (static_cast<uint8_t>(power_regulator) << 2) | contrast);
}

// Requires IS=1 and RE=0
inline constexpr uint8_t lcd_contrast_set(uint8_t contrast) {
  assert(contrast <= 0b1111);
  return 0b1110000 | contrast;
}

// Requires IS=1 and RE=0
inline constexpr uint8_t
lcd_divider_oscillator_set(uint8_t bias, uint8_t oscillator_frequency) {
  assert(bias <= 1);
  assert(oscillator_frequency <= 0b111);
  return (0b10000 | (bias << 3) | oscillator_frequency);
}

enum class DoubleHeightMode : uint8_t {
  SmallSmallBig = 0,
  SmallBigSmall = 1,
  BigBig = 2,
  BigSmallSmall = 3,
};

// Requires IS=0 and RE=1
inline constexpr uint8_t
lcd_double_height_bias_dot_shift_set(DoubleHeightMode double_height,
                                     uint8_t bias, bool display_shift) {
  assert(bias <= 1);
  return (0b10000 | (static_cast<uint8_t>(double_height) << 2) | bias << 1 |
          display_shift);
}

enum class DataLengthControl : bool {
  FourBitBus = 0,
  EightBitBus = 1,
};

enum class DisplayLineControl : bool {
  OneOrThreeLine = 0,
  TwoOrFourLine = 1,
};

enum class ExtendedFunctions : bool {
  Off = 0,
  On = 1,
};

enum class DoubleHeight : bool {
  Off = 0,
  On = 1,
};

enum class SpecialRegisters : bool {
  Off = 0,
  On = 1,
};

// Requires RE=1
inline constexpr uint8_t lcd_function_set(DataLengthControl data_length_control,
                                          DisplayLineControl display_lines,
                                          DoubleHeight double_height,
                                          SpecialRegisters special_registers) {
  return (0b100000| (static_cast<bool>(data_length_control) << 4) |
          (static_cast<bool>(display_lines) << 3) |
          (static_cast<bool>(double_height) << 2) |
          (static_cast<bool>(ExtendedFunctions::Off) << 1) |
          static_cast<bool>(special_registers));
}

enum class DataBlink : bool {
  Off = 0,
  On = 1,
};

enum class Reverse : bool {
  Off = 0,
  On = 1,
};

inline constexpr uint8_t lcd_function_set(DataLengthControl data_length_control,
                                          DisplayLineControl display_lines,
                                          DataBlink blink, Reverse reverse) {

  return (0b100000 | (static_cast<bool>(data_length_control) << 4) |
          (static_cast<bool>(display_lines) << 3) |
          (static_cast<bool>(blink) << 2) |
          (static_cast<bool>(ExtendedFunctions::On) << 1) |
          static_cast<bool>(reverse));
}

static void lcd_write_packet(const struct pio_spi_inst *lcd_spi,
                             ReadWrite read_write,
                             RegisterSelect register_select,
                             std::span<const uint8_t> body) {
  gpio_put(LED_PIN, 1);

  std::array<uint8_t, 256> packet;
  assert(1 + body.size() * 2 <= 256);

  size_t size = 0;
  packet[size++] = lcd_packet_header(read_write, register_select);

  for (auto byte : body) {
    packet[size++] = static_cast<uint8_t>(byte & 0x0f);
    packet[size++] = static_cast<uint8_t>((byte & 0xf0) >> 4);
  }

  pio_spi_write8_blocking(lcd_spi, packet.data(), size);
  pio_spi_wait_til_idle(lcd_spi);

  gpio_put(LED_PIN, 0);
}

static void lcd_init(const struct pio_spi_inst *lcd_spi) {
  const uint8_t LCD_INITIALIZATION[] = {
      lcd_function_set(DataLengthControl::EightBitBus, DisplayLineControl::TwoOrFourLine, DataBlink::Off, Reverse::Off),
      lcd_extended_function_set(FontWidth::FiveDot, BlackWhiteInversion::Off, FourLineMode::On),
      lcd_entry_mode_set(CommonDataShiftDirection::Reverse, SegmentDataShiftDirection::Normal), // Bottom view
      lcd_double_height_bias_dot_shift_set(DoubleHeightMode::BigSmallSmall, 1, false),
      lcd_function_set(DataLengthControl::EightBitBus, DisplayLineControl::TwoOrFourLine, DoubleHeight::Off, SpecialRegisters::On),
      lcd_divider_oscillator_set(1, 0b011),
      lcd_follower_control(DividerCircuit::On, 0b100),
      lcd_power_icon_contrast_set(IconDisplay::Off, PowerRegulator::On, 0),
      lcd_contrast_set(0b1111),
      lcd_function_set(DataLengthControl::EightBitBus, DisplayLineControl::TwoOrFourLine, DoubleHeight::Off, SpecialRegisters::Off),
      lcd_display_on_off_control(DisplayState::On, CursorState::Off, CursorBlink::Off),
      lcd_clear_display(),
  };

  lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Instruction, std::span(LCD_INITIALIZATION));
}

static void lcd_display(const struct pio_spi_inst *lcd_spi,
                        std::span<const uint8_t> data) {
  const uint8_t packet[] = {
      0x84, // Set RAM address to 4 since we're in bottom view and the first 4
            // bytes are off-screen.
  };
  lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Instruction,
                   packet);
  lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Data, data);
}

static void lcd_set(const struct pio_spi_inst *lcd_spi, uint8_t offset,
                    std::span<const uint8_t> data) {
  assert(offset <= 0x73);
  const uint8_t packet[] = {
      static_cast<uint8_t>(0x84 + offset),
  };
  lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Instruction,
                   packet);
  lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Data, data);
}

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
