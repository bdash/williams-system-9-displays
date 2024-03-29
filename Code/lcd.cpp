#include "lcd.h"

#include <cassert>
#include "hardware/gpio.h"

#include "pio_spi.h"
#include "spi.pio.h"

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

enum class ReadWrite : bool {
  Read = 1,
  Write = 0,
};

enum class RegisterSelect : bool {
  Instruction = 0,
  Data = 1,
};

static void lcd_write_packet(const struct pio_spi_inst *lcd_spi,
                             ReadWrite read_write,
                             RegisterSelect register_select,
                             std::span<const uint8_t> body);


inline constexpr uint8_t lcd_packet_header(ReadWrite read_write,
                                           RegisterSelect register_select) {
  return 0x1f | (static_cast<uint8_t>(register_select) << 6) |
         (static_cast<uint8_t>(read_write) << 5);
}


static void lcd_write_packet(const struct pio_spi_inst *lcd_spi,
                             ReadWrite read_write,
                             RegisterSelect register_select,
                             std::span<const uint8_t> body) {
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

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

  gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

class LCD_DOGS164::Impl {
public:
  Impl(PIO pio, uint sm, float clock_divisor, Pins pins) : pio_spi_{pio, sm} {
    uint offset = pio_add_program(pio0, &spi_program);

    pio_spi_cs_init(pio, sm, offset, 8 /* bits per SPI frame*/, clock_divisor,
                    pins.CS, pins.SCLK, pins.MOSI, pins.MISO);

    init();
  }

  void display(std::span<const uint8_t> data) {
    const uint8_t packet[] = {
        0x84, // Set RAM address to 4 since we're in bottom view and the first 4
              // bytes are off-screen.
    };
    lcd_write_packet(&pio_spi_, ReadWrite::Write, RegisterSelect::Instruction,
                     packet);
    lcd_write_packet(&pio_spi_, ReadWrite::Write, RegisterSelect::Data, data);
  }

  void display_at(uint8_t offset, std::span<const uint8_t> data) {
    assert(offset <= 0x73);
    const uint8_t packet[] = {
        static_cast<uint8_t>(0x84 + offset),
    };
    lcd_write_packet(&pio_spi_, ReadWrite::Write, RegisterSelect::Instruction,
                     packet);
    lcd_write_packet(&pio_spi_, ReadWrite::Write, RegisterSelect::Data, data);
  }

private:
  void init() {
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

    lcd_write_packet(&pio_spi_, ReadWrite::Write, RegisterSelect::Instruction, std::span(LCD_INITIALIZATION));
  }

  pio_spi_inst_t pio_spi_;
};

LCD_DOGS164::LCD_DOGS164(PIO pio, uint sm, float clock_divisor, Pins pins) {
  static_assert(sizeof(Impl) <= sizeof(impl_));
  ::new(&impl_[0]) Impl(pio, sm, clock_divisor, std::move(pins));
}

void LCD_DOGS164::display(std::span<const uint8_t> data) {
  impl().display(data);
}

void LCD_DOGS164::display_at(uint8_t offset, std::span<const uint8_t> data) {
  impl().display_at(offset, data);
}

LCD_DOGS164::Impl& LCD_DOGS164::impl() {
  return *reinterpret_cast<LCD_DOGS164::Impl*>(&impl_);
}
