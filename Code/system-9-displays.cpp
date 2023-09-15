#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"

#include <span>

#include "pio_spi.h"
#include "spi.pio.h"

const uint LED_PIN = 25;

const uint LCD_RESET_PIN = 16;
const uint LCD_CS_PIN = 17;
const uint LCD_SCLK_PIN = 18;
const uint LCD_MOSI_PIN = 19;
const uint LCD_MISO_PIN = 20;

enum class ReadWrite: uint8_t {
    Read = 1,
    Write = 0,
};

enum class RegisterSelect: uint8_t {
    Instruction = 0,
    Data = 1,
};

inline constexpr uint8_t lcd_packet_header(ReadWrite read_write, RegisterSelect register_select) {
    return 0x1f | (static_cast<uint8_t>(register_select) << 6) | (static_cast<uint8_t>(read_write) << 5);
}

enum class DisplayState: uint8_t {
    Off = 0,
    On = 1,
};

enum class CursorState: uint8_t {
    Off = 0,
    On = 1,
};

enum class CursorBlink: uint8_t {
    Off = 0,
    On = 1,
};

inline constexpr uint8_t lcd_display_on_off_control(DisplayState display_state, CursorState cursor_state, CursorBlink cursor_blink) {
    return (0b1000 
           | (static_cast<uint8_t>(display_state) << 2)
           | (static_cast<uint8_t>(cursor_state) << 1)
           | (static_cast<uint8_t>(cursor_blink) << 0));
}

inline constexpr uint8_t lcd_clear_display() {
    return 1;
}

enum class FontWidth: uint8_t {
    FiveDot = 0,
    SixDot = 1,
};

enum class BlackWhiteInversion: uint8_t {
    Off = 0,
    On = 1,
};

enum class FourLineMode: uint8_t {
    Off = 0,
    On = 1,
};

inline constexpr uint8_t lcd_extended_function_set(FontWidth font_width, BlackWhiteInversion black_white_inversion, FourLineMode four_line_mode) {
   return (0b1000
          | (static_cast<uint8_t>(font_width) << 2)
          | (static_cast<uint8_t>(black_white_inversion) << 1)
          | (static_cast<uint8_t>(four_line_mode) << 0));
}

enum class CommonDataShiftDirection: uint8_t {
    Reverse = 0,
    Normal = 1,
};

enum class SegmentDataShiftDirection: uint8_t {
    Reverse = 0,
    Normal = 1,
};

// TODO: This is for when RE=1. What about RE=0?
inline constexpr uint8_t lcd_entry_mode_set(CommonDataShiftDirection common_data_shift_direction, SegmentDataShiftDirection segment_data_shift_direction) {
    return (0b100
           | (static_cast<uint8_t>(common_data_shift_direction) << 1)
           | (static_cast<uint8_t>(segment_data_shift_direction) << 0));
}


static void lcd_write_packet(const struct pio_spi_inst* lcd_spi, ReadWrite read_write, RegisterSelect register_select, std::span<const uint8_t> body) {
    gpio_put(lcd_spi->cs_pin, 0);
    gpio_put(LED_PIN, 1);

    const uint8_t header = lcd_packet_header(read_write, register_select);
    pio_spi_write8_blocking(lcd_spi, &header, sizeof(header));

    for (auto byte : body) {
        uint8_t packet[] = {
            static_cast<uint8_t>(byte & 0x0f),
            static_cast<uint8_t>((byte & 0xf0) >> 4),
         };

        pio_spi_write8_blocking(lcd_spi, packet, sizeof(packet));
    }

    pio_spi_wait_til_idle(lcd_spi);
    
    gpio_put(lcd_spi->cs_pin, 1);
    gpio_put(LED_PIN, 0);
}

static void lcd_init(const struct pio_spi_inst* lcd_spi) {
    const uint8_t LCD_INITIALIZATION[] = {
        0x3a,
        lcd_extended_function_set(FontWidth::FiveDot, BlackWhiteInversion::Off, FourLineMode::On),
        lcd_entry_mode_set(CommonDataShiftDirection::Reverse, SegmentDataShiftDirection::Normal), // Bottom view
        0x1e,
        0x39,
        0x1b,
        0x6c,
        0x54,
        0x7a,
        0x38,
        lcd_display_on_off_control(DisplayState::On, CursorState::Off, CursorBlink::Off),
        lcd_clear_display(),
    };

    lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Instruction, std::span(LCD_INITIALIZATION));
}

static void lcd_display(const struct pio_spi_inst* lcd_spi, std::span<const uint8_t> data) {
    const uint8_t packet[] = { 
        0x84, // Set RAM address to 4 since we're in bottom view and the first 4 bytes are off-screen.
    };
    lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Instruction, packet);
    lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Data, data);
}

static void lcd_set(const struct pio_spi_inst* lcd_spi, uint8_t offset, std::span<const uint8_t> data) {
    assert(offset <= 0x73);
    const uint8_t packet[] = { 
        static_cast<uint8_t>(0x84 + offset),
    };
    lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Instruction, packet);
    lcd_write_packet(lcd_spi, ReadWrite::Write, RegisterSelect::Data, data);
}


int main()
{
    stdio_init_all();
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(LCD_RESET_PIN);
    gpio_put(LCD_RESET_PIN, 1);
    gpio_set_dir(LCD_RESET_PIN, GPIO_OUT);

    gpio_init(LCD_CS_PIN);
    gpio_put(LCD_CS_PIN, 1);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);

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

    pio_spi_cs_init(lcd_spi.pio, lcd_spi.sm, prog_offset, 8 /* bits per SPI frame*/, clkdiv, LCD_SCLK_PIN, LCD_MOSI_PIN, LCD_MISO_PIN);

    lcd_init(&lcd_spi);
    uint8_t message[] = "0123456789012345XXXX0123456789012345XXXX0123456789012345XXXX0123456789012345";
    lcd_display(&lcd_spi, std::span(message, sizeof(message) - 1));
    
    while (true) {
        sleep_ms(500);
        message[1]++;
        lcd_set(&lcd_spi, 1, std::span(&message[1], 1));
    }

    return 0;
}
