#include "lcd.h"
#include <stdio.h>
#include <esp_rom_sys.h>
#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/task.h>

// PCF8574 pin mapping to HD44780
// Bit 0 = RS, Bit 1 = RW, Bit 2 = EN, Bit 3 = Backlight
// Bit 4 = D4, Bit 5 = D5, Bit 6 = D6, Bit 7 = D7
#define LCD_RS        0x01
#define LCD_RW        0x02
#define LCD_EN        0x04
#define LCD_BACKLIGHT 0x08

static void i2c_install(int sda_pin, int scl_pin)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 100000 },
        .clk_flags = 0,
    };
    i2c_param_config(LCD_I2C_PORT, &conf);
    i2c_driver_install(LCD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

void LCD::scan(int sda_pin, int scl_pin)
{
    i2c_install(sda_pin, scl_pin);

    printf("[LCD scan] Scanning addresses 0x08 to 0x77...\n");
    bool found = false;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(LCD_I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("[LCD scan] Device found at 0x%02X\n", addr);
            found = true;
        }
    }
    if (!found)
        printf("[LCD scan] No devices found — check wiring and power\n");

    i2c_driver_delete(LCD_I2C_PORT);
}

LCD::LCD()
{
    _address = 0;
}

LCD::~LCD()
{
    i2c_driver_delete(LCD_I2C_PORT);
}

bool LCD::setup(int sda_pin, int scl_pin, uint8_t address)
{
    _address = address;

    i2c_install(sda_pin, scl_pin);

    // Verify device is present before initializing
    i2c_cmd_handle_t probe = i2c_cmd_link_create();
    i2c_master_start(probe);
    i2c_master_write_byte(probe, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(probe);
    esp_err_t ret = i2c_master_cmd_begin(LCD_I2C_PORT, probe, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(probe);

    if (ret != ESP_OK) {
        printf("[LCD] Device not found at 0x%02X — check wiring\n", address);
        return false;
    }

    // Wait >40ms after power on
    vTaskDelay(pdMS_TO_TICKS(50));

    // HD44780 initialization in 4-bit mode
    sendNibble(0x03, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    sendNibble(0x03, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    sendNibble(0x03, false);
    esp_rom_delay_us(150);
    sendNibble(0x02, false);

    writeCommand(CMD_4BIT_MODE);
    writeCommand(CMD_DISPLAY_ON);
    writeCommand(CMD_ENTRY_MODE);
    writeCommand(CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));

    printf("[LCD] Initialized at 0x%02X\n", address);
    return true;
}

void LCD::writeI2C(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(LCD_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

void LCD::pulseEnable(uint8_t data)
{
    writeI2C(data & ~LCD_EN);
    esp_rom_delay_us(1);
    writeI2C(data | LCD_EN);
    esp_rom_delay_us(1);
    writeI2C(data & ~LCD_EN);
    esp_rom_delay_us(100);
}

void LCD::sendNibble(uint8_t nibble, bool isData)
{
    uint8_t data = (nibble << 4) | LCD_BACKLIGHT;
    if (isData)
        data |= LCD_RS;
    pulseEnable(data);
}

void LCD::sendByte(uint8_t byte, bool isData)
{
    sendNibble(byte >> 4, isData);
    sendNibble(byte & 0x0F, isData);
}

void LCD::writeCommand(uint8_t command)
{
    sendByte(command, false);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void LCD::print(char character)
{
    sendByte(character, true);
}

void LCD::printStr(const char* str)
{
    writeCommand(CMD_CLEAR);
    writeCommand(CMD_HOME);
    while (*str) {
        if (*str == '\n') {
            writeCommand(CMD_SECOND_LINE);
            str++;
        } else {
            print(*str++);
        }
    }
}
