#ifndef _LCD_H
#define _LCD_H

#include <stdint.h>
#include <driver/i2c.h>

#define LCD_I2C_PORT    I2C_NUM_0

#define CMD_CLEAR       0x01
#define CMD_HOME        0x02
#define CMD_ENTRY_MODE  0x06
#define CMD_DISPLAY_ON  0x0C
#define CMD_DISPLAY_OFF 0x08
#define CMD_4BIT_MODE   0x28
#define CMD_SECOND_LINE 0xC0

class LCD
{
public:
    LCD();
    ~LCD();
    static void scan(int sda_pin, int scl_pin);
    bool setup(int sda_pin, int scl_pin, uint8_t address = 0x27);
    void writeCommand(uint8_t command);
    void print(char character);
    void printStr(const char* str);

private:
    void sendNibble(uint8_t nibble, bool isData);
    void sendByte(uint8_t byte, bool isData);
    void pulseEnable(uint8_t data);
    void writeI2C(uint8_t data);

    uint8_t _address;
};

#endif // _LCD_H
