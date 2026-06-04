#ifndef _SIMPLEUART_H_
#define _SIMPLEUART_H_

#include <driver/uart.h>

class SimpleUART
{
public:
    SimpleUART(int baud_rate, uart_port_t port = UART_NUM_0, int tx_io = UART_PIN_NO_CHANGE, int rx_io = UART_PIN_NO_CHANGE, int buff_size = 256);
    ~SimpleUART();
    int read(char message_in[], uint32_t message_length);
    void write(char message_out[], uint32_t message_length);
    int available();

private:
    uart_port_t _port;
    QueueHandle_t _queue;
    uart_event_t _event;
};



#endif // _SIMPLEUART_H_