#include "SimpleUART.h"

SimpleUART::SimpleUART(int baud_rate, uart_port_t port, int tx_io, int rx_io, int buff_size)
{
    _port = port;
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_REF_TICK,
    };
    uart_driver_install(port, buff_size, buff_size, 20, &_queue, 0);
    uart_param_config(port, &uart_config);
    uart_set_pin(port, tx_io, rx_io, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

SimpleUART::~SimpleUART()
{
}

int SimpleUART::read(char message_in[], uint32_t message_length)
{
    return uart_read_bytes(_port, message_in, message_length, 10 / portTICK_PERIOD_MS);
}

void SimpleUART::write(char message_out[], uint32_t message_length)
{
    uart_write_bytes(_port, message_out, message_length);
}

int SimpleUART::available()
{
    int buffered_size;
    uart_get_buffered_data_len(_port, (size_t*)&buffered_size);
    return buffered_size;
}