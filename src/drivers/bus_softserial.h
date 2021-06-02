#include "targets.h"
#include "defines.h"
#include "time.h"




typedef struct SoftSerialData_s
{
    GPIO_TypeDef* tx_port;
    uint16_t tx_pin;
    GPIO_TypeDef* rx_port;
    uint16_t rx_pin;
    uint32_t baud;
    uint32_t micros_per_bit;
    uint32_t micros_per_bit_half;
} SoftSerialData_t;

SoftSerialData_t softserial_init(GPIO_TypeDef* tx_port, uint16_t tx_pin, GPIO_TypeDef* rx_port, uint16_t rx_pin,  uint32_t baudrate);
int softserial_read_byte(uint8_t* byte);
void softserial_write_byte(uint8_t byte);
int softserial_read_byte_ex(const SoftSerialData_t* data, uint8_t* byte);
void softserial_write_byte_ex(const SoftSerialData_t* data, uint8_t byte);
void softserial_set_input(const SoftSerialData_t* data);
void softserial_set_output(const SoftSerialData_t* data);

inline void delay_until(uint32_t uS )
{
    while (gettime() < uS) ;
}




