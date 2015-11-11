#ifndef __BMC_H__
#define __BMC_H__

#include <inttypes.h>

// TX
void bmc_begin_tx(uint8_t pin, uint16_t rate);
void bmc_sendData(uint32_t data);

#ifdef BMC_ENABLE_RX
// RX
void bmc_begin_rx(uint8_t pin, uint16_t rate);
uint8_t bmc_hasData();
uint32_t bmc_readData();

void _bmc_rx_isr();

ISR(TIMER1_COMPA_vect) {
	_bmc_rx_isr();
}
#endif

#endif
