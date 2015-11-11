#include <BMC.h>
#include <util/delay_basic.h>
#include <../FastArduino/FastArduino.h>

const uint8_t BMC_NSAMPLES = 8;
const uint8_t BMC_MAXL = BMC_NSAMPLES * 5 / 4; // 175%
const uint8_t BMC_MAXS = BMC_NSAMPLES * 3 / 4; //  75%
const uint8_t BMC_MINS = BMC_NSAMPLES * 1 / 4; //  25%

const uint32_t BMC_MAXTCNT = 65536;

const uint8_t BMC_STATUS_INIT = 0;
const uint8_t BMC_STATUS_IDLE = 1;
const uint8_t BMC_STATUS_SYNC = 2;
const uint8_t BMC_STATUS_READ = 3;
const uint8_t BMC_STATUS_DONE = 4;

fPin _bmc_tx_pin;
fPin _bmc_rx_pin;
uint16_t _bmc_lcycles;
volatile uint32_t _bmc_data;
volatile uint8_t _bmc_status;
uint8_t _bmc_bits = 0;
uint8_t _bmc_ticks = 0;
uint8_t _bmc_prev_value = 0;
uint8_t _bmc_transitions = 0;

void bmc_begin_tx(uint8_t pin, uint16_t rate) {
	_bmc_tx_pin = fGetPin(pin);
	fPinMode(_bmc_tx_pin, fOUTPUT);
	fDigitalWrite(_bmc_tx_pin, 0);
	_bmc_lcycles = (F_CPU / 4) / rate; // cycles for _delay_loop_2
}

void _bmc_send0() {
	fDigitalToggle(_bmc_tx_pin);
	_delay_loop_2(_bmc_lcycles);
}

void _bmc_send1() {
	uint16_t scycles = _bmc_lcycles/2;
	fDigitalToggle(_bmc_tx_pin);
	_delay_loop_2(scycles);
	fDigitalToggle(_bmc_tx_pin);
	_delay_loop_2(scycles);
}

void _bmc_sendByte(uint8_t b) {
	uint8_t m = 0x80;
	for (uint8_t i = 0; i < 8; i++) {
		if (b & m) {
			_bmc_send1();
		} else {
			_bmc_send0();
		}
		m >>= 1;
	}
}

void bmc_sendData(uint32_t data) {
	for (uint8_t i = 0; i < 31; i++) {
		_bmc_send1();
	}
	_bmc_send0();

	_bmc_sendByte((uint8_t) (data >> 24));
	_bmc_sendByte((uint8_t) (data >> 16));
	_bmc_sendByte((uint8_t) (data >> 8));
	_bmc_sendByte((uint8_t) (data >> 0));

	_bmc_send0();
	fDigitalWrite(_bmc_tx_pin, 0);
	_delay_loop_2(_bmc_lcycles*4);
}

#ifdef TCCR1A
void bmc_begin_rx(uint8_t pin, uint16_t rate) {
	_bmc_rx_pin = fGetPin(pin);
	fPinMode(_bmc_rx_pin, fINPUT);

	uint8_t prescaler = 0;
	uint32_t cycles = (F_CPU / BMC_NSAMPLES) / rate;

	if (cycles < BMC_MAXTCNT) { // clk/1
		prescaler = 1 << CS10;
	} else if (cycles < BMC_MAXTCNT * 8) { // clk/8
		prescaler = 2 << CS10;
		cycles /= 8;
	} else if (cycles < BMC_MAXTCNT * 64) { // clk/64
		prescaler = 3 << CS10;
		cycles /= 64;
	} else if (cycles < BMC_MAXTCNT * 256) { // clk/256
		prescaler = 4 << CS10;
		cycles /= 256;
	} else if (cycles < BMC_MAXTCNT * 1024) { // clk/1024
		prescaler = 5 << CS10;
		cycles /= 1024;
	} else { // Max Time
		prescaler = 5 << CS10;
		cycles = BMC_MAXTCNT - 1;
	}

	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	TCNT1 = 0;
	TIMSK1 = 0;

	// CTC Mode
	// TCCR1A: WGM11=0 WGM10=0
	// TCCR1B: WGM12=1 WGM13=0
	OCR1A = cycles - 1;
	TCCR1B |= (1 << WGM12) | prescaler;

	// Enable interrupt on OCR1A match
	TIMSK1 |= (1 << OCIE1A);
}
#endif

uint8_t bmc_hasData() {
	return _bmc_status == BMC_STATUS_DONE;
}

uint32_t bmc_readData() {
	while(!bmc_hasData());
	uint32_t res = _bmc_data;
	_bmc_status = BMC_STATUS_INIT;
	return res;
}

void _bmc_rx_isr() {
	// If the message has been received,
	// do nothing until readed
	if (_bmc_status == BMC_STATUS_DONE) {
		return;
	}

	// Initialize variables to start a
	// new receive process
	if (_bmc_status == BMC_STATUS_INIT) {
		_bmc_data = 0;
		_bmc_ticks = 0;
		_bmc_transitions = 0;
		_bmc_bits = 0;
		_bmc_status = BMC_STATUS_IDLE;
		return;
	}

	// read the current signal level
	// detect transitions and count ticks
	uint8_t value = fDigitalRead(_bmc_rx_pin);
	uint8_t isTransition = (value != _bmc_prev_value);
	_bmc_prev_value = value;
	_bmc_ticks++;
	// If haven't detect a transition, wait more time
	if (!isTransition) {
		return;
	}

	// We have detected a transition
	_bmc_transitions++;

	// If it's the first transition in this cycle
	if (_bmc_transitions == 1) {
		if (_bmc_ticks > BMC_MAXL || _bmc_ticks < BMC_MINS) {
			// Too fast or too slow... discard message
			_bmc_status = BMC_STATUS_INIT;
			return;
		}

		if (_bmc_ticks > BMC_MAXS) {
			// It's a 0
			_bmc_bits++;
			_bmc_ticks = 0;
			_bmc_transitions = 0;

			if (_bmc_status == BMC_STATUS_SYNC) {
				// I've received more than 8 (1) and now a 0
				// This is the start condition
				_bmc_bits = 0;
				_bmc_status = BMC_STATUS_READ;
			} else if (_bmc_status == BMC_STATUS_READ) {
				_bmc_data <<= 1;
				if (_bmc_bits >= 32) {
					_bmc_status = BMC_STATUS_DONE;
				}
			} else {
				_bmc_status = BMC_STATUS_INIT;
			}
		}
	} else if (_bmc_transitions == 2) {
		if (_bmc_ticks > BMC_MAXL || _bmc_ticks < 2 * BMC_MINS) {
			// Too fast or too slow... discard message
			_bmc_status = BMC_STATUS_INIT;
			return;
		}

		// It's a 1
		_bmc_bits++;
		_bmc_ticks = 0;
		_bmc_transitions = 0;

		if (_bmc_status == BMC_STATUS_IDLE) {
			// A minimum of 8 bits (1)
			if (_bmc_bits >= 8) {
				_bmc_status = BMC_STATUS_SYNC;
			}
		} else if (_bmc_status == BMC_STATUS_SYNC) {
			// Aready in sync, do nothing
		} else if (_bmc_status == BMC_STATUS_READ) {
			_bmc_data <<= 1;
			_bmc_data |= 0x01;
			if (_bmc_bits >= 32) {
				_bmc_status = BMC_STATUS_DONE;
			}
		} else {
			_bmc_status = BMC_STATUS_INIT;
		}

	} else {
		// Too Many transitions.. discard message
		_bmc_status = BMC_STATUS_INIT;
	}
}

