/**
 * @file    BufferedSerial2.cpp
 * @brief   Software Buffer - Extends mbed Serial functionallity adding irq driven TX and RX
 * @author  sam grove
 * @version 1.0
 * @see
 *
 * Copyright (c) 2013
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "BufferedSerial2.h"
#include "Serial.h"

using namespace mbed;

BufferedSerial2::BufferedSerial2(PinName tx, PinName rx, char *rx_buf, size_t rx_buf_size, char *tx_buf, size_t tx_buf_size, int baud, bool block_on_full)
    : _rxbuf(rx_buf, rx_buf_size), _txbuf(tx_buf, tx_buf_size), RawSerial(tx, rx, baud), m_block_on_full(block_on_full)
{
    RawSerial::attach(callback(this, &BufferedSerial2::rxIrq), Serial::RxIrq);
    return;
}

BufferedSerial2::~BufferedSerial2(void)
{
    RawSerial::attach(NULL, RawSerial::RxIrq);
    RawSerial::attach(NULL, RawSerial::TxIrq);

    return;
}

bool BufferedSerial2::readable() const
{
    return _rxbuf.empty() ? false : true;  // note: look if things are in the buffer
}

int BufferedSerial2::writeable(void)
{
    return 1;   // buffer allows overwriting by design, always true
}

int BufferedSerial2::getc(void)
{
    char c = 0;
    _rxbuf.pop(c);
    return c;
}

int BufferedSerial2::putc(int c)
{
    while (m_block_on_full && _txbuf.full());
    _txbuf.push((char)c);
    BufferedSerial2::prime();

    return c;
}

int BufferedSerial2::puts(const char *s)
{
    if (s != NULL) {
        const char* ptr = s;
    
        while(*(ptr) != 0) {
            while (m_block_on_full && _txbuf.full());
            _txbuf.push(*(ptr++));
        }
        _txbuf.push('\n');  // done per puts definition
        BufferedSerial2::prime();
    
        return (ptr - s) + 1;
    }
    return 0;
}

ssize_t BufferedSerial2::write(const void *s, size_t length)
{
    if (s != NULL && length > 0) {
        const char* ptr = (const char*)s;
        const char* end = ptr + length;
    
        while (ptr != end) {
            while (m_block_on_full && _txbuf.full());
            _txbuf.push(*(ptr++));
        }
        BufferedSerial2::prime();
    
        return ptr - (const char*)s;
    }
    return 0;
}


void BufferedSerial2::rxIrq(void)
{
    // read from the peripheral and make sure something is available
    if(serial_readable(&_serial)) {
        _rxbuf.push(serial_getc(&_serial)); // if so load them into a buffer
    }

    return;
}

void BufferedSerial2::txIrq(void)
{
    // see if there is room in the hardware fifo and if something is in the software fifo
    while(serial_writable(&_serial)) {
        if(!_txbuf.empty()) {
            char c = 0;
            _txbuf.pop(c);
            serial_putc(&_serial, (int)c);
        } else {
            // disable the TX interrupt when there is nothing left to send
            RawSerial::attach(NULL, RawSerial::TxIrq);
            break;
        }
    }

    return;
}

void BufferedSerial2::prime(void)
{
    // if already busy then the irq will pick this up
    if(serial_writable(&_serial)) {
        RawSerial::attach(NULL, RawSerial::TxIrq);    // make sure not to cause contention in the irq
        BufferedSerial2::txIrq();                // only write to hardware in one place
        RawSerial::attach(callback(this, &BufferedSerial2::txIrq), RawSerial::TxIrq);
    }

    return;
}


