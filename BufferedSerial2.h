
/**
 * @file    BufferedSerial2.h
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

#ifndef BUFFEREDSERIAL2_H
#define BUFFEREDSERIAL2_H

#include "mbed_version.h"
#include "RawSerial.h"
#include "Stream.h"
#include "NonCopyable.h"
#include "CircularBuffer2.h"

#if !defined(BUFFEREDSERIAL2_TX_SIZE)
#define BUFFEREDSERIAL2_TX_SIZE 0x200
#endif

#if !defined(BUFFEREDSERIAL2_RX_SIZE)
#define BUFFEREDSERIAL2_RX_SIZE 0x100
#endif

#if (MBED_MAJOR_VERSION == 5) && (MBED_MINOR_VERSION >= 2)
#elif (MBED_MAJOR_VERSION == 2) && (MBED_PATCH_VERSION > 130)
#else
#error "BufferedSerial version 13 and newer requires use of Mbed OS 5.2.0 and newer or Mbed 2 version 130 and newer. Use BufferedSerial version 12 and older or upgrade the Mbed version."
#endif

/** A serial port (UART) for communication with other serial devices
 *
 * Can be used for Full Duplex communication, or Simplex by specifying
 * one pin as NC (Not Connected)
 *
 * Example:
 * @code
 *  #include "mbed.h"
 *  #include "BufferedSerial.h"
 *
 *  BufferedSerial pc(USBTX, USBRX);
 *
 *  int main()
 *  { 
 *      while(1)
 *      {
 *          Timer s;
 *        
 *          s.start();
 *          pc.printf("Hello World - buffered\n");
 *          int buffered_time = s.read_us();
 *          wait(0.1f); // give time for the buffer to empty
 *        
 *          s.reset();
 *          printf("Hello World - blocking\n");
 *          int polled_time = s.read_us();
 *          s.stop();
 *          wait(0.1f); // give time for the buffer to empty
 *        
 *          pc.printf("printf buffered took %d us\n", buffered_time);
 *          pc.printf("printf blocking took %d us\n", polled_time);
 *          wait(0.5f);
 *      }
 *  }
 * @endcode
 */

/**
 *  @class BufferedSerial
 *  @brief Software buffers and interrupt driven tx and rx for Serial
 */
class BufferedSerial2 : public mbed::RawSerial, public mbed::Stream, private mbed::NonCopyable<BufferedSerial2>
{
private:
    mbed::CircularBuffer2<char> _rxbuf;
    mbed::CircularBuffer2<char> _txbuf;
    bool m_block_on_full;
 
    void rxIrq(void);
    void txIrq(void);
    void prime(void);
    
public:
    /** Create a BufferedSerial port, connected to the specified transmit and receive pins
     *  @param tx Transmit pin
     *  @param rx Receive pin
     *  @param buf_size printf() buffer size
     *  @param tx_multiple amount of max printf() present in the internal ring buffer at one time
     *  @param name optional name
     *  @note Either tx or rx may be specified as NC if unused
     */
    BufferedSerial2(PinName tx, PinName rx, char *rx_buf, size_t rx_buf_size, char *tx_buf, size_t tx_buf_size, int baud = MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE, bool block_on_full=true);
    
    /** Destroy a BufferedSerial port
     */
    virtual ~BufferedSerial2(void);
    
    /** Check on how many bytes are in the rx buffer
     *  @return 1 if something exists, 0 otherwise
     */
    bool readable() const;
    
    /** Check to see if the tx buffer has room
     *  @return 1 always has room and can overwrite previous content if too small / slow
     */
    virtual int writeable(void);
    
    /** Get a single byte from the BufferedSerial Port.
     *  Should check readable() before calling this.
     *  @return A byte that came in on the Serial Port
     */
    virtual int getc(void);
    
    /** Write a single byte to the BufferedSerial Port.
     *  @param c The byte to write to the Serial Port
     *  @return The byte that was written to the Serial Port Buffer
     */
    virtual int putc(int c);
    
    /** Write a string to the BufferedSerial Port. Must be NULL terminated
     *  @param s The string to write to the Serial Port
     *  @return The number of bytes written to the Serial Port Buffer
     */
    virtual int puts(const char *s);
    
    /** Write a formatted string to the BufferedSerial Port.
     *  @param format The string + format specifiers to write to the Serial Port
     *  @return The number of bytes written to the Serial Port Buffer
     */
    using Stream::printf;
    using Stream::vprintf;
    
    /** Write data to the Buffered Serial Port
     *  @param s A pointer to data to send
     *  @param length The amount of data being pointed to
     *  @return The number of bytes written to the Serial Port Buffer
     */
    virtual ssize_t write(const void *s, std::size_t length);

    virtual int sync() {while(!_txbuf.empty()); return 0;}

    virtual int _putc(int c) {return putc(c);}
    virtual int _getc() {
        char c = 0;
        _rxbuf.pop(c);
        return c;
    }

    virtual short poll(short events) const {
        return _rxbuf.empty() ? 0 : POLLIN;
    }
};

#endif
