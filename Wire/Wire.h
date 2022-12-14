/*
 * Wire.h
 *
 *  Created on: 2022-06-28
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#ifndef WIRE_WIRE_H_
#define WIRE_WIRE_H_

class TwoWire
{
 public:
  TwoWire();
  ~TwoWire();
  void begin();
  void begin(uint8_t);
  void begin(int);
  void end();
  void setClock(uint32_t);
  void beginTransmission(uint8_t);
  void beginTransmission(int);
  uint8_t endTransmission(void);
  uint8_t endTransmission(uint8_t);
  uint8_t requestFrom(uint8_t, uint8_t);
  uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
  uint8_t requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, uint8_t);
  uint8_t requestFrom(int, int);
  uint8_t requestFrom(int, int, int);
  size_t write(uint8_t);
  size_t write(const uint8_t *, size_t);
  int available(void);
  int read(void);
  int peek(void);
  void flush(void);
  void onReceive(void (*)(int));
  void onRequest(void (*)(void));

  size_t write(unsigned long n) {
    return write((uint8_t) n);
  }
  size_t write(long n) {
    return write((uint8_t) n);
  }
  size_t write(unsigned int n) {
    return write((uint8_t) n);
  }
  size_t write(int n) {
    return write((uint8_t) n);
  }

 private:
  enum {BUF_SIZE = 32};

  cyhal_i2c_t *i2c;
  uint8_t addr, n_bytes_avail, n_bytes_to_send;
  uint8_t recv_buf[BUF_SIZE], send_buf[BUF_SIZE];
  uint8_t *pbyte_to_recv, *pbyte_to_send;

  void (*on_request_cb)(void);
  void (*on_receive_cb)(int);
  void onRequestService(void);
  void onReceiveService(uint8_t *, int);
};

extern TwoWire Wire;


#endif /* WIRE_WIRE_H_ */
