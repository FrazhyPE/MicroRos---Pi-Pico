#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#include <stdint.h>

#include "SCS.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "hardware/uart.h"
#ifdef __cplusplus
}
#endif

class SCSerial : public SCS
{
public:
  SCSerial();
  SCSerial(u8 End);
  SCSerial(u8 End, u8 Level);

  // NUEVO: asignar UART del Pico
  void setUart(uart_inst_t *u) { uart = u; }

protected:
  virtual int writeSCS(unsigned char *nDat, int nLen);
  virtual int readSCS(unsigned char *nDat, int nLen);
  virtual int writeSCS(unsigned char bDat);
  virtual void rFlushSCS();
  virtual void wFlushSCS();

public:
  unsigned long int IOTimeOut;
  int Err;

  virtual int getErr(){ return Err; }

private:
  uart_inst_t *uart = nullptr;   // <- reemplaza HardwareSerial*
};

#endif
