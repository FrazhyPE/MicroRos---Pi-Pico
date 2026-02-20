#include "SCSerial.h"
#include "pico/stdlib.h"
#include "pico/time.h"

SCSerial::SCSerial() : SCS()
{
  IOTimeOut = 50;
  Err = 0;
}

SCSerial::SCSerial(u8 End) : SCS(End)
{
  IOTimeOut = 50;
  Err = 0;
}

SCSerial::SCSerial(u8 End, u8 Level) : SCS(End, Level)
{
  IOTimeOut = 50;
  Err = 0;
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen){
  if(!uart) return 0;
  uart_write_blocking(uart, nDat, nLen);
  return nLen;
}

int SCSerial::writeSCS(unsigned char bDat){
  if(!uart) return 0;
  uart_putc_raw(uart, bDat);
  return 1;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen){
  if(!uart) return 0;

  int got = 0;
  absolute_time_t t0 = get_absolute_time();

  while(got < nLen){
    if(uart_is_readable(uart)){
      nDat[got++] = (unsigned char)uart_getc(uart);
    } else {
      // timeout simple basado en IOTimeOut (ms)
      if(absolute_time_diff_us(t0, get_absolute_time()) > (int64_t)IOTimeOut * 1000){
        break;
      }
      tight_loop_contents();
    }
  }
  return got;
}

void SCSerial::rFlushSCS(){
  if(!uart) return;
  while(uart_is_readable(uart)) (void)uart_getc(uart);
}

void SCSerial::wFlushSCS(){
  // Pico SDK no tiene flush TX directo; normalmente no se requiere.
}
