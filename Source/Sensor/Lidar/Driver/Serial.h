#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdio.h>
#include <string.h>
#include <unistd.h>

namespace NS_Selidar
{
  
  class Serial
  {
  public:
    
    enum
    {
      ANS_OK = 0, ANS_TIMEOUT = -1, ANS_DEV_ERR = -2,
    };

    enum
    {
      SERIAL_RX_BUFFER_SIZE = 512, SERIAL_TX_BUFFER_SIZE = 128,
    };

    Serial ();
    ~Serial ();
    bool
    bind (const char * portname, unsigned int baudrate, unsigned int flags = 0);
    bool
    open ();
    void
    close ();
    void
    flush (unsigned int flags);

    int
    waitfordata (size_t data_count, unsigned int timeout = -1,
                 size_t * returned_size = NULL);

    int
    senddata (const unsigned char * data, size_t size);
    int
    recvdata (unsigned char * data, size_t size);

    int
    waitforsent (unsigned int timeout = -1, size_t * returned_size = NULL);
    int
    waitforrecv (unsigned int timeout = -1, size_t * returned_size = NULL);

    size_t
    rxqueue_count ();

    void
    setDTR ();
    void
    clearDTR ();

    bool
    isOpened ()
    {
      return _is_serial_opened;
    }
    
    unsigned int
    getTermBaudBitmap (unsigned int baud);

  private:
    bool
    open (const char * portname, unsigned int baudrate, unsigned int flags = 0);
    void
    _init ();

    bool _is_serial_opened;

    char _portName[200];
    unsigned int _baudrate;
    unsigned int _flags;

    int serial_fd;

    size_t required_tx_cnt;
    size_t required_rx_cnt;
  };

}

#endif
