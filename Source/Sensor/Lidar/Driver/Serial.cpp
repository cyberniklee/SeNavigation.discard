
#include "Serial.h"

#include <iostream>
#include <termios.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/ioctl.h>

using std::cin;
using std::cout;
using std::endl;

namespace NS_Selidar{

Serial::Serial()
    : _baudrate(0)
    , _flags(0)
    , serial_fd(-1)
    , _is_serial_opened(false)
{
    _init();
}

Serial::~Serial()
{
    close();

}

bool Serial::open()
{
    return open(_portName, _baudrate, _flags);
}

bool Serial::bind(const char * portname, unsigned int baudrate, unsigned int flags)
{   
    strncpy(_portName, portname, sizeof(_portName));
    _baudrate = baudrate;
    _flags    = flags;
    return true;
}

bool Serial::open(const char * portname, unsigned int baudrate, unsigned int flags)
{
    if (isOpened()) close();
    
    serial_fd = ::open(portname, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_fd == -1)
    {
      return false;
    }

    struct termios options, oldopt;
    tcgetattr(serial_fd, &oldopt);
    bzero(&options,sizeof(struct termios));

    unsigned int termbaud = getTermBaudBitmap(baudrate);

    if (termbaud == (unsigned int)-1) {
        close();
        return false;
    }
    cfsetispeed(&options, termbaud);
    cfsetospeed(&options, termbaud);
    
    // enable rx and tx
    options.c_cflag |= (CLOCAL | CREAD);


    options.c_cflag &= ~PARENB; //no checkbit
    options.c_cflag &= ~CSTOPB; //1bit stop bit

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; /* Select 8 data bits */

#ifdef CNEW_RTSCTS
    options.c_cflag &= ~CNEW_RTSCTS; // no hw flow control
#endif

    options.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control

    // raw input mode   
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // raw output mode   
    options.c_oflag &= ~OPOST;
    
    tcflush(serial_fd,TCIFLUSH); 

    if (fcntl(serial_fd, F_SETFL, FNDELAY))
    {
        close();
        return false;
    }
    if (tcsetattr(serial_fd, TCSANOW, &options))
    {
        close();
        return false;
    }

    _is_serial_opened = true;

    //Clear the DTR bit to let the motor spin
    clearDTR();
    
    return true;
}

void Serial::close()
{
    if (serial_fd != -1)
        ::close(serial_fd);
    serial_fd = -1;
    
    _is_serial_opened = false;
}

int Serial::senddata(const unsigned char * data, size_t size)
{
// FIXME: non-block io should be used
    if (!isOpened()) return 0;

    if (data == NULL || size ==0) return 0;
    
    size_t tx_len = 0;
    required_tx_cnt = 0;
    do {
        int ans = ::write(serial_fd, data + tx_len, size-tx_len);

        if (ans == -1) return tx_len;
        
        tx_len += ans;
        required_tx_cnt = tx_len;
    }while (tx_len<size);

    return tx_len;
}


int Serial::recvdata(unsigned char * data, size_t size)
{
    if (!isOpened()) return 0;
    
    int ans = ::read(serial_fd, data, size);
    
    if (ans == -1) ans=0;
    required_rx_cnt = ans;
    return ans;
}

void Serial::flush( unsigned int flags)
{
    tcflush(serial_fd,TCIFLUSH); 
}

int Serial::waitforsent(unsigned int timeout, size_t * returned_size)
{
    if (returned_size) *returned_size = required_tx_cnt;
    return 0;
}

int Serial::waitforrecv(unsigned int timeout, size_t * returned_size)
{
    if (!isOpened() ) return -1;
   
    if (returned_size) *returned_size = required_rx_cnt;
    return 0;
}

int Serial::waitfordata(size_t data_count, unsigned int timeout, size_t * returned_size)
{
    size_t length = 0;
    if (returned_size==NULL) returned_size=(size_t *)&length;
    *returned_size = 0;

    int max_fd;
    fd_set input_set;
    struct timeval timeout_val;

    /* Initialize the input set */
    FD_ZERO(&input_set);
    FD_SET(serial_fd, &input_set);
    max_fd = serial_fd + 1;

    /* Initialize the timeout structure */
    timeout_val.tv_sec = timeout / 1000;
    timeout_val.tv_usec = (timeout % 1000) * 1000;

    if ( isOpened() )
    {
        if ( ioctl(serial_fd, FIONREAD, returned_size) == -1)
        {
        	return ANS_DEV_ERR;
        }
       
        if (*returned_size >= data_count)  // @test, indicate already get enough data to return
        {
            return ANS_OK;
        }
    }
    while ( isOpened() )
    {
        /* Do the select */
        int n = ::select(max_fd, &input_set, NULL, NULL, &timeout_val);
        if (n < 0)
        {
            // select error
            return ANS_DEV_ERR;
        }
        else if (n == 0)
        {
            // time out
            return ANS_TIMEOUT;
        }
        else
        {
            // data avaliable
            assert (FD_ISSET(serial_fd, &input_set));

            if ( ioctl(serial_fd, FIONREAD, returned_size) == -1)
            {
            	return ANS_DEV_ERR;
            }
            if (*returned_size >= data_count)
            {
                return ANS_OK;
            }
            else 
            {         
                int remain_timeout = timeout_val.tv_sec*1000000 + timeout_val.tv_usec;
                int expect_remain_time = (data_count - *returned_size)*1000000*8/_baudrate;
                if (remain_timeout > expect_remain_time)
                    usleep(expect_remain_time);
            }
        }
        
    }

    return ANS_DEV_ERR;
}

size_t Serial::rxqueue_count()
{
    if  ( !isOpened() ) return 0;
    size_t remaining;
    
    if (::ioctl(serial_fd, FIONREAD, &remaining) == -1) return 0;
    return remaining;
}

void Serial::setDTR()
{
    if ( !isOpened() ) return;

    unsigned int dtr_bit = TIOCM_DTR;
    ioctl(serial_fd, TIOCMBIS, &dtr_bit);
}

void Serial::clearDTR()
{
    if ( !isOpened() ) return;

    unsigned int dtr_bit = TIOCM_DTR;
    ioctl(serial_fd, TIOCMBIC, &dtr_bit);
}

void Serial::_init()
{
    serial_fd = 0;  
    _portName[0] = 0;
    required_tx_cnt = required_rx_cnt = 0;
}



unsigned int Serial::getTermBaudBitmap(unsigned int baud)
{
#define BAUD_CONV( _baud_) case _baud_:  return B##_baud_ 
switch (baud) {
        BAUD_CONV(1200);
        BAUD_CONV(1800);
        BAUD_CONV(2400);
        BAUD_CONV(4800);
        BAUD_CONV(9600);
        BAUD_CONV(19200);
        BAUD_CONV(38400);
        BAUD_CONV(57600);
        BAUD_CONV(115200);
        BAUD_CONV(230400);
        BAUD_CONV(460800);
        BAUD_CONV(500000);
        BAUD_CONV(576000);
        BAUD_CONV(921600);
        BAUD_CONV(1000000);
        BAUD_CONV(1152000);
        BAUD_CONV(1500000);
        BAUD_CONV(2000000);
        BAUD_CONV(2500000);
        BAUD_CONV(3000000);
        BAUD_CONV(3500000);
        BAUD_CONV(4000000);
    }
    return -1;
}

}
