
#ifndef _SELIDAR_PROTOCOL_H_
#define _SELIDAR_PROTOCOL_H_

namespace NS_Selidar{
// SE-Lidar Input Packets

#define SELIDAR_CMD_SYNC_BYTE        0xA5
//#define SELIDAR_CMDFLAG_HAS_PAYLOAD  0x80
 //@test
#define SELIDAR_CMDFLAG_HAS_PAYLOAD 0xF0

#define SELIDAR_ANS_SYNC_BYTE1       0xA5
#define SELIDAR_ANS_SYNC_BYTE2       0x5A

#define SELIDAR_ANS_PKTFLAG_LOOP     0x1

#define SELIDAR_ANS_HEADER_SIZE_MASK        0x3FFFFFFF
#define SELIDAR_ANS_HEADER_SUBTYPE_SHIFT    (30)

typedef struct _selidar_cmd_packet_t {
    unsigned char syncByte; //must be SELIDAR_CMD_SYNC_BYTE
    unsigned char cmd_flag; 
    unsigned char size;
    unsigned char data[0];
} __attribute__((packed)) selidar_cmd_packet_t;


typedef struct _selidar_ans_header_t {
    unsigned char  syncByte1; // must be SELIDAR_ANS_SYNC_BYTE1
    unsigned char  syncByte2; // must be SELIDAR_ANS_SYNC_BYTE2
    unsigned int size_q30_subtype; // see unsigned int size:30; unsigned int subType:2;
    unsigned char  type;
} __attribute__((packed)) selidar_ans_header_t;

}

#endif
