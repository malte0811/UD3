#ifndef MIN_ID_H
#define MIN_ID_H

#define MIN_ID_TERM     0
#define MIN_ID_WD       10
#define MIN_ID_RESET    11
#define MIN_ID_COMMAND  12
#define MIN_ID_SOCKET   13
#define MIN_ID_SYNTH    14
#define MIN_ID_FEATURE  15
#define MIN_ID_MIDI     20
#define MIN_ID_SID      21
// QCW ramp is 400 bytes, so does not fit in a single MIN frame. Message format:
// 2 bytes offset; the highest bit indicates last frame of ramp
// Rest of message is raw ramp data (uint8)
// This can be send both ways, UD3->TT is for display, TT->UD3 is for setting custom ramps
#define MIN_ID_QCW_RAMP 22
    
#define MIN_ID_EVENT    40
#define MIN_ID_ALARM    41
#define MIN_ID_DEBUG    42
#define MIN_ID_VMS      43

#define SYNTH_CMD_FLUSH     1
#define SYNTH_CMD_SID       2
#define SYNTH_CMD_MIDI      3
#define SYNTH_CMD_OFF       4

    
#define SOCKET_DISCONNECTED  0
#define SOCKET_CONNECTED     1
    
#define CMD_HELLO_WORLD     0x01    
#define CMD_FEATURE_FRAME   0x02
#define CMD_LINK            0x03
    
#define EVENT_GET_INFO          1 
#define EVENT_ETH_INIT_FAIL     2    
#define EVENT_ETH_INIT_DONE     3
#define EVENT_ETH_LINK_UP       4
#define EVENT_ETH_LINK_DOWN     5
#define EVENT_ETH_DHCP_SUCCESS  6
#define EVENT_ETH_DHCP_FAIL     7
#define EVENT_FS_CARD_CONNECTED 8
#define EVENT_FS_CARD_REMOVED   9

    
#define EVENT_STRUCT_VERSION    1
#define OS_INFO_STRUCT_VERSION    1
    
#endif
