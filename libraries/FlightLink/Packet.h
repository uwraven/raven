#include "Arduino.h"

class Packet {
  public:
    Packet(MessageType header, byte length);
    MessageType header;
    byte length;
    enum MessageType: byte {
        COMMAND = 0x17,
        DATA = 0x18,
        DC3 = 0x19,
        DC4 = 0x20,
        ACK = 0x06
    };
    int size;
    float buffer[248];

    bool pushData(float data[]);
    bool pushFloat(float val);

    void clearBuffer();
}