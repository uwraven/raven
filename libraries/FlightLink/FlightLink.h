#include "Arduino.h"
#inlcude "Packet.h"

class FlightLink {
   public:
    FlightLink();

    bool init();

    void writeOutput(Serial stream);
    void sendPacket(Serial* stream, Packet* packet);
    bool receivePacket(Serial* stream, Packet* packet);

   private:
    bool malFormed;
    int dataSize;
    int counter;
    bool validPacket;
    byte dataBuffer[4];
    int lastIndex;
};