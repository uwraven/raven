#include "Arduino.h"

class FlightLink {
   public:
    FlightLink();

    bool init();

    void writeOutput(Serial stream);

   private:
    int bufferLength = 60;
    float outputBuffer[bufferLength];
    int lastIndex;
};