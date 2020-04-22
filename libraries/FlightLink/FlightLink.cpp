#include "FlightLink.h"

FlightLink::FlightLink()
{
    // constructor
}

bool FlightLink::init()
{
    return true;
}

void FlightLink::writeOutput(Serial stream)
{
    stream.write(0x01);
    stream.write(0x18);
    stream.write(bufferLength);
    stream.write(0x40);
    for (i = 0; i < bufferLength; i++)
    {
        union {
            float value;
            byte b[4];
        } fl;
        fl.value = outputBuffer[i];
        for (j = 0; j < 4; j++)
        {
            stream.write(fl.b[j]);
        }
    }
    stream.write(0x41);
    stream.write(0x04);
    lastIndex = 0;
}

void pushToDataOuputBuffer(float value)
{
    lastIndex++;
    outputBuffer[lastIndex] = value;
}