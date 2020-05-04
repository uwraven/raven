#include "FlightLink.h"
#include "Packet.h"

FlightLink::FlightLink()
{
    this.dataSize = 0;
    this.counter = 0;
    this.validPacket = 0;
    this.malFormed = false;
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

void FlightLink::sendPacket(Serial* stream, Packet* packet) {
    stream->write(0x01);
    stream->write(packet->header);
    stream->write(packet->size);
    for (i = 0; i < packet->size; i++)
    {
        union {
            float value;
            byte b[4];
        } fl;
        fl.value = packet->buffer[i];
        for (j = 0; j < 4; j++)
        {
            stream->write(fl.b[j]);
        }
    }
    stream->write(0x04);
}

bool FlightLink::receivePacket(Serial* stream, Packet* packet) {

    byte read;
    while (serial->available() > 0) {
        read = serial->read();
        if (this.validPacket) {
            if (counter == 0) {
                packet->header = read;
            } else if (counter == 1) {
                packet->length = read;
            } else if (counter == packet->length * 4 + 2) {
                this.validPacket = false;
                this.counter = 0;
                this.dataSize = 0;
                if (read != 0x04) {
                    this.malFormed = true;
                    return false;
                } else {
                    this.malFormed = false;
                    return true;
                }
            } else {
                dataBuffer[this.dataSize] = read;
                this.dataSize++;
                if (this.dataSize == 4) {
                    union {
                        float value;
                        byte b[4];
                    } fl;
                    fl.b = this.dataBuffer;
                    packet->pushFloat(fl.value);
                    this.dataSize = 0;
                }
            }
            this.counter++;
        } else if (read == 0x01) {
            this.validPacket = true;
        }
    }
    return false;
}