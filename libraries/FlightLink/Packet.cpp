#include "Arduino.h"
#include "Packet.h"

Packet::Packet(MessageType header, byte length) {
    this.header = header;
    this.length = length;
    this.size = 0;
}

bool Packet::pushData(float data[]) {
    if (this.size + data.size() > 248) {
        return false;
    }
    for (int i = 0; i < data.size(); i++) {
        buffer[i + this.size] = data[i];
    }
    this.size += data.size();
    return true;
}

bool Packet::pushFloat(float val) {
    if (this.size + 1 > 248) {
        return false;
    }
    buffer[this.size] = val;
    this.size++;
    return true;
}

void Packet::clearBuffer() {
    this.size = 0;
}