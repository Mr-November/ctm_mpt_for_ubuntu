#ifndef UTILS_H
#define UTILS_H

#include <serial/serial.h>

namespace utils
{
    void loadInt32ToUint8Array(const int32_t* src, uint8_t* dst);

    void loadUint32ToUint8Array(const uint32_t* src, uint8_t* dst);

    void loadUint8ArrayToUint16(const uint8_t* src, uint16_t* dst);

    void loadUint8ArrayToFloat32(const uint8_t* src, float* dst);

    void addCRC16(const uint8_t* src, uint8_t* dst, const size_t size);
}

#endif