#include <serial/serial.h>
#include <iostream>
#include "utils.h"

void utils::loadInt32ToUint8Array(const int32_t* src, uint8_t* dst)
{
    const size_t size = 4;
    size_t i = 0;

    for (i = 0; i < size; i++)
	{
		*(dst + i) = *((uint8_t*)src + 3 - i);
	}

    return;
}

void utils::loadUint32ToUint8Array(const uint32_t* src, uint8_t* dst)
{
    const size_t size = 4;
    size_t i = 0;

    for (i = 0; i < size; i++)
	{
		*(dst + i) = *((uint8_t*)src + 3 - i);
	}

    return;
}

void utils::loadUint8ArrayToUint16(const uint8_t* src, uint16_t* dst)
{
	*((uint8_t*)dst) = *(src + 1);
    *((uint8_t*)dst + 1) = *src;

    return;
}

void utils::loadUint8ArrayToFloat32(const uint8_t* src, float* dst)
{
    const size_t size = 4;
    size_t i = 4;

    for (i = 0; i < size; i++)
	{
		*((uint8_t*)dst + i) = *(src + i);
	}

    return;
}

void utils::addCRC16(const uint8_t* src, uint8_t* dst, const size_t size)
{
	uint16_t crc = 0xffff; // CRC16-MODBUS.
    size_t i = 0, j = 0;

	//Add check.
    for (i = 0; i < size; i++)
	{
		*(dst + i) = *(src + i);
	}

	for (i = 0; i < size; i++)
	{
		crc ^= (dst[i] & 0x00ff);
		for (j = 0; j < 8; j++)
		{
			if (crc & 0x01)
			{
				crc >>= 1;
				crc ^= 0xa001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	*(dst + size) = crc;
	*(dst + size + 1) = crc >> 8;

    return;
}

void utils::dispUint8Array(const uint8_t* src, const size_t size,
						   const std::string prefix)
{
	size_t i = 0;

	std::cout << prefix;
	for (i = 0; i < size; i++)
	{
		printf("%02x ", src[i]);
	}
	std::cout << "." << std::endl << std::endl;
}
