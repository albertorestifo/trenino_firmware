#pragma once
#include <stddef.h>
#include <stdint.h>
#include <vector>
// Native transport boundary: record actual encoded messages sent by handlers.
struct COBS {};
template <typename Encoding> class PacketSerial_ {
public:
    std::vector<std::vector<uint8_t>> packets;
    void send(const uint8_t* bytes, size_t count) { packets.emplace_back(bytes, bytes + count); }
};
