# pragma once

#include <cstdint>
#include <vector>

inline constexpr uint16_t KILO_BYTE = 1024;

class BaseMemory{
public:
  BaseMemory(uint32_t size);
  virtual ~BaseMemory();
  virtual uint8_t read(uint16_t address);
  virtual void write(uint16_t addresss, uint8_t value);

protected:
  std::vector<uint8_t> mainRam_;
};