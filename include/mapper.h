// mapper.h
#pragma once
#include <cstdint>
#include <vector>

class Mapper{
public:
  Mapper() = default;
  virtual ~Mapper() = default;

  virtual uint8_t readPRG(uint16_t addr) = 0;
  virtual void writePRG(uint16_t addr, uint8_t value) = 0;

  virtual uint8_t readCHR(uint16_t addr) = 0;
  virtual void writeCHR(uint16_t addr, uint8_t value) = 0;
};
