#pragma once
#include "mapper.h"

class Mapper0 : public Mapper{
public:
  Mapper0(const std::vector<uint8_t> &prg, const std::vector<uint8_t> &chr);

  uint8_t readPRG(uint16_t addr) override;
  void writePRG(uint16_t addr, uint8_t value) override;

  uint8_t readCHR(uint16_t addr) override;
  void writeCHR(uint16_t addr, uint8_t value) override;

private:
  std::vector<uint8_t> prgROM_;
  std::vector<uint8_t> chrROM_;
};
