#pragma once
#include "mapper.h"

enum class Mirroring {
  HORIZONTAL, // 0
  VERTICAL,   // 1
  FOUR_SCREEN,
  SINGLE_SCREEN_LOWER,
  SINGLE_SCREEN_UPPER
};

class Mapper0 : public Mapper{
public:
  Mapper0(
    const std::vector<uint8_t> &prg, const std::vector<uint8_t> &chr,
    const Mirroring& mirroring);

  uint8_t readPRG(uint16_t addr) override;
  void writePRG(uint16_t addr, uint8_t value) override;

  uint8_t readCHR(uint16_t addr) override;
  void writeCHR(uint16_t addr, uint8_t value) override;
  uint16_t mirrorVRAM(uint16_t address) override;

private:
  std::vector<uint8_t> prgROM_;
  std::vector<uint8_t> chrROM_;
  Mirroring mirroring_;
};
