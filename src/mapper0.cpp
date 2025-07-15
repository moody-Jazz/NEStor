#include "mapper0.h"

Mapper0::Mapper0(const std::vector<uint8_t> &prg, const std::vector<uint8_t> &chr)
  : prgROM_(prg), chrROM_(chr)
  {}

uint8_t Mapper0::readPRG(uint16_t addr){
  if (prgROM_.size() == 0x4000)    // 16KB
    return prgROM_[addr % 0x4000]; // mirror into 0x8000–0xFFFF
  else
    return prgROM_[addr - 0x8000]; // full 32KB
}

void Mapper0::writePRG(uint16_t, uint8_t){
  // NROM is read-only
}

uint8_t Mapper0::readCHR(uint16_t addr){
  return chrROM_[addr];
}

void Mapper0::writeCHR(uint16_t addr, uint8_t value){
  if (!chrROM_.empty())
    chrROM_[addr] = value; // if using CHR-RAM
}

