// cartridge.h

#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <memory>

#include "helper.h"
#include "mapper0.h"

struct INesHeader{
  uint8_t prgRomSize; // in 16KB units
  uint8_t chrRomSize; // in 8KB units
  uint8_t mapperNumber;
  Mirroring mirroring;
};

std::vector<uint8_t> loadRom(const std::string &filepath);
INesHeader parseHeader(const std::vector<uint8_t> &romData);
std::vector<uint8_t> extractPRG(const std::vector<uint8_t> &romData, uint8_t prgSize);
std::vector<uint8_t> extractCHR(const std::vector<uint8_t> &romData, uint8_t prgSize, uint8_t chrSize);
std::unique_ptr<class Mapper> createMapper(const std::vector<uint8_t> &romData);
