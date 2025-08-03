#include <fstream>
#include <stdexcept>

#include "cartridge.h"

std::vector<uint8_t> loadRom(const std::string &filepath){
  std::ifstream file(filepath, std::ios::binary | std::ios::ate);
  if (!file)
    throw std::runtime_error("Failed to open ROM file: " + filepath);

  std::streamsize size = file.tellg();
  if (size < 16)
    throw std::runtime_error("ROM too small to contain valid iNES header.");

  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> buffer(size);
  if (!file.read(reinterpret_cast<char *>(buffer.data()), size))
    throw std::runtime_error("Failed to read ROM data.");

  return buffer;
}

INesHeader parseHeader(const std::vector<uint8_t> &romData) {
  if (romData[0] != 'N' || romData[1] != 'E' || romData[2] != 'S' ||
      romData[3] != 0x1A)
    throw std::runtime_error("Invalid iNES header");

  uint8_t flag6 = romData[6];
  uint8_t flag7 = romData[7];

  uint8_t mapperLow = (flag6 >> 4);
  uint8_t mapperHigh = (flag7 >> 4);
  uint8_t mapperID = (mapperHigh << 4) | mapperLow;

  Mirroring mirroring;
  if (flag6 & 0x08)
    mirroring = Mirroring::FOUR_SCREEN;
  else if (flag6 & 0x01)
    mirroring = Mirroring::VERTICAL;
  else
    mirroring = Mirroring::HORIZONTAL;

  return {.prgRomSize = romData[4],
          .chrRomSize = romData[5],
          .mapperNumber = mapperID,
          .mirroring = mirroring
        };
}

std::vector<uint8_t> extractPRG(const std::vector<uint8_t> &romData,
                                uint8_t prgSize) {
  size_t offset = 16;  // skip header
  return std::vector<uint8_t>(
      romData.begin() + offset,
      romData.begin() + offset + prgSize * 16 * KILO_BYTE);
}

std::vector<uint8_t> extractCHR(const std::vector<uint8_t> &romData,
                                uint8_t prgSize, uint8_t chrSize) {
  size_t offset = 16 + prgSize * 16 * KILO_BYTE;
  return std::vector<uint8_t>(
      romData.begin() + offset,
      romData.begin() + offset + chrSize * 8 * KILO_BYTE);
}

std::unique_ptr<Mapper> createMapper(const std::vector<uint8_t> &romData) {
  INesHeader header = parseHeader(romData);

  // Extract PRG and CHR ROM
  auto prg = extractPRG(romData, header.prgRomSize);
  auto chr = extractCHR(romData, header.prgRomSize, header.chrRomSize);

  switch (header.mapperNumber) {
    case 0:
      return std::make_unique<Mapper0>(prg, chr, header.mirroring);
    default:
      throw std::runtime_error("Unsupported mapper ID: " +
                               std::to_string(header.mapperNumber));
  }
  
}