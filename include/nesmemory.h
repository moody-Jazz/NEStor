#pragma once

#include <array>
#include <memory>

#include "mapper.h"
#include "basememory.h"
#include "helper.h"

class NesMemory : public BaseMemory{

public:
  NesMemory(std::unique_ptr<Mapper> mapper);
  ~NesMemory();

  enum MEMORY_REGIONS{
      MAIN_RAM,      // 0x0000-0x1FFF (2KB internal RAM, mirrored)
      PPU_REGISTERS, // 0x2000-0x3FFF (8 bytes, mirrored)
      APU_REGISTERS, // 0x4000-0x401F
      CONTROLLERS,   // 0x4016-0x4017
      EXPANSION_ROM, // 0x4020-0x5FFF
      SRAM,          // 0x6000-0x7FFF
      PRG_ROM,       // 0x8000-0xFFFF
      INVALID
  }; 
  
  MEMORY_REGIONS getRegion(const uint16_t& address);
  void mirrorAddress(uint16_t &address);

  uint8_t read(uint16_t address) override;
  void write(uint16_t addresss, uint8_t value) override;

private:

  std::unique_ptr<Mapper> mapper_;
  std::array<uint8_t, 8> ppuRegisters_;
  std::array<uint8_t, 256> oam_;
  std::array<uint8_t, 31> apuRegisters_;
  uint8_t controller_;
  std::array<uint8_t, 8 * KILO_BYTE> expansionRom_;
  std::array<uint8_t, 8 * KILO_BYTE> sram_;
};