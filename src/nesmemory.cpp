#include "nesmemory.h"

NesMemory::NesMemory() : BaseMemory(2 * KILO_BYTE){
  prgRom_.resize(KILO_BYTE);
  chrRom_.resize(KILO_BYTE);
}

NesMemory::~NesMemory(){}

NesMemory::MEMORY_REGIONS NesMemory::getRegion(const uint16_t &address){
  if (address >= 0x0000 && address <= 0x1FFF)
    return NesMemory::MAIN_RAM;
  if (address >= 0x2000 && address <= 0x3FFF)
    return NesMemory::PPU_REGISTERS;
  if (address >= 0x4000 && address <= 0x401F)
    return NesMemory::APU_REGISTERS;
  if (address >= 0x4016 && address <= 0x4017)
    return NesMemory::CONTROLLERS;
  if (address >= 0x4020 && address <= 0x5FFF)
    return NesMemory::EXPANSION_ROM;
  if (address >= 0x6000 && address <= 0x7FFF)
    return NesMemory::SRAM;
  if (address >= 0x8000 && address <= 0xFFFF)
    return NesMemory::PRG_ROM;
  return INVALID;
}

void NesMemory::mirrorAddress(uint16_t& address){
  switch (getRegion(address)){
  case NesMemory::MAIN_RAM:
    address &= 0x07FF;
    break;
  case NesMemory::PPU_REGISTERS:
    address &= 0x0007;
    break;
  default: break;
  }
}

uint8_t NesMemory::read(uint16_t address){
  switch (getRegion(address)){
  case MAIN_RAM:
    mirrorAddress(address);
    return mainRam_[address];
  case PPU_REGISTERS:
    mirrorAddress(address);
    return ppuRegisters_[address];
  case APU_REGISTERS:
    return apuRegisters_[address - 0x4000];
  case CONTROLLERS:
    return controller_;
  case EXPANSION_ROM:
    return expansionRom_[address - 0x4020];
  case SRAM:
    return sram_[address - 0x6000];
  case PRG_ROM:
    return prgRom_[address - 0x8000];
  default:
    break;
  }
  return 0;
}

void NesMemory::write(uint16_t address, uint8_t value){
  switch (getRegion(address)){
  case MAIN_RAM:
    mirrorAddress(address);
    mainRam_[address] = value;
    break;
  case PPU_REGISTERS:
    mirrorAddress(address);
    ppuRegisters_[address] = value;
    break;
  case APU_REGISTERS:
    apuRegisters_[address - 0x4000] = value;
    break;
  case CONTROLLERS:
    controller_ = value;
    break;
  case EXPANSION_ROM:
    expansionRom_[address - 0x4020] = value;
    break;
  case SRAM:
    sram_[address - 0x6000] = value;
    break;
  case PRG_ROM:
    prgRom_[address - 0x8000] = value;
    break;
  default:
    break;
  }
}