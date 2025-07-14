#include "basememory.h"

BaseMemory::BaseMemory(uint32_t size): mainRam_(size){}

BaseMemory::~BaseMemory(){}

uint8_t BaseMemory::read(uint16_t address){
  return mainRam_[address];
}

void BaseMemory::write(uint16_t address, uint8_t value){
  mainRam_[address] = value;
}