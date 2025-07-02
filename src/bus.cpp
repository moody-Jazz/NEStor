#include "bus.hpp"

Bus::Bus() {
  // Connect CPU to communication bus
  cpu6502.connectBus(this);

  for (auto &i : memory) i = 0x00;
}

Bus::~Bus() {}

void Bus::write(uint16_t addr, uint8_t data) {
  if (addr >= 0x0000 && addr <= 0xFFFF) memory[addr] = data;
}

uint8_t Bus::read(uint16_t addr, bool bReadOnly) {
  if (addr >= 0x0000 && addr <= 0xFFFF) return memory[addr];

  return 0x00;
}