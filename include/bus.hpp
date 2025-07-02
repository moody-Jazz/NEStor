#pragma once
#include "cpu.hpp"

class Bus {
 public:
  Bus();
  ~Bus();

  // Devices on bus
  Cpu cpu6502;
  uint8_t memory[65535];

  void write(uint16_t addr, uint8_t data);
  uint8_t read(uint16_t addr, bool bReadOnly = false);
};