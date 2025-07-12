#pragma once

namespace CpuTypes {

// CPU addressing modes
enum ADDRESSING_MODES {
  acc,
  implied,
  immediate,
  zeroPage,
  absolute,
  indirect,
  relative,
};

// CPU registers
enum REGISTERS { 
  NA, 
  A, 
  X, 
  Y, 
  PC, 
  SR 
};

// CPU status flags
enum STATUS_FLAGS {
  carry = 0b00000001,
  zero = 0b00000010,
  intrptDisable = 0b00000100,
  decimal = 0b00001000,
  bFlag = 0b00010000,
  unused = 0b00100000,
  overflow = 0b01000000,
  negative = 0b10000000,
};

} // namespace CpuTypes 