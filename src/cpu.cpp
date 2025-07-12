#include "cpu.hpp"
#include <iostream>

Cpu::Cpu() : argList(256) {
  stackPointer_ = 0xFD;
  programCounter_ = 0x0;
  statusRegister_ |= unused;

  argList[0x00] = {implied, NA};
  argList[0x01] = {indirect, X};
  argList[0x05] = {zeroPage, NA};
  argList[0x06] = {zeroPage, NA};
  argList[0x08] = {implied, NA};
  argList[0x09] = {immediate, NA};
  argList[0x0A] = {acc, NA};
  argList[0x0D] = {absolute, NA};
  argList[0x0E] = {absolute, NA};
  argList[0x10] = {relative, NA};
  argList[0x11] = {indirect, Y};
  argList[0x15] = {zeroPage, X};
  argList[0x16] = {zeroPage, X};
  argList[0x18] = {implied, NA};
  argList[0x19] = {absolute, Y};
  argList[0x1D] = {absolute, X};
  argList[0x1E] = {absolute, X};
  argList[0x20] = {absolute, NA};
  argList[0x21] = {indirect, X};
  argList[0x24] = {zeroPage, NA};
  argList[0x25] = {zeroPage, NA};
  argList[0x26] = {zeroPage, NA};
  argList[0x28] = {implied, NA};
  argList[0x29] = {immediate, NA};
  argList[0x2A] = {acc, NA};
  argList[0x2C] = {absolute, NA};
  argList[0x2D] = {absolute, NA};
  argList[0x2E] = {absolute, NA};
  argList[0x30] = {relative, NA};
  argList[0x31] = {indirect, Y};
  argList[0x35] = {zeroPage, X};
  argList[0x36] = {zeroPage, X};
  argList[0x38] = {implied, NA};
  argList[0x39] = {absolute, Y};
  argList[0x3D] = {absolute, X};
  argList[0x3E] = {absolute, X};
  argList[0x40] = {implied, NA};
  argList[0x41] = {indirect, X};
  argList[0x45] = {zeroPage, NA};
  argList[0x46] = {zeroPage, NA};
  argList[0x48] = {implied, NA};
  argList[0x49] = {immediate, NA};
  argList[0x4A] = {acc, NA};
  argList[0x4C] = {absolute, NA};
  argList[0x4D] = {absolute, NA};
  argList[0x4E] = {absolute, NA};
  argList[0x50] = {relative, NA};
  argList[0x51] = {indirect, Y};
  argList[0x55] = {zeroPage, X};
  argList[0x56] = {zeroPage, X};
  argList[0x58] = {implied, NA};
  argList[0x59] = {absolute, Y};
  argList[0x5D] = {absolute, X};
  argList[0x5E] = {absolute, X};
  argList[0x60] = {implied, NA};
  argList[0x61] = {indirect, X};
  argList[0x65] = {zeroPage, NA};
  argList[0x66] = {zeroPage, NA};
  argList[0x68] = {implied, NA};
  argList[0x69] = {immediate, NA};
  argList[0x6A] = {acc, NA};
  argList[0x6C] = {indirect, NA};
  argList[0x6D] = {absolute, NA};
  argList[0x6E] = {absolute, NA};
  argList[0x70] = {relative, NA};
  argList[0x71] = {indirect, Y};
  argList[0x75] = {zeroPage, X};
  argList[0x76] = {zeroPage, X};
  argList[0x78] = {implied, NA};
  argList[0x79] = {absolute, Y};
  argList[0x7D] = {absolute, X};
  argList[0x7E] = {absolute, X};
  argList[0x81] = {indirect, X};
  argList[0x84] = {zeroPage, NA};
  argList[0x85] = {zeroPage, NA};
  argList[0x86] = {zeroPage, NA};
  argList[0x88] = {implied, NA};
  argList[0x8A] = {implied, NA};
  argList[0x8C] = {absolute, NA};
  argList[0x8D] = {absolute, NA};
  argList[0x8E] = {absolute, NA};
  argList[0x90] = {relative, NA};
  argList[0x91] = {indirect, Y};
  argList[0x94] = {zeroPage, X};
  argList[0x95] = {zeroPage, X};
  argList[0x96] = {zeroPage, Y};
  argList[0x98] = {implied, NA};
  argList[0x99] = {absolute, Y};
  argList[0x9A] = {implied, NA};
  argList[0x9D] = {absolute, X};
  argList[0xA0] = {immediate, NA};
  argList[0xA1] = {indirect, X};
  argList[0xA2] = {immediate, NA};
  argList[0xA4] = {zeroPage, NA};
  argList[0xA5] = {zeroPage, NA};
  argList[0xA6] = {zeroPage, NA};
  argList[0xA8] = {implied, NA};
  argList[0xA9] = {immediate, NA};
  argList[0xAA] = {implied, NA};
  argList[0xAC] = {absolute, NA};
  argList[0xAD] = {absolute, NA};
  argList[0xAE] = {absolute, NA};
  argList[0xB0] = {relative, NA};
  argList[0xB1] = {indirect, Y};
  argList[0xB4] = {zeroPage, X};
  argList[0xB5] = {zeroPage, X};
  argList[0xB6] = {zeroPage, Y};
  argList[0xB8] = {implied, NA};
  argList[0xB9] = {absolute, Y};
  argList[0xBA] = {implied, NA};
  argList[0xBC] = {absolute, X};
  argList[0xBD] = {absolute, X};
  argList[0xBE] = {absolute, Y};
  argList[0xC0] = {immediate, NA};
  argList[0xC1] = {indirect, X};
  argList[0xC4] = {zeroPage, NA};
  argList[0xC5] = {zeroPage, NA};
  argList[0xC6] = {zeroPage, NA};
  argList[0xC8] = {implied, NA};
  argList[0xC9] = {immediate, NA};
  argList[0xCA] = {implied, NA};
  argList[0xCC] = {absolute, NA};
  argList[0xCD] = {absolute, NA};
  argList[0xCE] = {absolute, NA};
  argList[0xD0] = {relative, NA};
  argList[0xD1] = {indirect, Y};
  argList[0xD5] = {zeroPage, X};
  argList[0xD6] = {zeroPage, X};
  argList[0xD8] = {implied, NA};
  argList[0xD9] = {absolute, Y};
  argList[0xDD] = {absolute, X};
  argList[0xDE] = {absolute, X};
  argList[0xE0] = {immediate, NA};
  argList[0xE1] = {indirect, X};
  argList[0xE4] = {zeroPage, NA};
  argList[0xE5] = {zeroPage, NA};
  argList[0xE6] = {zeroPage, NA};
  argList[0xE8] = {implied, NA};
  argList[0xE9] = {immediate, NA};
  argList[0xEA] = {implied, NA};
  argList[0xEC] = {absolute, NA};
  argList[0xED] = {absolute, NA};
  argList[0xEE] = {absolute, NA};
  argList[0xF0] = {relative, NA};
  argList[0xF1] = {indirect, Y};
  argList[0xF5] = {zeroPage, X};
  argList[0xF6] = {zeroPage, X};
  argList[0xF8] = {implied, NA};
  argList[0xF9] = {absolute, Y};
  argList[0xFD] = {absolute, X};
  argList[0xFE] = {absolute, X};
}

void Cpu::executeInstruction() {
  opcode_ = cpuRead(programCounter_++);
  args = argList[opcode_];
  (this->*opcodeMap[opcode_])(args.first, args.second);
}

void Cpu::reset() {
  accumulator_ = 0;
  indexRegisterX_ = 0;
  indexRegisterY_ = 0;
  stackPointer_ = 0xFD;
  statusRegister_ = unused;  // Set unused bit (bit 5)
  programCounter_ = 0;
}

uint8_t Cpu::cpuRead(uint16_t addr) { return memory[addr]; }

void Cpu::cpuWrite(uint16_t addr, uint8_t value) { memory[addr] = value; }

void Cpu::printInfo() {
  std::cout << "prgrm counter: " << programCounter_ << "\n";
  std::cout << "index x and y reg: " << indexRegisterX_ << " " << indexRegisterY_
            << "\n";
  std::cout << "stack pointer: " << STACK_PAGE_STARTING + stackPointer_
            << "\n\n";
}

bool Cpu::getFlag(uint8_t flag) { return (statusRegister_ & flag); }
void Cpu::clearFlag(uint8_t flag) { statusRegister_ &= ~flag; }
void Cpu::setFlag(uint8_t flag) { statusRegister_ |= flag; }

void Cpu::setFlag(uint8_t flag, bool condition) {
  if (condition)
    setFlag(flag);
  else
    clearFlag(flag);
}

void Cpu::setProgramCounter(uint16_t pc) { programCounter_ = pc; }
void Cpu::setStackPointer(uint16_t sp) { stackPointer_ = sp; }
void Cpu::setAccumulator(uint8_t acc) { accumulator_ = acc; }
void Cpu::setIndexRegisterX(uint8_t x) { indexRegisterX_ = x; }
void Cpu::setIndexRegisterY(uint8_t y) { indexRegisterY_ = y; }
void Cpu::setStatusRegister(uint8_t status) { statusRegister_ = status; }
uint16_t Cpu::getProgramCounter() { return programCounter_; }
uint16_t Cpu::getStackPointer() { return stackPointer_; }
uint8_t Cpu::getAccumulator() { return accumulator_; }
uint8_t Cpu::getIndexRegisterX() { return indexRegisterX_; }
uint8_t Cpu::getIndexRegisterY() { return indexRegisterY_; }
uint8_t Cpu::getStatusRegister() { return statusRegister_; }

void Cpu::updateZNflag(uint8_t reg, uint8_t val) {
  if (reg == val)
    setFlag(zero);
  else
    clearFlag(zero);
  if (reg & negative)
    setFlag(negative);
  else
    clearFlag(negative);
}

uint16_t Cpu::getAddress(uint8_t mode, uint8_t offset) {
  uint16_t address{};
  uint8_t low{}, high{};

  if (mode == indirect) {
    uint8_t baseAddr = cpuRead(programCounter_++);

    if (offset == X) {
      low = cpuRead((baseAddr + indexRegisterX_) % 256);
      high = cpuRead((baseAddr + indexRegisterX_ + 1) % 256);
      address = (high << 8) | low;
    } else {
      low = cpuRead(baseAddr);
      high = cpuRead((baseAddr + 1) % 256);
      address = ((high << 8) | low) + indexRegisterY_;
    }
  }

  if (offset == X)
    offset = indexRegisterX_;
  else if (offset == Y)
    offset = indexRegisterY_;

  if (mode == immediate) {
    address = programCounter_++;
  } else if (mode == zeroPage) {
    address = (cpuRead(programCounter_++) + offset) % 256;
  } else if (mode == absolute) {
    low = cpuRead(programCounter_++);
    high = cpuRead(programCounter_++);
    address = ((high << 8) | low) + offset;
  }

  return address;
}

// Access instructions: LDA	STA	LDX	STX	LDY	STY
void Cpu::LDA(uint8_t mode, uint8_t offset) {
  accumulator_ = cpuRead(getAddress(mode, offset));
  updateZNflag(accumulator_, 0);
}

void Cpu::STA(uint8_t mode, uint8_t offset) {
  cpuWrite(getAddress(mode, offset), accumulator_);
}

void Cpu::LDX(uint8_t mode, uint8_t offset) {
  indexRegisterX_ = cpuRead(getAddress(mode, offset));
  updateZNflag(indexRegisterX_, 0);
}

void Cpu::STX(uint8_t mode, uint8_t offset) {
  cpuWrite(getAddress(mode, offset), indexRegisterX_);
}

void Cpu::LDY(uint8_t mode, uint8_t offset) {
  indexRegisterY_ = cpuRead(getAddress(mode, offset));
  updateZNflag(indexRegisterY_, 0);
}

void Cpu::STY(uint8_t mode, uint8_t offset) {
  cpuWrite(getAddress(mode, offset), indexRegisterY_);
}

// Transfer instructions: TAX TXA TAY TYA
void Cpu::TAX(uint8_t, uint8_t) {
  indexRegisterX_ = accumulator_;
  updateZNflag(indexRegisterX_, 0);
}
void Cpu::TXA(uint8_t, uint8_t) {
  accumulator_ = indexRegisterX_;
  updateZNflag(accumulator_, 0);
}
void Cpu::TAY(uint8_t, uint8_t) {
  indexRegisterY_ = accumulator_;
  updateZNflag(indexRegisterY_, 0);
}
void Cpu::TYA(uint8_t, uint8_t) {
  accumulator_ = indexRegisterY_;
  updateZNflag(accumulator_, 0);
}

// Arithmetic instructions: ADC SBC INC DEC INX DEX INY DEY
void Cpu::ADC(uint8_t mode, uint8_t offset) {
  uint8_t memval = cpuRead(getAddress(mode, offset));
  uint8_t accval = accumulator_;
  uint16_t sum = accumulator_ + memval + getFlag(carry);
  accumulator_ = sum;

  if (sum > 0xFF)
    setFlag(carry);
  else
    clearFlag(carry);
  if ((sum ^ accval) & (sum ^ memval) & negative)
    setFlag(overflow);
  else
    clearFlag(overflow);
  updateZNflag(accumulator_, 0);
}

void Cpu::SBC(uint8_t mode, uint8_t offset) {
  uint8_t memval = cpuRead(getAddress(mode, offset));
  uint8_t accval = accumulator_;
  uint16_t sum = accumulator_ - memval - !getFlag(carry);

  accumulator_ = sum;

  if (sum > 0xFF)
    clearFlag(carry);
  else
    setFlag(carry);
  if ((sum ^ accval) & (sum ^ ~memval) & negative)
    setFlag(overflow);
  else
    clearFlag(overflow);
  updateZNflag(accumulator_, 0);
}

void Cpu::INC(uint8_t mode, uint8_t offset) {
  uint16_t address = getAddress(mode, offset);
  uint8_t value = cpuRead(address) + 1;
  cpuWrite(address, value);
  updateZNflag(value, 0);
}

void Cpu::DEC(uint8_t mode, uint8_t offset) {
  uint16_t address = getAddress(mode, offset);
  uint8_t value = cpuRead(address) - 1;
  cpuWrite(address, value);
  updateZNflag(value, 0);
}

void Cpu::INX(uint8_t, uint8_t) {
  indexRegisterX_++;
  updateZNflag(indexRegisterX_, 0);
}
void Cpu::DEX(uint8_t, uint8_t) {
  indexRegisterX_--;
  updateZNflag(indexRegisterX_, 0);
}
void Cpu::INY(uint8_t, uint8_t) {
  indexRegisterY_++;
  updateZNflag(indexRegisterY_, 0);
}
void Cpu::DEY(uint8_t, uint8_t) {
  indexRegisterY_--;
  updateZNflag(indexRegisterY_, 0);
}

// Shift instructions : ASL LSR ROL ROR
void Cpu::ASL_acc(uint8_t, uint8_t) {
  if (accumulator_ & negative)
    setFlag(carry);
  else
    clearFlag(carry);

  accumulator_ <<= 1;
  updateZNflag(accumulator_, 0);
}

void Cpu::ASL(uint8_t mode, uint8_t offset) {
  uint16_t address = getAddress(mode, offset);
  uint8_t value = cpuRead(address);
  if (value & negative)
    setFlag(carry);
  else
    clearFlag(carry);

  value <<= 1;
  cpuWrite(address, value);
  updateZNflag(value, 0);
}

void Cpu::LSR_acc(uint8_t, uint8_t) {
  if (accumulator_ & carry)
    setFlag(carry);
  else
    clearFlag(carry);

  accumulator_ >>= 1;
  updateZNflag(accumulator_, 0);
  clearFlag(negative);
}
void Cpu::LSR(uint8_t mode, uint8_t offset) {
  uint16_t address = getAddress(mode, offset);
  uint8_t value = cpuRead(address);
  if (value & carry)
    setFlag(carry);
  else
    clearFlag(carry);

  value >>= 1;
  cpuWrite(address, value);
  updateZNflag(value, 0);
  clearFlag(negative);
}

void Cpu::ROL_acc(uint8_t, uint8_t) {
  bool prevCarry = getFlag(carry);
  if (accumulator_ & negative)
    setFlag(carry);
  else
    clearFlag(carry);

  accumulator_ <<= 1;
  accumulator_ |= prevCarry;
  updateZNflag(accumulator_, 0);
}

void Cpu::ROL(uint8_t mode, uint8_t offset) {
  uint16_t address = getAddress(mode, offset);
  bool prevCarry = getFlag(carry);
  uint8_t value = cpuRead(address);

  if (value & negative)
    setFlag(carry);
  else
    clearFlag(carry);

  value <<= 1;
  value |= prevCarry;
  cpuWrite(address, value);
  updateZNflag(value, 0);
}

void Cpu::ROR_acc(uint8_t, uint8_t) {
  bool prevCarry = getFlag(carry);
  if (accumulator_ & carry)
    setFlag(carry);
  else
    clearFlag(carry);

  accumulator_ >>= 1;
  accumulator_ |= (prevCarry << 7);
  updateZNflag(accumulator_, 0);
}
void Cpu::ROR(uint8_t mode, uint8_t offset) {
  uint16_t address = getAddress(mode, offset);
  bool prevCarry = getFlag(carry);
  uint8_t value = cpuRead(address);

  if (value & carry)
    setFlag(carry);
  else
    clearFlag(carry);

  value >>= 1;
  value |= (prevCarry << 7);
  cpuWrite(address, value);
  updateZNflag(value, 0);
}

// Bitwise instructions: AND ORA EOR BIT
void Cpu::AND(uint8_t mode, uint8_t offset) {
  accumulator_ &= cpuRead(getAddress(mode, offset));
  updateZNflag(accumulator_, 0);
}

void Cpu::ORA(uint8_t mode, uint8_t offset) {
  accumulator_ |= cpuRead(getAddress(mode, offset));
  updateZNflag(accumulator_, 0);
}

void Cpu::EOR(uint8_t mode, uint8_t offset) {
  accumulator_ ^= cpuRead(getAddress(mode, offset));
  updateZNflag(accumulator_, 0);
}

void Cpu::BIT(uint8_t mode, uint8_t offset) {
  uint8_t value = cpuRead(getAddress(mode, offset));

  setFlag(zero, (accumulator_ & value) == 0);
  setFlag(overflow, value & overflow);
  setFlag(negative, value & negative);
}

// Comparison instructions: CMP CPX CPY
void Cpu::CMP(uint8_t mode, uint8_t offset) {
  uint8_t value = cpuRead(getAddress(mode, offset));
  uint8_t result = accumulator_ - value;

  setFlag(carry, accumulator_ >= value);
  setFlag(zero, accumulator_ == value);
  setFlag(negative, result & negative);
}

void Cpu::CPX(uint8_t mode, uint8_t offset) {
  uint8_t value = cpuRead(getAddress(mode, offset));
  uint8_t result = indexRegisterX_ - value;

  setFlag(carry, indexRegisterX_ >= value);
  setFlag(zero, indexRegisterX_ == value);
  setFlag(negative, result & negative);
}

void Cpu::CPY(uint8_t mode, uint8_t offset) {
  uint8_t value = cpuRead(getAddress(mode, offset));
  uint8_t result = indexRegisterY_ - value;

  setFlag(carry, indexRegisterY_ >= value);
  setFlag(zero, indexRegisterY_ == value);
  setFlag(negative, result & negative);
}

// Branch instructions
#define BRANCH_IF(condition)                                    \
  uint8_t offset = cpuRead(programCounter_++);                 \
  if (condition) programCounter_ += static_cast<int8_t>(offset);

void Cpu::BPL(uint8_t, uint8_t) { BRANCH_IF(!(statusRegister_ & negative)); }
void Cpu::BMI(uint8_t, uint8_t) { BRANCH_IF(statusRegister_ & negative); }
void Cpu::BVC(uint8_t, uint8_t) { BRANCH_IF(!(statusRegister_ & overflow)); }
void Cpu::BVS(uint8_t, uint8_t) { BRANCH_IF(statusRegister_ & overflow); }
void Cpu::BCC(uint8_t, uint8_t) { BRANCH_IF(!(statusRegister_ & carry)); }
void Cpu::BCS(uint8_t, uint8_t) { BRANCH_IF(statusRegister_ & carry); }
void Cpu::BNE(uint8_t, uint8_t) { BRANCH_IF(!(statusRegister_ & zero)); }
void Cpu::BEQ(uint8_t, uint8_t) { BRANCH_IF(statusRegister_ & zero); }

// Jump instructions: JMP JSR RTS BRK RTI
void Cpu::JMP_abs(uint8_t, uint8_t) { programCounter_ = getAddress(absolute, 0); }

void Cpu::JMP_indr(uint8_t, uint8_t) {
  uint8_t low = cpuRead(programCounter_++);
  uint8_t high = cpuRead(programCounter_++);
  uint16_t address = (high << 8) | low;

  low = cpuRead(address++);
  high = cpuRead(address);
  programCounter_ = (high << 8) | low;
}

void Cpu::JSR(uint8_t, uint8_t) {
  uint16_t address = getAddress(absolute, 0);
  uint16_t return_address = programCounter_ - 1;

  cpuWrite(STACK_PAGE_STARTING + stackPointer_--, (return_address >> 8) & 0xFF);
  cpuWrite(STACK_PAGE_STARTING + stackPointer_--, return_address & 0xFF);

  programCounter_ = address;
}

void Cpu::RTS(uint8_t, uint8_t) {
  uint8_t low = cpuRead(STACK_PAGE_STARTING + ++stackPointer_);
  uint8_t high = cpuRead(STACK_PAGE_STARTING + ++stackPointer_);
  programCounter_ = ((high << 8) | low) + 1;
}

void Cpu::BRK(uint8_t, uint8_t) {
  ++programCounter_;
  cpuWrite(STACK_PAGE_STARTING + stackPointer_--, (programCounter_ >> 8) & 0xFF);
  cpuWrite(STACK_PAGE_STARTING + stackPointer_--, programCounter_ & 0xFF);
  cpuWrite(STACK_PAGE_STARTING + stackPointer_--, statusRegister_ | bFlag);

  setFlag(intrptDisable);
  uint8_t low = cpuRead(0xFFFE);
  uint8_t high = cpuRead(0xFFFF);
  programCounter_ = (high << 8) | low;
}

void Cpu::RTI(uint8_t, uint8_t) {
  statusRegister_ = cpuRead(STACK_PAGE_STARTING + ++stackPointer_) | unused;
  clearFlag(bFlag);
  uint8_t low = cpuRead(STACK_PAGE_STARTING + ++stackPointer_);
  uint8_t high = cpuRead(STACK_PAGE_STARTING + ++stackPointer_);
  programCounter_ = (high << 8) | low;
}

// Stack instructions: PHA PLA PHP PLP TXS TSX
void Cpu::PHA(uint8_t, uint8_t) {
  cpuWrite(STACK_PAGE_STARTING + stackPointer_--, accumulator_);
}

void Cpu::PLA(uint8_t, uint8_t) {
  accumulator_ = cpuRead(STACK_PAGE_STARTING + ++stackPointer_);
  updateZNflag(accumulator_, 0);
}

void Cpu::PHP(uint8_t, uint8_t) {
  cpuWrite(STACK_PAGE_STARTING + stackPointer_--, statusRegister_ | 0b00110000);
}

void Cpu::PLP(uint8_t, uint8_t) {
  statusRegister_ &= 0b00110000;
  statusRegister_ |= cpuRead(STACK_PAGE_STARTING + ++stackPointer_) & 0b11001111;
}

void Cpu::TXS(uint8_t, uint8_t) { stackPointer_ = indexRegisterX_; }

void Cpu::TSX(uint8_t, uint8_t) {
  indexRegisterX_ = stackPointer_;
  updateZNflag(indexRegisterX_, 0);
}

// Flag instructions: CLC SEC CLI SEI CLD SED CLV
void Cpu::CLC(uint8_t, uint8_t) { clearFlag(carry); }
void Cpu::SEC(uint8_t, uint8_t) { setFlag(carry); }

void Cpu::CLI(uint8_t, uint8_t) { clearFlag(intrptDisable); }
void Cpu::SEI(uint8_t, uint8_t) { setFlag(intrptDisable); }

void Cpu::CLD(uint8_t, uint8_t) { clearFlag(decimal); }
void Cpu::SED(uint8_t, uint8_t) { setFlag(decimal); }

void Cpu::CLV(uint8_t, uint8_t) { clearFlag(overflow); }

// Other instruction: NOP
void Cpu::NOP(uint8_t, uint8_t) {
  // Intentionally does nothing
}