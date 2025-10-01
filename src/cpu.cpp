#include <iostream>
#include "cpu.h"

using namespace CpuTypes;

Cpu::Cpu(std::unique_ptr<BaseMemory> memPtr)
 : memPtr_(std::move(memPtr)), opcodeArgList(256) {
  stackPointer_ = 0xFD;
  programCounter_ = 0x0;
  statusRegister_ |= unused;
  pageCrossed_ = false;

  loadOpcodedata(opcodeArgList);
}

void Cpu::executeInstruction() {
  pageCrossed_ = false;
  opcode_ = cpuRead(programCounter_++);
  currCycleCount_ = opcodeCycles[opcode_];

  args = opcodeArgList[opcode_];

  // the function call below has extra side effect of 
  // incrementing currCycleCount_ if page is crossed
  (this->*opcodeMap[opcode_])(args.first, args.second);

  totalCycleCount_ += currCycleCount_;
}

void Cpu::reset() {
  accumulator_ = 0;
  indexRegisterX_ = 0;
  indexRegisterY_ = 0;
  stackPointer_ = 0xFD;
  statusRegister_ = unused;  // Set unused bit (bit 5)
  programCounter_ = 0;
}

uint8_t Cpu::cpuRead(uint16_t addr) { return memPtr_->read(addr); }

void Cpu::cpuWrite(uint16_t addr, uint8_t value) { memPtr_->write(addr, value); }

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
uint32_t Cpu::getCurrCycleCount() { return currCycleCount_; }
uint32_t Cpu::getTotalCycleCount() { return totalCycleCount_; }

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

bool Cpu::isPageCrossed(uint16_t oldAddr, uint16_t newAddr){
  return (oldAddr & 0xFF00) != (newAddr & 0xFF00);
}

// this function has side effect of incrementing the cyclecount if page is crossed
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
      address = ((high << 8) | low);
      pageCrossed_ = isPageCrossed(address, address + indexRegisterY_);
      address += indexRegisterY_;
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
    address = ((high << 8) | low);
    pageCrossed_ = isPageCrossed(address, address + offset);
    address += offset;
  }

  return address;
}

// Access instructions: LDA	STA	LDX	STX	LDY	STY
void Cpu::LDA(uint8_t mode, uint8_t offset) {
  accumulator_ = cpuRead(getAddress(mode, offset));
  updateZNflag(accumulator_, 0);
  currCycleCount_ += pageCrossed_;
}

void Cpu::STA(uint8_t mode, uint8_t offset) {
  cpuWrite(getAddress(mode, offset), accumulator_);
}

void Cpu::LDX(uint8_t mode, uint8_t offset) {
  indexRegisterX_ = cpuRead(getAddress(mode, offset));
  updateZNflag(indexRegisterX_, 0);
  currCycleCount_ += pageCrossed_;
}

void Cpu::STX(uint8_t mode, uint8_t offset) {
  cpuWrite(getAddress(mode, offset), indexRegisterX_);
}

void Cpu::LDY(uint8_t mode, uint8_t offset) {
  indexRegisterY_ = cpuRead(getAddress(mode, offset));
  updateZNflag(indexRegisterY_, 0);
  currCycleCount_ += pageCrossed_;
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
  currCycleCount_ += pageCrossed_;
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
  currCycleCount_ += pageCrossed_;
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
  currCycleCount_ += pageCrossed_;
}

void Cpu::ORA(uint8_t mode, uint8_t offset) {
  accumulator_ |= cpuRead(getAddress(mode, offset));
  updateZNflag(accumulator_, 0);
  currCycleCount_ += pageCrossed_;
}

void Cpu::EOR(uint8_t mode, uint8_t offset) {
  accumulator_ ^= cpuRead(getAddress(mode, offset));
  updateZNflag(accumulator_, 0);
  currCycleCount_ += pageCrossed_;
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
  currCycleCount_ += pageCrossed_;
}

void Cpu::CPX(uint8_t mode, uint8_t offset) {
  uint8_t value = cpuRead(getAddress(mode, offset));
  uint8_t result = indexRegisterX_ - value;

  setFlag(carry, indexRegisterX_ >= value);
  setFlag(zero, indexRegisterX_ == value);
  setFlag(negative, result & negative);
  currCycleCount_ += pageCrossed_;
}

void Cpu::CPY(uint8_t mode, uint8_t offset) {
  uint8_t value = cpuRead(getAddress(mode, offset));
  uint8_t result = indexRegisterY_ - value;

  setFlag(carry, indexRegisterY_ >= value);
  setFlag(zero, indexRegisterY_ == value);
  setFlag(negative, result & negative);
  currCycleCount_ += pageCrossed_;
}

// Branch instructions
#define BRANCH_IF(condition)                                  \
  uint8_t offset = cpuRead(programCounter_++);                \
  if (condition){                                             \
    uint16_t oldPc = programCounter_;                         \
    programCounter_ += static_cast<int8_t>(offset);           \
    currCycleCount_ += isPageCrossed(oldPc, programCounter_); \
    currCycleCount_++;                                        \
  }                                                           \

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