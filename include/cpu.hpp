#pragma once

#include <cstdint>
#include <vector>
#include "global.hpp"

constexpr uint8_t ZERO_PAGE_STARTING = 0X00;
constexpr uint8_t ZERO_PAGE_ENDING = 0XFF;
constexpr uint16_t STACK_PAGE_STARTING = 0X0100;
constexpr uint16_t STACK_PAGE_ENDING = 0X01FF;

class Cpu {
 public:
  Cpu();

  std::pair<uint8_t, uint8_t> args;
  void executeInstruction();
  void reset();
  void printInfo();

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

  enum ADDRESSING_MODES {
    acc,
    implied,
    immediate,
    zeroPage,
    absolute,
    indirect,
    relative,
  };

  enum REGISTERS { NA, A, X, Y, PC, SR };

  bool getFlag(uint8_t flag);
  void clearFlag(uint8_t flag);
  void setFlag(uint8_t flag);
  void setFlag(uint8_t flag, bool condition);
  uint16_t getAddress(uint8_t mode, uint8_t offset);
  void setProgramCounter(uint16_t pc);
  void setStackPointer(uint16_t sp);
  void setAccumulator(uint8_t acc);
  void setIndexRegisterX(uint8_t x);
  void setIndexRegisterY(uint8_t y);
  void setStatusRegister(uint8_t status);
  uint16_t getProgramCounter();
  uint16_t getStackPointer();
  uint8_t getAccumulator();
  uint8_t getIndexRegisterX();
  uint8_t getIndexRegisterY();
  uint8_t getStatusRegister();
  void updateZNflag(uint8_t reg, uint8_t val);

  uint8_t cpuRead(uint16_t addr);
  void cpuWrite(uint16_t addr, uint8_t value);

  void LDA(uint8_t mode, uint8_t offset);
  void STA(uint8_t mode, uint8_t offset);
  void LDX(uint8_t mode, uint8_t offset);
  void STX(uint8_t mode, uint8_t offset);
  void LDY(uint8_t mode, uint8_t offset);
  void STY(uint8_t mode, uint8_t offset);
  void TAX(uint8_t, uint8_t);
  void TXA(uint8_t, uint8_t);
  void TAY(uint8_t, uint8_t);
  void TYA(uint8_t, uint8_t);
  void ADC(uint8_t mode, uint8_t offset);
  void SBC(uint8_t mode, uint8_t offset);
  void INC(uint8_t mode, uint8_t offset);
  void DEC(uint8_t mode, uint8_t offset);
  void INX(uint8_t, uint8_t);
  void DEX(uint8_t, uint8_t);
  void INY(uint8_t, uint8_t);
  void DEY(uint8_t, uint8_t);
  void ASL_acc(uint8_t, uint8_t);
  void ASL(uint8_t mode, uint8_t offset);
  void LSR_acc(uint8_t, uint8_t);
  void LSR(uint8_t mode, uint8_t offset);
  void ROL_acc(uint8_t, uint8_t);
  void ROL(uint8_t mode, uint8_t offset);
  void ROR_acc(uint8_t, uint8_t);
  void ROR(uint8_t mode, uint8_t offset);
  void AND(uint8_t mode, uint8_t offset);
  void ORA(uint8_t mode, uint8_t offset);
  void EOR(uint8_t mode, uint8_t offset);
  void BIT(uint8_t mode, uint8_t offset);
  void CMP(uint8_t mode, uint8_t offset);
  void CPX(uint8_t mode, uint8_t offset);
  void CPY(uint8_t mode, uint8_t offset);
  void BPL(uint8_t, uint8_t);
  void BMI(uint8_t, uint8_t);
  void BVS(uint8_t, uint8_t);
  void BVC(uint8_t, uint8_t);
  void BCC(uint8_t, uint8_t);
  void BCS(uint8_t, uint8_t);
  void BNE(uint8_t, uint8_t);
  void BEQ(uint8_t, uint8_t);
  void JMP_abs(uint8_t, uint8_t);
  void JMP_indr(uint8_t, uint8_t);
  void JSR(uint8_t, uint8_t);
  void RTS(uint8_t, uint8_t);
  void BRK(uint8_t, uint8_t);
  void RTI(uint8_t, uint8_t);
  void PHA(uint8_t, uint8_t);
  void PLA(uint8_t, uint8_t);
  void PHP(uint8_t, uint8_t);
  void PLP(uint8_t, uint8_t);
  void TXS(uint8_t, uint8_t);
  void TSX(uint8_t, uint8_t);
  void CLC(uint8_t, uint8_t);
  void SEC(uint8_t, uint8_t);
  void CLI(uint8_t, uint8_t);
  void SEI(uint8_t, uint8_t);
  void CLD(uint8_t, uint8_t);
  void SED(uint8_t, uint8_t);
  void CLV(uint8_t, uint8_t);
  void NOP(uint8_t, uint8_t);

  std::vector<std::pair<ADDRESSING_MODES, REGISTERS>> argList;

  void (Cpu::*opcodeMap[256])(uint8_t, uint8_t) = {
      BRK, ORA, NOP, NOP, NOP, ORA, ASL, NOP,     PHP,     ORA,      ASL_acc,
      NOP, NOP, ORA, ASL, NOP, BPL, ORA, NOP,     NOP,     NOP,      ORA,
      ASL, NOP, CLC, ORA, NOP, NOP, NOP, ORA,     ASL,     NOP,      JSR,
      AND, NOP, NOP, BIT, AND, ROL, NOP, PLP,     AND,     ROL_acc,  NOP,
      BIT, AND, ROL, NOP, BMI, AND, NOP, NOP,     NOP,     AND,      ROL,
      NOP, SEC, AND, NOP, NOP, NOP, AND, ROL,     NOP,     RTI,      EOR,
      NOP, NOP, NOP, EOR, LSR, NOP, PHA, EOR,     LSR_acc, NOP,      JMP_abs,
      EOR, LSR, NOP, BVC, EOR, NOP, NOP, NOP,     EOR,     LSR,      NOP,
      CLI, EOR, NOP, NOP, NOP, EOR, LSR, NOP,     RTS,     ADC,      NOP,
      NOP, NOP, ADC, ROR, NOP, PLA, ADC, ROR_acc, NOP,     JMP_indr, ADC,
      ROR, NOP, BVS, ADC, NOP, NOP, NOP, ADC,     ROR,     NOP,      SEI,
      ADC, NOP, NOP, NOP, ADC, ROR, NOP, NOP,     STA,     NOP,      NOP,
      STY, STA, STX, NOP, DEY, NOP, TXA, NOP,     STY,     STA,      STX,
      NOP, BCC, STA, NOP, NOP, STY, STA, STX,     NOP,     TYA,      STA,
      TXS, NOP, NOP, STA, NOP, NOP, LDY, LDA,     LDX,     NOP,      LDY,
      LDA, LDX, NOP, TAY, LDA, TAX, NOP, LDY,     LDA,     LDX,      NOP,
      BCS, LDA, NOP, NOP, LDY, LDA, LDX, NOP,     CLV,     LDA,      TSX,
      NOP, LDY, LDA, LDX, NOP, CPY, CMP, NOP,     NOP,     CPY,      CMP,
      DEC, NOP, INY, CMP, DEX, NOP, CPY, CMP,     DEC,     NOP,      BNE,
      CMP, NOP, NOP, NOP, CMP, DEC, NOP, CLD,     CMP,     NOP,      NOP,
      NOP, CMP, DEC, NOP, CPX, SBC, NOP, NOP,     CPX,     SBC,      INC,
      NOP, INX, SBC, NOP, NOP, CPX, SBC, INC,     NOP,     BEQ,      SBC,
      NOP, NOP, NOP, SBC, INC, NOP, SED, SBC,     NOP,     NOP,      NOP,
      SBC, INC, NOP,
  };

 private:
  uint8_t accumulator_;
  uint8_t indexRegisterX_, indexRegisterY_;

  uint16_t programCounter_;
  uint8_t stackPointer_;

  uint8_t statusRegister_{};

  uint8_t opcode_;
};