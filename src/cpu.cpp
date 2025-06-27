#include "cpu.hpp"

cpu::cpu(): argList(256){
    stkPtr_ = STACK_PAGE_ENDING;
    prgrmCtr_ = 0x0;

    argList[0x00] = {implied, 0};
    argList[0x01] = {indirect, indexregX_};
    argList[0x05] = {zeroPage, 0};
    argList[0x06] = {zeroPage, 0};
    argList[0x08] = {implied, 0};
    argList[0x09] = {immediate, 0};
    argList[0x0A] = {acc, 0};
    argList[0x0D] = {absolute, 0};
    argList[0x0E] = {absolute, 0};
    argList[0x10] = {relative, 0};
    argList[0x11] = {indirect, indexregY_};
    argList[0x15] = {zeroPage, indexregX_};
    argList[0x16] = {zeroPage, indexregX_};
    argList[0x18] = {implied, 0};
    argList[0x19] = {absolute, indexregY_};
    argList[0x1D] = {absolute, indexregX_};
    argList[0x1E] = {absolute, indexregX_};
    argList[0x20] = {absolute, 0};
    argList[0x21] = {indirect, indexregX_};
    argList[0x24] = {zeroPage, 0};
    argList[0x25] = {zeroPage, 0};
    argList[0x26] = {zeroPage, 0};
    argList[0x28] = {implied, 0};
    argList[0x29] = {immediate, 0};
    argList[0x2A] = {acc, 0};
    argList[0x2C] = {absolute, 0};
    argList[0x2D] = {absolute, 0};
    argList[0x2E] = {absolute, 0};
    argList[0x30] = {relative, 0};
    argList[0x31] = {indirect, indexregY_};
    argList[0x35] = {zeroPage, indexregX_};
    argList[0x36] = {zeroPage, indexregX_};
    argList[0x38] = {implied, 0};
    argList[0x39] = {absolute, indexregY_};
    argList[0x3D] = {absolute, indexregX_};
    argList[0x3E] = {absolute, indexregX_};
    argList[0x40] = {implied, 0};
    argList[0x41] = {indirect, indexregX_};
    argList[0x45] = {zeroPage, 0};
    argList[0x46] = {zeroPage, 0};
    argList[0x48] = {implied, 0};
    argList[0x49] = {immediate, 0};
    argList[0x4A] = {acc, 0};
    argList[0x4C] = {absolute, 0};
    argList[0x4D] = {absolute, 0};
    argList[0x4E] = {absolute, 0};
    argList[0x50] = {relative, 0};
    argList[0x51] = {indirect, indexregY_};
    argList[0x55] = {zeroPage, indexregX_};
    argList[0x56] = {zeroPage, indexregX_};
    argList[0x58] = {implied, 0};
    argList[0x59] = {absolute, indexregY_};
    argList[0x5D] = {absolute, indexregX_};
    argList[0x5E] = {absolute, indexregX_};
    argList[0x60] = {implied, 0};
    argList[0x61] = {indirect, indexregX_};
    argList[0x65] = {zeroPage, 0};
    argList[0x66] = {zeroPage, 0};
    argList[0x68] = {implied, 0};
    argList[0x69] = {immediate, 0};
    argList[0x6A] = {acc, 0};
    argList[0x6C] = {indirect, 0};
    argList[0x6D] = {absolute, 0};
    argList[0x6E] = {absolute, 0};
    argList[0x70] = {relative, 0};
    argList[0x71] = {indirect, indexregY_};
    argList[0x75] = {zeroPage, indexregX_};
    argList[0x76] = {zeroPage, indexregX_};
    argList[0x78] = {implied, 0};
    argList[0x79] = {absolute, indexregY_};
    argList[0x7D] = {absolute, indexregX_};
    argList[0x7E] = {absolute, indexregX_};
    argList[0x81] = {indirect, indexregX_};
    argList[0x84] = {zeroPage, 0};
    argList[0x85] = {zeroPage, 0};
    argList[0x86] = {zeroPage, 0};
    argList[0x88] = {implied, 0};
    argList[0x8A] = {implied, 0};
    argList[0x8C] = {absolute, 0};
    argList[0x8D] = {absolute, 0};
    argList[0x8E] = {absolute, 0};
    argList[0x90] = {relative, 0};
    argList[0x91] = {indirect, indexregY_};
    argList[0x94] = {zeroPage, indexregX_};
    argList[0x95] = {zeroPage, indexregX_};
    argList[0x96] = {zeroPage, indexregY_};
    argList[0x98] = {implied, 0};
    argList[0x99] = {absolute, indexregY_};
    argList[0x9A] = {implied, 0};
    argList[0x9D] = {absolute, indexregX_};
    argList[0xA0] = {immediate, 0};
    argList[0xA1] = {indirect, indexregX_};
    argList[0xA2] = {immediate, 0};
    argList[0xA4] = {zeroPage, 0};
    argList[0xA5] = {zeroPage, 0};
    argList[0xA6] = {zeroPage, 0};
    argList[0xA8] = {implied, 0};
    argList[0xA9] = {immediate, 0};
    argList[0xAA] = {implied, 0};
    argList[0xAC] = {absolute, 0};
    argList[0xAD] = {absolute, 0};
    argList[0xAE] = {absolute, 0};
    argList[0xB0] = {relative, 0};
    argList[0xB1] = {indirect, indexregY_};
    argList[0xB4] = {zeroPage, indexregX_};
    argList[0xB5] = {zeroPage, indexregX_};
    argList[0xB6] = {zeroPage, indexregY_};
    argList[0xB8] = {implied, 0};
    argList[0xB9] = {absolute, indexregY_};
    argList[0xBA] = {implied, 0};
    argList[0xBC] = {absolute, indexregX_};
    argList[0xBD] = {absolute, indexregX_};
    argList[0xBE] = {absolute, indexregY_};
    argList[0xC0] = {immediate, 0};
    argList[0xC1] = {indirect, indexregX_};
    argList[0xC4] = {zeroPage, 0};
    argList[0xC5] = {zeroPage, 0};
    argList[0xC6] = {zeroPage, 0};
    argList[0xC8] = {implied, 0};
    argList[0xC9] = {immediate, 0};
    argList[0xCA] = {implied, 0};
    argList[0xCC] = {absolute, 0};
    argList[0xCD] = {absolute, 0};
    argList[0xCE] = {absolute, 0};
    argList[0xD0] = {relative, 0};
    argList[0xD1] = {indirect, indexregY_};
    argList[0xD5] = {zeroPage, indexregX_};
    argList[0xD6] = {zeroPage, indexregX_};
    argList[0xD8] = {implied, 0};
    argList[0xD9] = {absolute, indexregY_};
    argList[0xDD] = {absolute, indexregX_};
    argList[0xDE] = {absolute, indexregX_};
    argList[0xE0] = {immediate, 0};
    argList[0xE1] = {indirect, indexregX_};
    argList[0xE4] = {zeroPage, 0};
    argList[0xE5] = {zeroPage, 0};
    argList[0xE6] = {zeroPage, 0};
    argList[0xE8] = {implied, 0};
    argList[0xE9] = {immediate, 0};
    argList[0xEA] = {implied, 0};
    argList[0xEC] = {absolute, 0};
    argList[0xED] = {absolute, 0};
    argList[0xEE] = {absolute, 0};
    argList[0xF0] = {relative, 0};
    argList[0xF1] = {indirect, indexregY_};
    argList[0xF5] = {zeroPage, indexregX_};
    argList[0xF6] = {zeroPage, indexregX_};
    argList[0xF8] = {implied, 0};
    argList[0xF9] = {absolute, indexregY_};
    argList[0xFD] = {absolute, indexregX_};
    argList[0xFE] = {absolute, indexregX_};
}

void cpu::cycle(){
    opcode_ = memory[prgrmCtr_++];
    
    args = argList[opcode_];
    (this->*opcodeMap[opcode_])(args.first, args.second);
}

void cpu::printInfo(){
    std::cout<<"prgrm counter: "<<prgrmCtr_<<"\n";
    std::cout<<"index x and y reg: "<<indexregX_<<" "<<indexregY_<<"\n";
    std::cout<<"stack pointer: "<<stkPtr_<<"\n\n";
}

bool cpu::getFlag(uint8_t flag){
    return (statusReg_ & flag); 
}
void cpu::clearFlag(uint8_t flag){
    statusReg_ &= ~flag;
}
void cpu::setFlag(uint8_t flag){
    statusReg_ |= flag;
}

uint16_t cpu::getAddress(uint8_t mode, uint8_t offset){
    uint16_t address{};

    if(mode == immediate)
        address = prgrmCtr_++;

    else if(mode == zeroPage)
        address = (memory[prgrmCtr_++] + offset) % 256;
    
    else if(mode == absolute){
        uint8_t low = memory[prgrmCtr_++];
        uint8_t high = memory[prgrmCtr_++];
        address = (high << 8) | low;
        address += offset;
    }
    else if(mode == indirect){
        uint8_t zpageAddr = memory[prgrmCtr_++];
        uint8_t low = memory[(zpageAddr + offset) % 256];
        uint8_t high = memory[(zpageAddr + offset + 1) % 256];
        address = (high << 8) | low;
    }
    return address;
}

// Access instructions: LDA	STA	LDX	STX	LDY	STY		
void cpu::LDA(uint8_t mode, uint8_t offset){
    acc_ = memory[getAddress(mode, offset)];

    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(zero);
}

void cpu::STA(uint8_t mode, uint8_t offset){
    memory[getAddress(mode, offset)] = acc_;
}

void cpu::LDX(uint8_t mode, uint8_t offset){
    indexregX_ = memory[getAddress(mode, offset)];

    if(indexregX_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregX_ & negative) setFlag(negative);
    else clearFlag(zero);
}

void cpu::STX(uint8_t mode, uint8_t offset){
    memory[getAddress(mode, offset)] = indexregX_;
}
 
void cpu::LDY(uint8_t mode, uint8_t offset){
    indexregY_ = memory[getAddress(mode, offset)];

    if(indexregY_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregY_ & negative) setFlag(negative);
    else clearFlag(zero);
}

void cpu::STY(uint8_t mode, uint8_t offset){
    memory[getAddress(mode, offset)] = indexregY_;
}

// Transfer instructions: TAX TXA TAY TYA		
void cpu::TAX(uint8_t, uint8_t){
    indexregX_ = acc_;

    if(indexregX_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregX_ & negative) setFlag(negative);
    else clearFlag(zero);
} 
void cpu::TXA(uint8_t, uint8_t){
    acc_ = indexregX_;

    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(zero);
} 
void cpu::TAY(uint8_t, uint8_t){
    indexregY_ = acc_;

    if(indexregY_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregY_ & negative) setFlag(negative);
    else clearFlag(zero);
} 
void cpu::TYA(uint8_t, uint8_t){
    acc_ = indexregY_;

    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(zero);
} 

// Arithmetic instructions: ADC	SBC	INC	DEC	INX	DEX	INY	DEY
void cpu::ADC(uint8_t mode, uint8_t offset){
    uint8_t memval = memory[getAddress(mode, offset)];
    uint8_t accval = acc_;
    uint16_t sum = acc_ + memval + getFlag(carry);
    acc_ = sum;

    if(sum > 0xFF) setFlag(carry);
    else clearFlag(carry);
    if(((accval & negative) && (memval & negative) && ~(sum & negative)) ||
    (~(accval & negative) && ~(memval & negative) && (sum & negative)))
        setFlag(overflow);
    else clearFlag(overflow);
    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(zero);
}

void cpu::SBC(uint8_t mode, uint8_t offset){
    uint8_t memval = memory[getAddress(mode, offset)];
    uint8_t accval = acc_;
    uint16_t sum = acc_ + ~memval + getFlag(carry);
    acc_ = sum;
    
    if(sum > 0xFF) clearFlag(carry);
    else setFlag(carry);
    if(((accval & negative) && (memval & negative) && ~(sum & negative)) ||
    (~(accval & negative) && ~(memval & negative) && (sum & negative)))
        setFlag(overflow);
    else clearFlag(overflow);
    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(zero);
}  

void cpu::INC(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    memory[address]++;

    if(memory[address] == 0) setFlag(zero);
    else clearFlag(zero);
    if(memory[address] & negative) setFlag(negative);
    else clearFlag(zero);
}

void cpu::DEC(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    memory[address]--;

    if(memory[address] == 0) setFlag(zero);
    else clearFlag(zero);
    if(memory[address] & negative) setFlag(negative);
    else clearFlag(zero);
}

void cpu::INX(uint8_t, uint8_t){
    indexregX_++;
    if(indexregX_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregX_ & negative) setFlag(negative);
    else clearFlag(zero);
} 
void cpu::DEX(uint8_t, uint8_t){
    indexregX_--;
    if(indexregX_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregX_ & negative) setFlag(negative);
    else clearFlag(zero);
} 
void cpu::INY(uint8_t, uint8_t){
    indexregY_++;
    if(indexregY_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregY_ & negative) setFlag(negative);
    else clearFlag(zero);
} 
void cpu::DEY(uint8_t, uint8_t){
    indexregY_--;
    if(indexregY_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregY_ & negative) setFlag(negative);
    else clearFlag(zero);
} 

// Shift instructions : ASL	LSR	ROL	ROR
void cpu::ASL_acc(uint8_t, uint8_t){
    if(acc_ & negative) setFlag(carry);
    else clearFlag(carry);

    acc_ <<= 1;
    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(negative);
}

void cpu::ASL(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    if(memory[address] & negative) setFlag(carry);
    else clearFlag(carry);

    memory[address] <<= 1;

    if(memory[address] == 0) setFlag(zero);
    else clearFlag(zero);
    if(memory[address] & negative) setFlag(negative);
    else clearFlag(negative);
}

void cpu::LSR_acc(uint8_t, uint8_t){
    if(acc_ & carry) setFlag(carry);
    else clearFlag(carry);

    acc_ >>= 1;
    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    clearFlag(negative);
}
void cpu::LSR(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    if(memory[address] & negative) setFlag(carry);
    else clearFlag(carry);

    memory[address] >>= 1;

    if(memory[address] == 0) setFlag(zero);
    else clearFlag(zero);
    clearFlag(negative);
}

void cpu::ROL_acc(uint8_t, uint8_t){
    bool prevCarry = getFlag(carry);
    if(acc_ & negative) setFlag(carry);
    else clearFlag(carry);

    acc_ <<= 1;
    acc_ |= prevCarry;
    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(negative);
}

void cpu::ROL(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    bool prevCarry = getFlag(carry);
    if(memory[address] & negative) setFlag(carry);
    else clearFlag(carry);

    memory[address] <<= 1;
    memory[address] |= prevCarry;
    if(memory[address] == 0) setFlag(zero);
    else clearFlag(zero);
    if(memory[address] & negative) setFlag(negative);
    else clearFlag(negative);
}

void cpu::ROR_acc(uint8_t, uint8_t){
    bool prevCarry = getFlag(carry);
    if(acc_ & negative) setFlag(carry);
    else clearFlag(carry);

    acc_ >>= 1;
    acc_ |= (prevCarry << 7);

    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(negative);   
}
void cpu::ROR(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    bool prevCarry = getFlag(carry);
    if(memory[address] & negative) setFlag(carry);
    else clearFlag(carry);

    memory[address] >>= 1;
    memory[address] |= (prevCarry << 7);

    if(memory[address] == 0) setFlag(zero);
    else clearFlag(zero);
    if(memory[address] & negative) setFlag(negative);
    else clearFlag(negative);   
}

// Bitwise instructions: AND ORA EOR BIT				
void cpu::AND(uint8_t mode, uint8_t offset){
    acc_ &= memory[getAddress(mode, offset)];

    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(negative);   
}

void cpu::ORA(uint8_t mode, uint8_t offset){
    acc_ |= memory[getAddress(mode, offset)];

    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(negative);   
}

void cpu::EOR(uint8_t mode, uint8_t offset){
    acc_ ^= memory[getAddress(mode, offset)];

    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(negative);   
}

void cpu::BIT(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);

    if(acc_ & memory[address]) setFlag(zero);
    else clearFlag(zero);
    if(memory[address] & overflow) setFlag(overflow);
    else clearFlag(overflow);
    if(memory[address] & negative) setFlag(negative);
    else clearFlag(negative);
}			

void cpu::CMP(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    uint8_t temp = acc_ - memory[address];

    if(acc_ >= memory[address]) setFlag(carry);
    else clearFlag(carry);
    if(acc_ == memory[address]) setFlag(zero);
    else clearFlag(zero);
    if(temp & negative) setFlag(negative);
    else clearFlag(negative);
}

void cpu::CPX(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    uint8_t temp = indexregX_ - memory[address];

    if(indexregX_ >= memory[address]) setFlag(carry);
    else clearFlag(carry);
    if(indexregX_ == memory[address]) setFlag(zero);
    else clearFlag(zero);
    if(temp & negative) setFlag(negative);
    else clearFlag(negative);
}

void cpu::CPY(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    uint8_t temp = indexregY_ - memory[address];

    if(indexregY_ >= memory[address]) setFlag(carry);
    else clearFlag(carry);
    if(indexregY_ == memory[address]) setFlag(zero);
    else clearFlag(zero);
    if(temp & negative) setFlag(negative);
    else clearFlag(negative);
}

// Branch instructions: BCC	BCS	BEQ	BNE	BPL	BMI	BVC	BVS
void cpu::BPL(uint8_t, uint8_t) {
    if (!(statusReg_ & negative)) {
        uint8_t offset = memory[prgrmCtr_++];
        prgrmCtr_ += offset;
    }
}

void cpu::BMI(uint8_t, uint8_t) {
    if (statusReg_ & negative) {
        uint8_t offset = memory[prgrmCtr_++];
        prgrmCtr_ += offset;
    }
}

void cpu::BVC(uint8_t, uint8_t) {
    if (!(statusReg_ & overflow)) {
        uint8_t offset = memory[prgrmCtr_++];
        prgrmCtr_ += offset;
    }
}

void cpu::BVS(uint8_t, uint8_t) {
    if (statusReg_ & overflow) {
        uint8_t offset = memory[prgrmCtr_++];
        prgrmCtr_ += offset;
    }
}

void cpu::BCC(uint8_t, uint8_t) {
    if (!(statusReg_ & carry)) {
        uint8_t offset = memory[prgrmCtr_++];
        prgrmCtr_ += offset;
    }
}

void cpu::BCS(uint8_t, uint8_t) {
    if (statusReg_ & carry) {
        uint8_t offset = memory[prgrmCtr_++];
        prgrmCtr_ += offset;
    }
}

void cpu::BNE(uint8_t, uint8_t) {
    if (!(statusReg_ & zero)) {
        uint8_t offset = memory[prgrmCtr_++];
        prgrmCtr_ += offset;
    }
}

void cpu::BEQ(uint8_t, uint8_t) {
    if (statusReg_ & zero) {
        uint8_t offset = memory[prgrmCtr_++];
        prgrmCtr_ += offset;
    }
}


// Jump instructions: JMP JSR RTS BRK RTI			
void cpu::JMP_abs(uint8_t, uint8_t){
    prgrmCtr_ = getAddress(absolute, 0);
}
void cpu::JMP_indr(uint8_t, uint8_t){
    uint8_t low = memory[prgrmCtr_++];
    uint8_t high = memory[prgrmCtr_++];
    uint16_t address = (high << 8) | low;

    low = memory[address++];
    high = memory[address];
    prgrmCtr_ = (high << 8) | low;
} 

void cpu::JSR(uint8_t, uint8_t){
    uint16_t address = getAddress(absolute, 0);

    uint16_t return_address = prgrmCtr_ - 1;
    uint8_t high = (return_address >> 8) & 0xFF;
    uint8_t low = return_address & 0xFF;

    memory[stkPtr_--] = high;
    memory[stkPtr_--] = low;

    prgrmCtr_ = address;
} 

void cpu::RTS(uint8_t, uint8_t){
    prgrmCtr_ = memory[stkPtr_++];
    prgrmCtr_++;
} 

void cpu::BRK(uint8_t, uint8_t){
    ++prgrmCtr_;
    uint8_t high = (prgrmCtr_ >> 8) & 0xFF;
    uint8_t low = prgrmCtr_ & 0xFF;
    setFlag(bFlag);
    memory[stkPtr_--] = high;
    memory[stkPtr_--] = low;
    memory[stkPtr_--] = statusReg_;

    low = memory[0xFFFE];
    high = memory[0xFFFF];
    prgrmCtr_ = (high << 8) | low;

    setFlag(intrptDisable);
} 

void cpu::RTI(uint8_t, uint8_t){
    statusReg_ = memory[stkPtr_++];
    uint8_t low = memory[stkPtr_++];
    uint8_t high = memory[stkPtr_++];
    
    prgrmCtr_ = (high << 8) | low;
} 

// cpu::Stack instruction: PHA PLA PHP PLP TXS TSX			
void cpu::PHA(uint8_t, uint8_t){
    memory[stkPtr_--] = acc_;
} 

void cpu::PLA(uint8_t, uint8_t){
    acc_ = memory[++stkPtr_];

    if(acc_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(acc_ & negative) setFlag(negative);
    else clearFlag(negative);
} 

void cpu::PHP(uint8_t, uint8_t){
    memory[stkPtr_--] = (statusReg_ | 0b00110000);
} 

void cpu::PLP(uint8_t, uint8_t){
    statusReg_ = (memory[++stkPtr_] & 0b11001111);
} 

void cpu::TXS(uint8_t, uint8_t){
    memory[stkPtr_--] = indexregX_;
} 

void cpu::TSX(uint8_t, uint8_t){
    indexregX_ = memory[++stkPtr_];
    
    if(indexregX_ == 0) setFlag(zero);
    else clearFlag(zero);
    if(indexregX_ & negative) setFlag(negative);
    else clearFlag(negative);
} 

// Flags instructions:  CLC	SEC	CLI	SEI	CLD	SED	CLV	
void cpu::CLC(uint8_t, uint8_t){
    clearFlag(carry);
} 
void cpu::SEC(uint8_t, uint8_t){
    setFlag(carry);
} 
void cpu::CLI(uint8_t, uint8_t){
    clearFlag(intrptDisable);
} 
void cpu::SEI(uint8_t, uint8_t){
    setFlag(intrptDisable);
} 
void cpu::CLD(uint8_t, uint8_t){
    clearFlag(decimal);
} 
void cpu::SED(uint8_t, uint8_t){
    setFlag(decimal);
} 
void cpu::CLV(uint8_t, uint8_t){
    clearFlag(overflow);
}

// Other instructions: NOP	
void cpu::NOP(uint8_t, uint8_t){

}