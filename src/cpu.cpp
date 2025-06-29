#include "cpu.hpp"

cpu::cpu(): argList(256){
    stkPtr_ = ZERO_PAGE_ENDING;
    prgrmCtr_ = 0x0;
    statusReg_ |= one;

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

void cpu::executeInstruction(){
    opcode_ = memory[prgrmCtr_++];
    
    args = argList[opcode_];
    (this->*opcodeMap[opcode_])(args.first, args.second);
}

void cpu::printInfo(){
    std::cout<<"prgrm counter: "<<prgrmCtr_<<"\n";
    std::cout<<"index x and y reg: "<<indexregX_<<" "<<indexregY_<<"\n";
    std::cout<<"stack pointer: "<<STACK_PAGE_STARTING + stkPtr_<<"\n\n";
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
void cpu::setPc(uint16_t pc){
    prgrmCtr_ = pc;
}
void cpu::setStkPtr(uint16_t stk){
    stkPtr_ = stk;
}
void cpu::setAcc(uint8_t acc){
    acc_ = acc;
}
void cpu::setX(uint8_t x){
    indexregX_ = x;
}
void cpu::setY(uint8_t y){
    indexregY_ = y;
}
void cpu::setStatus(uint8_t status){
    statusReg_ = status;
}
uint16_t cpu::getPc(){
    return prgrmCtr_;
}
uint16_t cpu::getStkPtr(){
    return stkPtr_;
}
uint8_t cpu::getAcc(){
    return acc_;
}
uint8_t cpu::getX(){
    return indexregX_;
}
uint8_t cpu::getY(){
    return indexregY_;
}
uint8_t cpu::getStatus(){
    return statusReg_;
}

void cpu::updateZNflag(uint8_t reg, uint8_t val){
    if(reg == val) setFlag(zero);
    else clearFlag(zero);
    if(reg & negative) setFlag(negative);
    else clearFlag(negative);
}

uint16_t cpu::getAddress(uint8_t mode, uint8_t offset){
    uint16_t address{};
    uint8_t low{}, high{};

    if(mode == indirect){
        uint8_t baseAddr = memory[prgrmCtr_++];

        if(offset == X){
            low = memory[(baseAddr + indexregX_) % 256];
            high = memory[(baseAddr + indexregX_ + 1) % 256];
            address = (high << 8) | low;
        }
        else{
            low = memory[baseAddr];
            high = memory[(baseAddr + 1) % 256];
            address = ((high << 8) | low) + indexregY_;
        }
    }

    if(offset == X)
        offset = indexregX_;
        
    else if(offset == Y)
        offset = indexregY_;

    if(mode == immediate)
        address = prgrmCtr_++;

    else if(mode == zeroPage)
        address = (memory[prgrmCtr_++] + offset) % 256;

    else if(mode == absolute){
        low = memory[prgrmCtr_++];
        high = memory[prgrmCtr_++];
        address = (high << 8) | low;
        address += offset;
    }

    return address;
}

// Access instructions: LDA	STA	LDX	STX	LDY	STY		
void cpu::LDA(uint8_t mode, uint8_t offset){
    acc_ = memory[getAddress(mode, offset)];
    updateZNflag(acc_, 0);
}

void cpu::STA(uint8_t mode, uint8_t offset){
    memory[getAddress(mode, offset)] = acc_;
}

void cpu::LDX(uint8_t mode, uint8_t offset){
    indexregX_ = memory[getAddress(mode, offset)];
    updateZNflag(indexregX_, 0);
}

void cpu::STX(uint8_t mode, uint8_t offset){
    memory[getAddress(mode, offset)] = indexregX_;
}
 
void cpu::LDY(uint8_t mode, uint8_t offset){
    indexregY_ = memory[getAddress(mode, offset)];
    updateZNflag(indexregY_, 0);
}

void cpu::STY(uint8_t mode, uint8_t offset){
    memory[getAddress(mode, offset)] = indexregY_;
}

// Transfer instructions: TAX TXA TAY TYA		
void cpu::TAX(uint8_t, uint8_t){
    indexregX_ = acc_;
    updateZNflag(indexregX_, 0);
} 
void cpu::TXA(uint8_t, uint8_t){
    acc_ = indexregX_;
    updateZNflag(acc_, 0);
} 
void cpu::TAY(uint8_t, uint8_t){
    indexregY_ = acc_;
    updateZNflag(indexregY_, 0);
} 
void cpu::TYA(uint8_t, uint8_t){
    acc_ = indexregY_;
    updateZNflag(acc_, 0);
} 

// Arithmetic instructions: ADC	SBC	INC	DEC	INX	DEX	INY	DEY
void cpu::ADC(uint8_t mode, uint8_t offset){
    uint8_t memval = memory[getAddress(mode, offset)];
    uint8_t accval = acc_;
    uint16_t sum = acc_ + memval + getFlag(carry);
    acc_ = sum;

    if(sum > 0xFF) setFlag(carry);
    else clearFlag(carry);
    if((sum ^ accval) & (sum ^ memval) & negative) setFlag(overflow);
    else clearFlag(overflow);
    updateZNflag(acc_, 0);
}

void cpu::SBC(uint8_t mode, uint8_t offset){
    uint8_t memval = memory[getAddress(mode, offset)];
    uint8_t accval = acc_;
    uint16_t sum = acc_ - memval - !getFlag(carry);

    acc_ = sum;

    if(sum > 0xFF) clearFlag(carry);
    else setFlag(carry);
    if((sum ^ accval) & (sum ^ ~memval) & negative) setFlag(overflow);
    else clearFlag(overflow);
    updateZNflag(acc_, 0);
}  

void cpu::INC(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    memory[address]++;
    updateZNflag(memory[address], 0);
}

void cpu::DEC(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    memory[address]--;
    updateZNflag(memory[address], 0);
}

void cpu::INX(uint8_t, uint8_t){
    indexregX_++;
    updateZNflag(indexregX_, 0);
} 
void cpu::DEX(uint8_t, uint8_t){
    indexregX_--;
    updateZNflag(indexregX_, 0);
} 
void cpu::INY(uint8_t, uint8_t){
    indexregY_++;
    updateZNflag(indexregY_, 0);
} 
void cpu::DEY(uint8_t, uint8_t){
    indexregY_--;
    updateZNflag(indexregY_, 0);
} 

// Shift instructions : ASL	LSR	ROL	ROR
void cpu::ASL_acc(uint8_t, uint8_t){
    if(acc_ & negative) setFlag(carry);
    else clearFlag(carry);

    acc_ <<= 1;
    updateZNflag(acc_, 0);
}

void cpu::ASL(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    if(memory[address] & negative) setFlag(carry);
    else clearFlag(carry);

    memory[address] <<= 1;
    updateZNflag(memory[address], 0);
}

void cpu::LSR_acc(uint8_t, uint8_t){
    if(acc_ & carry) setFlag(carry);
    else clearFlag(carry);

    acc_ >>= 1;
    updateZNflag(acc_, 0);
    clearFlag(negative);
}
void cpu::LSR(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    if(memory[address] & carry) setFlag(carry);
    else clearFlag(carry);

    memory[address] >>= 1;
    updateZNflag(memory[address], 0);
    clearFlag(negative);
}

void cpu::ROL_acc(uint8_t, uint8_t){
    bool prevCarry = getFlag(carry);
    if(acc_ & negative) setFlag(carry);
    else clearFlag(carry);

    acc_ <<= 1;
    acc_ |= prevCarry;
    updateZNflag(acc_, 0);
}

void cpu::ROL(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    bool prevCarry = getFlag(carry);
    if(memory[address] & negative) setFlag(carry);
    else clearFlag(carry);

    memory[address] <<= 1;
    memory[address] |= prevCarry;
    updateZNflag(memory[address], 0);
}

void cpu::ROR_acc(uint8_t, uint8_t){
    bool prevCarry = getFlag(carry);
    if(acc_ & carry) setFlag(carry);
    else clearFlag(carry);

    acc_ >>= 1;
    acc_ |= (prevCarry << 7);
    updateZNflag(acc_, 0);   
}
void cpu::ROR(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
    bool prevCarry = getFlag(carry);
    if(memory[address] & carry) setFlag(carry);
    else clearFlag(carry);

    memory[address] >>= 1;
    memory[address] |= (prevCarry << 7);
    updateZNflag(memory[address], 0);
}

// Bitwise instructions: AND ORA EOR BIT				
void cpu::AND(uint8_t mode, uint8_t offset){
    acc_ &= memory[getAddress(mode, offset)];
    updateZNflag(acc_, 0);   
}

void cpu::ORA(uint8_t mode, uint8_t offset){
    acc_ |= memory[getAddress(mode, offset)];
    updateZNflag(acc_, 0);   
}

void cpu::EOR(uint8_t mode, uint8_t offset){
    acc_ ^= memory[getAddress(mode, offset)];
    updateZNflag(acc_, 0);   
}

void cpu::BIT(uint8_t mode, uint8_t offset){
    uint16_t address = getAddress(mode, offset);
  
    if((acc_ & memory[address]) == 0) setFlag(zero);
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
    uint8_t offset = memory[prgrmCtr_++];

    if (!(statusReg_ & negative)) 
        prgrmCtr_ += static_cast<int8_t>(offset);
}

void cpu::BMI(uint8_t, uint8_t) {
    uint8_t offset = memory[prgrmCtr_++];

    if (statusReg_ & negative)
        prgrmCtr_ += static_cast<int8_t>(offset);    
}

void cpu::BVC(uint8_t, uint8_t) {
    uint8_t offset = memory[prgrmCtr_++];

    if (!(statusReg_ & overflow)) 
        prgrmCtr_ += static_cast<int8_t>(offset);
}

void cpu::BVS(uint8_t, uint8_t) {
    uint8_t offset = memory[prgrmCtr_++];

    if (statusReg_ & overflow)
        prgrmCtr_ += static_cast<int8_t>(offset);
}

void cpu::BCC(uint8_t, uint8_t) {
    uint8_t offset = memory[prgrmCtr_++];

    if (!(statusReg_ & carry)) 
        prgrmCtr_ += static_cast<int8_t>(offset);
}

void cpu::BCS(uint8_t, uint8_t) {
    uint8_t offset = memory[prgrmCtr_++];

    if (statusReg_ & carry) 
        prgrmCtr_ += static_cast<int8_t>(offset);
}

void cpu::BNE(uint8_t, uint8_t) {
    uint8_t offset = memory[prgrmCtr_++];

    if (!(statusReg_ & zero)) 
        prgrmCtr_ += static_cast<int8_t>(offset);
}

void cpu::BEQ(uint8_t, uint8_t) {
    uint8_t offset = memory[prgrmCtr_++];

    if (statusReg_ & zero) 
        prgrmCtr_ += static_cast<int8_t>(offset);
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

    memory[STACK_PAGE_STARTING + stkPtr_--] = high;
    memory[STACK_PAGE_STARTING + stkPtr_--] = low;

    prgrmCtr_ = address;
} 

void cpu::RTS(uint8_t, uint8_t){
    uint8_t low = memory[STACK_PAGE_STARTING + ++stkPtr_];
    uint8_t high = memory[STACK_PAGE_STARTING + ++stkPtr_];

    prgrmCtr_ = (high << 8) | low;
    prgrmCtr_++;
} 

void cpu::BRK(uint8_t, uint8_t){
    ++prgrmCtr_;
    
    uint8_t high = (prgrmCtr_ >> 8) & 0xFF;
    uint8_t low = prgrmCtr_ & 0xFF;
    
    memory[STACK_PAGE_STARTING + stkPtr_--] = high;
    memory[STACK_PAGE_STARTING + stkPtr_--] = low;
    memory[STACK_PAGE_STARTING + stkPtr_--] = statusReg_ | bFlag;
    
    setFlag(intrptDisable);
    low = memory[0xFFFE];
    high = memory[0xFFFF];
    prgrmCtr_ = (high << 8) | low;
} 

void cpu::RTI(uint8_t, uint8_t){
    statusReg_ = memory[STACK_PAGE_STARTING + ++stkPtr_] | one;
    clearFlag(bFlag);
    
    uint8_t low = memory[STACK_PAGE_STARTING + ++stkPtr_];
    uint8_t high = memory[STACK_PAGE_STARTING + ++stkPtr_];
    
    prgrmCtr_ = (high << 8) | low;
} 

// cpu::Stack instruction: PHA PLA PHP PLP TXS TSX			
void cpu::PHA(uint8_t, uint8_t){
    memory[STACK_PAGE_STARTING + stkPtr_--] = acc_;
} 

void cpu::PLA(uint8_t, uint8_t){
    acc_ = memory[STACK_PAGE_STARTING + ++stkPtr_];

    updateZNflag(acc_, 0);
} 

void cpu::PHP(uint8_t, uint8_t){
    memory[STACK_PAGE_STARTING + stkPtr_--] = (statusReg_ | 0b00110000);
} 

void cpu::PLP(uint8_t, uint8_t){
    statusReg_ &= 0b00110000; 
    statusReg_ |= memory[STACK_PAGE_STARTING + ++stkPtr_] & 0b11001111;
} 

void cpu::TXS(uint8_t, uint8_t){
    stkPtr_ = indexregX_;
} 

void cpu::TSX(uint8_t, uint8_t){
    indexregX_ = stkPtr_;
    
    updateZNflag(indexregX_, 0);
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
void cpu::NOP(uint8_t, uint8_t){}