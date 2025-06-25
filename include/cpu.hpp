#include "global.hpp"

constexpr uint8_t ZERO_PAGE_STARTING    = 0X00;
constexpr uint8_t ZERO_PAGE_ENDING      = 0XFF;
constexpr uint16_t STACK_PAGE_STARTING  = 0X0100;
constexpr uint16_t STACK_PAGE_ENDING    = 0X01FF;

class cpu
{
public:
    cpu();
    ~cpu();

    enum STATUS_FLAGS{
        carry           = 0b00000001,
        zero            = 0b00000010,
        intrptDisable   = 0b00000100,
        decimal         = 0b00001000,
        bFlag           = 0b00010000,
        one             = 0b00100000, 
        overflow        = 0b01000000,
        negative        = 0b10000000,
    };

    enum ADDRESSING_MODES{
        immediate,
        zeroPage,
        absolute,
        indirect, 
        relative, 
    };

    bool getFlag(uint8_t flag);
    void clearFlag(uint8_t flag);
    void setFlag(uint8_t flag);
    uint16_t getAddress(uint8_t mode, uint8_t offset);

    void LDA(uint8_t mode, uint8_t offset);	
    void STA(uint8_t mode, uint8_t offset);	
    void LDX(uint8_t mode, uint8_t offset);	
    void STX(uint8_t mode, uint8_t offset);	
    void LDY(uint8_t mode, uint8_t offset);	
    void STY(uint8_t mode, uint8_t offset);		
    void TAX();	
    void TXA();	
    void TAY();	
    void TYA();				
    void ADC(uint8_t mode, uint8_t offset);	
    void SBC(uint8_t mode, uint8_t offset);	
    void INC(uint8_t mode, uint8_t offset);	
    void DEC(uint8_t mode, uint8_t offset);	
    void INX();	
    void DEX();	
    void INY();	
    void DEY();
    void ASL_acc();
    void ASL(uint8_t mode, uint8_t offset);	
    void LSR_acc();
    void LSR(uint8_t mode, uint8_t offset);	
    void ROL_acc();
    void ROL(uint8_t mode, uint8_t offset);	
    void ROR_acc();
    void ROR(uint8_t mode, uint8_t offset);				
    void AND(uint8_t mode, uint8_t offset);	
    void ORA(uint8_t mode, uint8_t offset);	
    void EOR(uint8_t mode, uint8_t offset);	
    void BIT(uint8_t mode, uint8_t offset);				
    void CMP(uint8_t mode, uint8_t offset);	
    void CPX(uint8_t mode, uint8_t offset);	
    void CPY(uint8_t mode, uint8_t offset);					
    void BRANCH(bool condition);
    void JMP_abs();
    void JMP_indr();
    void JSR();
    void RTS();
    void BRK();
    void RTI();
    void PHA();
    void PLA();
    void PHP();
    void PLP();
    void TXS();
    void TSX();
    void CLC();
    void SEC();
    void CLI();
    void SEI();
    void CLD();
    void SED();
    void CLV();
    void NOP();

private:
    uint8_t acc_;
    uint8_t indexregX_, indexregY_;

    uint16_t prgrmCtr_;
    uint8_t  stkPtr_;

    uint8_t statusReg_;

    uint8_t opcode_;
};