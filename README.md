# CPU6502

This is a modular 6502 CPU emulator that can be integrated into emulator projects.  
It supports full instruction execution, cycle tracking, and pluggable memory mapping.  
A JSON-based test suite is also included to verify correctness.

## Features
- Full 6502 instruction set
- Cycle accurate execution
- Opcode argument mapping via `opcodedata.txt`
- Abstract memory interface (`BaseMemory`)
- JSON-based automated test suite

## Project Structure
```
basememory.h          # Abstract memory interface
cpu.h                 # CPU class
cpu.cpp               # CPU implementation
helper.h              # Supporting enums/macros
opcodedata.txt        # Opcode addressing modes
install_cputests.py   # Downloads JSON test cases
testcpu.py            # Runs tests
cpu_test_suit/        # Directory created for JSON test cases
```

## Integration Steps

### 1. Add Files
Copy all the files from include and src directory along with opcodedata.txt

### 2. Implement Memory
```cpp
#include "basememory.h"

class MyMemory : public BaseMemory {
public:
    MyMemory(uint32_t size) : BaseMemory(size) {}

    uint8_t read(uint16_t addr) override {
        return mainRam_[addr];  // or custom mapping
    }

    void write(uint16_t addr, uint8_t value) override {
        mainRam_[addr] = value; // with mirroring / IO logic
    }
};
```

### 3. Initialize CPU
```cpp
#include "cpu.h"
#include <memory>

int main() {
    std::unique_ptr<BaseMemory> memory = std::make_unique<MyMemory>(0x10000);
    Cpu cpu(std::move(memory));

    cpu.reset();
    cpu.executeInstruction();
    cpu.printInfo();
    return 0;
}
```

### 4. Emulation Loop
```cpp
while (running) {
    cpu.executeInstruction();
    // integrate with graphics, audio, timers, etc.
}
```

### 5. Opcode Data
`opcodedata.txt` defines addressing modes for each opcode:
```
0x00 implied NA
0x01 indirect X
0x05 zeroPage NA
0x06 zeroPage NA
0x08 implied NA
```
This is neccessary and used during CPU construction.

### 6. Testing
1. Install tests:
   ```bash
   python install_cputests.py
   ```
   Creates `cpu_test_suit/` and install all the files from [SingleStepTests](https://github.com/SingleStepTests/ProcessorTests/tree/main/6502/v1). There are 10,000 test cases for each individual instruction.

2. Run tests:
   ```bash
   python testcpu.py
   ```
   Each test sets CPU state, executes one instruction, and verifies result.
