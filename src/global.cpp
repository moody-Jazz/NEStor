#include "global.h"
#include <sstream>

uint8_t memory[65535];

void loadRom(const std::string filename) {
  std::ifstream file;
  file.open(filename, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "couldn't open the ROM file\n";
    return;
  }

  file.seekg(0, file.end);
  size_t size = file.tellg();
  file.seekg(0, file.beg);

  std::vector<char> buffer(size);
  std::cout << size << ": is the size of the rom file\n";
  if (!file.read(buffer.data(), size)) {
    std::cerr << "Failed to read the ROM file data\n";
    return;
  }

  for (int i{}; i < size; i++) memory[0x0 + i] = buffer[i];

  file.close();
}

void loadOpcodedata(std::vector<std::pair<ADDRESSING_MODES, REGISTERS>>& table){
  std::ifstream file;
  file.open("opcodedata.txt", std::ios::in);
  std::string line;
  std::string opcodeStr, modeStr, regStr;
  
  if (!file.is_open()) {
    std::cerr << "couldn't open the opcodedata file\n";
    return;
  }
  
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    
    std::istringstream iss(line);
    
    if (iss >> opcodeStr >> modeStr >> regStr) {
      uint8_t opcode = std::stoi(opcodeStr, nullptr, 16);
      ADDRESSING_MODES mode = stringToMode(modeStr);
      REGISTERS reg = stringToReg(regStr);
      
      table[opcode] = {mode, reg};
    }
  }
}

ADDRESSING_MODES stringToMode(const std::string& modeStr){
  if (modeStr == "acc") return acc;
  if (modeStr == "immediate") return immediate;
  if (modeStr == "zeroPage") return zeroPage;
  if (modeStr == "absolute") return absolute;
  if (modeStr == "indirect") return indirect;
  if (modeStr == "relative") return relative;
  return implied;
}

REGISTERS stringToReg(const std::string& regStr){
  if (regStr == "A") return A;
  if (regStr == "X") return X;
  if (regStr == "Y") return Y;
  if (regStr == "PC") return PC;
  if (regStr == "SR") return SR;
  return NA;
}
