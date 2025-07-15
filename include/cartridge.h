#pragma once

#include <cstdint>
#include <vector>
#include <fstream>
#include <iostream>
#include "cpuTypes.h"

using namespace CpuTypes;

void loadRom(const std::string filename);
void loadOpcodedata(std::vector<std::pair<ADDRESSING_MODES, REGISTERS>>& table);
ADDRESSING_MODES stringToMode(const std::string& modeStr);
REGISTERS stringToReg(const std::string& regStr);
