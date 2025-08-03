#pragma once

#include <iostream>
#include <cstdint>
#include <vector>
#include "cputypes.h"

using namespace CpuTypes;

inline constexpr uint16_t KILO_BYTE = 1024;

void loadOpcodedata(
  std::vector<std::pair<ADDRESSING_MODES, REGISTERS>> &table);

ADDRESSING_MODES stringToMode(const std::string &modeStr);
REGISTERS stringToReg(const std::string &regStr);