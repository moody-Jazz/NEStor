#include "cstdint"
#include "vector"
#include "fstream"
#include "iostream"

extern uint8_t memory[65535];
extern std::vector<std::pair<int, int>> argList[256];
void loadRom(const std::string filename);
