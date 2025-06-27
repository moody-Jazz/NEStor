#include "global.hpp"

uint8_t memory[65535];

void loadRom(const std::string filename){
    std::ifstream file;
    file.open(filename, std::ios::binary);
    
    if (!file.is_open()){
        std::cerr << "couldn't open the ROM file\n";
        return;
    }

    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);

    std::vector<char> buffer(size);
    std::cout<<size<<": is the size of the rom file\n";
    if (!file.read(buffer.data(), size)) {
        std::cerr << "Failed to read the ROM file data\n";
        return;
    }

    for (int i{}; i < size; i++)
        memory[0x0 + i] = buffer[i];
        
    file.close();
}