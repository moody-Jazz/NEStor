#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include "cpu.hpp"

void runTest(Cpu& testcpu){
    std::string filename = "testval.txt";

    std::ifstream ifile(filename);
    std::vector<size_t> address;
    bool readingFinalBlock = false;

    if (!ifile)
    {
        std::cerr << "Failed to open file\n";
        return;
    }

    std::string line;
    if (std::getline(ifile, line))
    {
        std::istringstream iss(line);

        // Read register values: pc stk acc x y status
        uint16_t pc, stk, acc, x, y, status;
        iss >> pc >> stk >> acc >> x >> y >> status;

        testcpu.setPc(pc);
        testcpu.setStkPtr(stk);
        testcpu.setAcc(acc);
        testcpu.setX(x);
        testcpu.setY(y);
        testcpu.setStatus(status);
    }

    // Process rest of the lines
    while (std::getline(ifile, line))
    {
        if (line.empty())
        {
            readingFinalBlock = true;
            continue; // move to next line
        }

        std::istringstream iss(line);
        int addr, val;

        if (iss >> addr >> val)
        {
            if (readingFinalBlock)
                address.push_back(addr); // final check memory address
        
            else
                testcpu.cpuWrite(addr, val); // initial memory load
        }
    }

    testcpu.executeInstruction();

    std::ofstream ofile(filename);
    if (!ofile)
    {
        std::cerr << "Failed to open file for writing\n";
        return;
    }

    // Write registers
    ofile << +testcpu.getPc() << " " << +testcpu.getStkPtr() << " " << +testcpu.getAcc() << " "
        << +testcpu.getX() << " " << +testcpu.getY() << " " << +testcpu.getStatus() << " \n";

    // Write updated addresses
    for (uint16_t addr : address)
        ofile << addr << " " << +testcpu.cpuRead(addr) << "\n";
    
    ifile.close();
    ofile.close();
}

int main()
{
    Cpu testcpu;
    runTest(testcpu);
}
