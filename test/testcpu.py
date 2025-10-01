import json
from sys import argv
import os
import subprocess

def test(instruction, itr=15):
    testfile_path = os.path.join('cpu_test_suit', instruction)
    with open(testfile_path) as testfile:
        registers_keys = ['pc', 's', 'a', 'x', 'y', 'p']
        test_count = 1

        while itr:
            with open('testval.txt', 'w') as outfile:
                test_instance = testfile.readline()
                if test_instance.startswith('[') or test_instance.startswith(']'):
                    continue
                if test_instance == '':
                    break
                test_instance = test_instance.strip().rstrip(',')
                test_dic = json.loads(test_instance)
                expected_cycle_count = len(test_dic['cycles'])

                # write all the register and memory values for test.cpp to read and set cpu values
                init_reg_values = [test_dic['initial'][key] for key in registers_keys]
                expected_mem = []
                expected_reg = [test_dic['final'][key] for key in registers_keys]

                for val in init_reg_values:
                    outfile.write(f'{val} ')
                outfile.write('\n')

                for val in test_dic['initial']['ram']:
                    outfile.write(f'{val[0]} {val[1]}\n')
                outfile.write('\n')

                for val in test_dic['final']['ram']:
                    expected_mem.append([val[0], val[1]])
                    outfile.write(f'{val[0]} {val[1]}\n')

            try:
                os.chdir("..")
                subprocess.run(
                    ['./MOS6502', os.path.join('test', 'testval.txt')],
                    check=True
                )
            except subprocess.CalledProcessError as e:
                print('error running the test script')
                exit(1)

            # read the updated file to extract the final register and memory values
            os.chdir("test/")
            with open('testval.txt', 'r') as infile:
                final_reg = infile.readline().strip().split(' ')
                actual_cycle_count = int(infile.readline().strip())
                final_mem = []

                while True:
                    temp_string = infile.readline()
                    temp_string = temp_string.strip()
                    if temp_string == '':
                        break
                    final_mem.append(temp_string.split(' '))

            final_reg = list(map(int, final_reg))
            final_mem = [[int(a), int(b)] for a, b in final_mem]

            if actual_cycle_count != expected_cycle_count:
                print(f"🔴 test case no {test_count} failed for instruction {instruction}")
                print("🔴 cycle count doesn't match")
                exit(1)

            for i in range(len(final_reg)):
                if final_reg[i] != expected_reg[i]:
                    print(f"🔴 test case no {test_count} failed for instruction {instruction}")
                    print("🔴 Register values don't match")
                    print('register order')
                    print(registers_keys)
                    print('Initial:', init_reg_values)
                    print('Expected:', expected_reg, '\nActual:', final_reg)
                    print(f'Expected:', expected_mem, '\nActual:', final_mem)
                    exit(1)

            for i in range(len(final_mem)):
                if final_mem[i][1] != expected_mem[i][1]:
                    print(f"🔴 test case no {test_count} failed for instruction {instruction}")
                    print("🔴 Memory values don't match")
                    print(f'for address {expected_mem[i][0]} Expected: {expected_mem[i][1]}')
                    print(f'for address {final_mem[i][0]} Actual: {final_mem[i][1]}')
                    print(f'Expected:', expected_mem, '\nActual:', final_mem)
                    print('Expected:', expected_reg, '\nActual:', final_reg)
                    exit(1)

            test_count += 1
            itr -= 1

def main():
    args = argv[1:]

    os.chdir("..")
    print("Working directory:", os.getcwd())

    print("Compiling using g++...")
    compile_cmd = [
        "g++",
        "-std=c++17",
        "-Iinclude",
        os.path.join("src", "basememory.cpp"),
        os.path.join("src", "cpu.cpp"),
        os.path.join("src", "helper.cpp"),
        os.path.join("test", "testcpu.cpp"),
        "-o",
        "MOS6502"
    ]
    print("Running:", ' '.join(compile_cmd))
    result = subprocess.run(compile_cmd)

    if result.returncode != 0:
        print("Compilation failed.")
        exit(1)
    print("Compilation succeeded.")

    os.chdir("test/")

    try:
        itr = int(input('How many test cases you want to test against: '))
    except ValueError:
        print("Invalid number")
        exit(1)

    if 'cpu_test_suit' in os.listdir(os.getcwd()):
        json_file = os.listdir('cpu_test_suit')
    else:
        print("!! Use the install_cputests script to install all the tests first.")
        exit(1)

    if len(args) > 0 and args[0] == 'single':
        file = input('Name of test file: ')
        print(f"🟢 Starting testing for {file}")
        test(file, itr)
        print(f"✅ All tests passed for {file}")
        exit(0)

    for file in json_file:
        print(f"🟢 Starting testing for {file}")
        test(file, itr)
        print(f"✅ All tests passed for {file}")

if __name__ == '__main__':
    main()