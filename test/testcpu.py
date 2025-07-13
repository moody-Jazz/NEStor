import json
from sys import argv
import os
import subprocess

def test(instruction, itr = 15):

    testfile = open(os.path.join('cpu_test_suit', instruction))
    registers_keys = ['pc', 's', 'a', 'x', 'y', 'p']
    test_count = 1

    os.chdir("..")
    while itr:
        outfile = open('build_test/testval.txt', 'w')
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

        outfile.close()
        try:
            os.chdir('build_test')
            subprocess.run('NEStor')
            os.chdir("..")

        except subprocess.CalledProcessError as e:
            print('error running the test script')
            exit(1)

        # read the updated file to extract the final register and memory values
        infile = open('build_test/testval.txt', 'r')
        final_reg = infile.readline().strip().rstrip('\n').split(' ')
        actual_cycle_count = int(infile.readline().strip().rstrip('\n'))
        final_mem = []
        
        while True:
            temp_string = infile.readline()
            temp_string = temp_string.strip().rstrip('\n')
            if temp_string == '':
                break
            final_mem.extend([temp_string.split(' ')])
        
        # convert the read values into integer for comparision
        final_reg = list(map(int, final_reg))
        final_mem = [[int(a), int(b)] for a, b in final_mem]
        infile.close()

        # compare expected and actual values to determine if Test was sucessful or not
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

        # print('🟢  Test passed for test count', test_count)
        test_count += 1
        itr -= 1
    os.chdir("test/")

def main():
    args = argv[1:]

    os.chdir("..")
    print(os.getcwd())

    print('cmake', '-G', '\"MinGW Makefiles\"', '-B', 'build_test', '-S',
        '.', '-DBUILD_TESTS=ON', '-DTEST_FILE=test/testcpu.cpp')
    subprocess.run([
        'cmake',
        '-G', 'MinGW Makefiles',
        '-B', 'build_test',
        '-S', '.',
        '-DBUILD_TESTS=ON',
        '-DTEST_FILE=test/testcpu.cpp'
    ])
    print('cmake', '--build', 'build_test')
    subprocess.run(['cmake', '--build', 'build_test']) 

    os.chdir("test/")

    itr = int(input('How many test cases you want to test against: '))
    
    if 'cpu_test_suit' in os.listdir(os.getcwd()):
        json_file = os.listdir('cpu_test_suit')
    else:
        print("Use the install_cputests script to install all the test first")
        exit(1)

    # For testing against specific instruction
    if len(args) > 0 and args[0] == 'single':
        file = input('Name of test file: ')
        print(f"🟢 starting testing for {file}")
        test(file, itr)
        print(f"🟢 All test passed for {file}")
        exit(1)
        

    # For testing against all the instructions for which jsons are available
    for file in json_file:
        print(f"🟢 starting testing for {file}")
        test(file, itr)
        print(f"🟢 All test passed for {file}")


if __name__ == '__main__':
    main()