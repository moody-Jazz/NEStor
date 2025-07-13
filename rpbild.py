import subprocess
import os
import shutil
import stat

root_dir            = os.getcwd()
raylib_dir          = "raylib"
build_raylib_cmd    = ['make', 'PLATFORM=PLATFORM_DESKTOP']
static_raylib_name  = "libraylib.a"
raylib_header_name  = "raylib.h"

# this functions is useful for deleting cloned raylib folder after building raylib 
def rmtree(top):
    for root, dirs, files in os.walk(top, topdown=False):
        for name in files:
            filename = os.path.join(root, name)
            os.chmod(filename, stat.S_IWUSR)
            os.remove(filename)
        for name in dirs:
            os.rmdir(os.path.join(root, name))
    os.rmdir(top)  
    

def init_project():
    global root_dir , raylib_dir, build_raylib_cmd, static_raylib_name, raylib_header_name
    dir_list = os.listdir()

    # check if host system has make
    try:
        subprocess.run(['make', '--version'], check=True, stdout=subprocess.DEVNULL)
    except FileNotFoundError:
        print('no make found')
        exit(1)
    except subprocess.CalledProcessError:
        print('unexpected error occured while running make --version')
        exit(1)

    # clone and build raylib
    clone_repo = ['git', 'clone', "https://github.com/raysan5/raylib"]
    subprocess.run(clone_repo)
    raylib_src = os.path.join(raylib_dir, 'src')

    os.chdir(raylib_src)
    print('building static library libraylib...........')
    print(build_raylib_cmd)
    subprocess.run(' '.join(build_raylib_cmd))

    staticlib_src       = os.path.join(raylib_src, static_raylib_name)
    staticlib_dest      = 'lib'
    raylibheader_src    = os.path.join(raylib_src, raylib_header_name)
    raylibheader_dest   = 'include'
    
    os.chdir(root_dir)
    print('copying the header file and static library................')
    if 'include' not in dir_list:
        os.makedirs(name = 'include')
    if 'lib' not in dir_list:
        os.makedirs(name = 'lib')

    shutil.copy(staticlib_src, staticlib_dest)
    shutil.copy(raylibheader_src, raylibheader_dest)
    
    print('Deleting raylib...............')
    rmtree('raylib')


def main():
    init_project()

if __name__ == "__main__":
    main()
    