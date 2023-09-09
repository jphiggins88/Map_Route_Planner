// Project Repo
git clone https://github.com/udacity/CppND-Route-Planning-Project.git --recurse-submodules

// IO2D Repo
https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md

To get this to work on Linux Ubuntu, do the folowing:

sudo apt update
sudo apt install build-essential
sudo apt install cmake
sudo apt install libcairo2-dev
sudo apt install libgraphicsmagick1-dev
sudo apt install libpng-dev

// Clone the I02D Repo. I cloned it in my project directory.
//With the following steps, you should be able to put it anywhere on your system and the last step should make it findable by Cmake.
git clone --recurse-submodules https://github.com/cpp-io2d/P0267_RefImpl
cd P0267_RefImpl
mkdir Debug
cd Debug
cmake --config Debug "-DCMAKE_BUILD_TYPE=Debug" ..
cmake --build .
// This will allow Cmake to find it.
sudo make install



// from within your project root directory (not necessarily workspace root)
mkdir build && cd build

// from within build
cmake ..
make

// Run the executable
./OSM_A_star_search

// Test your code with the supplied unit tests
./test

// I've added "target_compile_options(OSM_A_star_search PUBLIC -g)" to CmakeList.txt so the code will compile with the debug symbols.