cd Compile
rm CMakeCache.txt
rm -r CMakeFiles
rm cmake_install.cmake
make clean # This may fail if no make has been called yet, but that is fine.

cmake .
make
cd ..
