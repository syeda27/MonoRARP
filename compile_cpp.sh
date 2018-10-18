cd Compile
make clean # This may fail if no make has been called yet, but that is fine.
rm CMakeCache.txt
rm -r CMakeFiles
rm cmake_install.cmake

cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 .
make
cd ..
