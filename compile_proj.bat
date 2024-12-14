cmake -G Ninja -B build
cmake --build ./build --config release
cd build/test
kalmanFilter-bench.exe
cd ../..
pause
