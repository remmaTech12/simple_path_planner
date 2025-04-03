g++ main.cpp -o main `pkg-config --cflags --libs opencv4` -I/usr/include/ompl-1.5 -I/usr/include/eigen3 -lompl
./main
