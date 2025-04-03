## Preparation
Need to install opencv.
```
sudo apt install libopencv-dev
```

## Command
Compile
```
g++ main.cpp -o main `pkg-config --cflags --libs opencv4`
```
Execute
```
./main
```
