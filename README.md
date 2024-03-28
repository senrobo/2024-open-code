# Robocup Junior Soccer Open Code

[![GitHub watchers](https://img.shields.io/github/watchers/senrobo/open-code.svg?style=social)](https://github.com/senrobo/open-code/watchers)
[![GitHub stars](https://img.shields.io/github/stars/senrobo/open-code.svg?style=social)](https://github.com/senrobo/open-code/stargazers)

## Directory Structure

``` bash
├──  include
│  ├──  shared.h
│  ├──  util.h
│  └──  vector.h
├──  src
│  ├──  l3-lidar
│  │  └──  main.cpp
│  ├──  Main-Teensy
│  │  ├──  basic
│  │  │  ├──  ballposition.cpp
│  │  │  ├──  kalman.cpp
│  │  │  ├──  movement.cpp
│  │  │  ├──  pid.cpp
│  │  │  └──  sensorfusion.cpp
│  │  ├──  include
│  │  │  ├──  ballposition.h
│  │  │  ├──  kalman.h
│  │  │  ├──  main.h
│  │  │  ├──  movement.h
│  │  │  ├──  pid.h
│  │  │  └──  sensorfusion.h
│  │  ├──  config.h
│  │  ├──  main.cpp
│  │  └──  subroutines.cpp
│  ├──  Teensy-L3
│  │  └──  main.cpp
│  ├──  util.cpp
│  └──  vector.cpp
├──  .clang-format
├──  .gitignore
├──  platformio.ini
└──  README.md
```
