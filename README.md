# Robocup Junior Soccer Open Code

[![GitHub watchers](https://img.shields.io/github/watchers/senrobo/open-code.svg?style=social)](https://github.com/senrobo/open-code/watchers)
[![GitHub stars](https://img.shields.io/github/stars/senrobo/open-code.svg?style=social)](https://github.com/senrobo/open-code/stargazers)

## Directory Structure

``` bash
├──  include
│  ├── 󰂺 README
│  ├──  shared.h
│  ├──  util.h
│  └──  vector.h
├──  src
│  ├──  l1-light
│  │  └──  main.cpp
│  ├──  l2-motor
│  │  └──  main.cpp
│  ├──  l3-lidar
│  │  └──  main.cpp
│  ├──  l3-master
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
│  ├──  util.cpp
│  └──  vector.cpp
├──  .clang-format
├──  .gitignore
├──  platformio.ini
└──  README.md
```

Please refer to the specific module directories and files for more details on their functionality and implementation.
