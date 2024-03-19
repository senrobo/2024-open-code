# open-code

This repository contains the source code for the "open-code" project. The project is organized into different directories, each representing a specific module or component.

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
