# Robocup Junior Soccer Open Code

[![GitHub watchers](https://img.shields.io/github/watchers/senrobo/open-code.svg?style=social)](https://github.com/senrobo/open-code/watchers)
[![GitHub stars](https://img.shields.io/github/stars/senrobo/open-code.svg?style=social)](https://github.com/senrobo/open-code/stargazers)

## Directory Structure

``` bash
├──  .vscode
│  ├──  c_cpp_properties.json
│  ├──  extensions.json
│  ├──  launch.json
│  └──  settings.json
├──  include
│  ├──  shared.h
│  ├──  util.h
│  └──  vector.h
├──  src
│  ├──  l2-tnsy
│  │  ├──  basic
│  │  │  ├──  movement.cpp
│  │  │  └──  pid.cpp
│  │  ├──  include
│  │  │  ├──  main.h
│  │  │  ├──  movement.h
│  │  │  └──  pid.h
│  │  ├──  config.h
│  │  ├──  main.cpp
│  │  └──  subroutines.cpp
│  ├──  l3-lidar
│  │  └──  main.cpp
│  ├──  l3-tnsy
│  │  ├──  basic
│  │  │  ├──  ballposition.cpp
│  │  │  ├──  kalman.cpp
│  │  │  └──  sensorfusion.cpp
│  │  ├──  include
│  │  │  ├──  ballposition.h
│  │  │  ├──  kalman.h
│  │  │  ├──  main.h
│  │  │  └──  sensorfusion.h
│  │  ├──  config.h
│  │  ├──  main.cpp
│  │  └──  subroutines.cpp
│  ├──  util.cpp
│  └──  vector.cpp
├──  .clang-format
├──  .DS_Store
├──  .gitignore
├──  platformio.ini
└──  README.md
```

Please refer to the specific module directories and files for more details on their functionality and implementation.
