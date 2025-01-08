<a id="readme-top"></a>
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/CQU-UISC/px4ctrl_client">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>
  <h3 align="center">UISC Lab Px4Ctrl Client</h3>
  <p align="center">
    Px4Ctrl地面端
  </p>
  <img align="center" src=https://img.shields.io/badge/license-GPL--3.0-blue  alt="license"/>
  
</div>

<!-- ABOUT -->
## About
一个使用ZMQ与ImGUI构建的无人机控制地面站，支持：
- 无人机的基本控制: 启动、关闭、悬停
- 无人机悬停set point的设置
- 无人机控制器切换
- 无人机集群控制

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites
- [spdlog](https://github.com/gabime/spdlog) >=v1.14.1 
- [GLFW](https://github.com/glfw/glfw)  == 3.4 
- OpenGL >= 3.3
- [format](https://github.com/fmtlib/fmt) 
- [C++ 20](https://en.cppreference.com/w/cpp/compiler_support)

### Installation
```
git clone https://github.com/CQU-UISC/px4ctrl_client.git
cd px4ctrl_client
git submodule update --init --recursive
mkdir build && build
cmake ..
make -j4
```

<!-- USAGE EXAMPLES -->
## Usage
./px4client -c ../config/zmq.yaml

<!-- ROADMAP -->
## Roadmap

- [ ] 添加轨迹的渲染
- [ ] 添加推力映射的显示


<!-- CONTACT -->
## Contact
Xu Lu - lux@cqu.edu.cn

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
* [ZJU FastLab](https://github.com/ZJU-FAST-Lab)
* [UZH Robotics and Perception Group](https://github.com/uzh-rpg)
