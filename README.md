# 环境安装
## 操作系统
Ubuntu 22.04
## Python配置
```
pip install osmnx==1.9.4
pip install pygame==2.6.1
# 或者不需要管版本，直接安装就行，这两版本之间差异应该不大
```
<!-- 
这是使用sumo的方式，在第六周暂时弃用

## SUMO
```sudo apt install sumo```
安装完成后，输入sumo应该输出
```
(airfogsim) squirtle@TX:~/Desktop/Squirtle/ProjectInMajor/drone-scheduling$ sumo
Eclipse SUMO sumo Version 1.12.0
 Build features: Linux-4.15.0-167-generic x86_64 GNU 11.2.0 None Proj GUI SWIG GDAL FFmpeg OSG GL2PS Eigen
 Copyright (C) 2001-2022 German Aerospace Center (DLR) and others; https://sumo.dlr.de
 License EPL-2.0: Eclipse Public License Version 2 <https://eclipse.org/legal/epl-v20.html>
 Use --help to get the list of options.
```
## Python 
没仔细测试，只写几个主要的包的版本：
```
pygame==2.6.1
traci==1.20.0
``` -->