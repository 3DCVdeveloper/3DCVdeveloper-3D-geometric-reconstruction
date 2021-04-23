# Dependencies:

1.

```bsh
sudo apt-get install build-essential freeglut3 freeglut3-dev 
```

2.OpenNI2 SDK

开发版是Arm64 (可通过uname -a查看)

https://developer.orbbec.com.cn/download.html?id=64

编译程序需要其中的Include和Redist，后者是链接库（不能用系统里之前装的OpenNI2的替代）。

3.OpenCV

4.PCL

5.[cpu_tsdf](https://github.com/sdmiller/cpu_tsdf)



# Build Instructions

```bsh
sh build.sh
```



# Usage

连接Orbbec Astra Pro采集数据，保存TUM-RGBD格式数据集

```bash
sh run_cam.sh
```

读取ORB_SLAM2作为基础框架获取的时间戳、关键帧位姿、RGB、深度图作为关键信息对，进行扩展的稠密三维重建

```bash
sh run_reconstruction.sh
```



# Reference

[3D相机D2C对齐](https://developer.orbbec.com.cn/forum_plate_module_details.html?id=185) 

[Ubuntu下Astro Pro配置openni踩坑小记](https://developer.orbbec.com.cn/forum_plate_module_details.html?id=677) 