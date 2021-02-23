# Pi-PCL
[English](./README.md)

pipcl是一个基于PCL的点云处理工具

## 描述
pipcl是一个基于PCL的点云处理命令行工具，通过管道的方式把多个点云处理操作连接在一起，从而完成复杂的点云处理

## 依赖
需要系统中安装有PCL
```
sudo apt install libpcl-dev
```
你也可以从[PCL源码](https://github.com/PointCloudLibrary/pcl)进行编译安装

## 编译
使用cmake进行编译
* 编译Debug版本
```
mkdir Debug
cd Debug
cmake ..
make
```

* 编译Release版本
```
mkdir Release
cd Release
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

编译成功后输出`pipcl`

## 使用方法
在pipcl中，可以用`:`/`:+`/`:-`/`:=`这几个符号把多个点云操作连接起来，把上一个操作输出的点云做为下一个操作的输入，其中：
* `:`
传递点云
* `:+`
传递主点云
* `:-`
传递次点云
* `:=`
将点云混合再传递

### 查看帮助信息
```
./pipcl help

usage: ./pipcl [help ActionClass] ActionClass=Action [ARGS] : ActionClass=Action [ARGS] ...

All action class are:
  source
  sink
  filter
  segmentation
  search
  feature
  registration
  misc

See './pipcl help ActionClass' for more information on a specific action.
```

### 一些例子

* 文件格式转换
```
./pipcl source=input.pcd : sink=output.ply
```

* 显示点云
```
./pipcl source=input.pcd : sink=VIS
```

* 生成几何形状的点云
```
./pipcl source=line point1=0,0,0 point2=1,1,1 : sink=VIS
./pipcl source=cube xrange=-1,1 yrange=-2,2 zrange=-3,3 density=2 : sink=output.pcd
```

* 过滤
```
./pipcl source=input.pcd : filter=voxelgrid leaf=0.01 : sink=output.pcd
```

* 把输入的多个点云混合后输出到文件
```
./pipcl source=in-1.pcd,in-2.pcd,in-3.pcd := sink=output.pcd ascii=1
```

* 输入点云经过降采样后进行分割，把分割得到的点云保存到out1.pcd，而把分割后舍弃的点云经过kdtree search后保存到out2.pcd
```
./pipcl source=input.pcd : filter=VoxelGrid leaf=0.01 : segmentation=SAC model=plane threshold=0.01 iterations=1000 :+ sink=out1.pcd :- search=kdtree : sink=out2.pcd
```

* 把输入的多幅点云使用ICP进行配准
```
./pipcl source=in-1.pcd,in-2.pcd,in-3.pcd,in-4.pcd : registration=icp dist=0.05 rans=0.05 : sink=out.pcd
```

* 对in-1.pcd使用passthrough滤波，对in-2.pcd使用kdtree search，最后把in-3.pcd与前面两个经过处理后的点云混合，再输出到文件
```
./pipcl source=in-1.pcd : filter=passthrough : source=in-2.pcd : search=kdtree : source=in-3.pcd := sink=out.pcd
```

