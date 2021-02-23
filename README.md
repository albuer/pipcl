# Pi-PCL
[中文](./README_cn.md)
pipcl is a point cloud processing tool based on PCL

## Description
pipcl is a PCL-based point cloud processing command line tool, which connects multiple point cloud processing operations through a pipeline to complete complex point cloud processing

## Dependence
PCL needs to be installed in the system
```
sudo apt install libpcl-dev
```
You can also compile and install from [PCL source code](https://github.com/PointCloudLibrary/pcl)

## Compile
Use cmake to compile
* Compile the Debug version
```
mkdir Debug
cd Debug
cmake ..
make
```

* Compile the Release version
```
mkdir Release
cd Release
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

Output `pipcl` after successful compilation

## Instructions
In pipcl, you can use the symbols `:`/`:+`/`:-`/`:=` to connect multiple point cloud operations, and use the point cloud output from the previous operation as the next operation Input, where:
* `:`
Transfer point cloud
* `:+`
Transfer master point cloud
* `:-`
Transfer secondary point cloud
* `:=`
Mix and transfer point clouds

### View the help information of pipcl
```
./pipcl help

usage: ./pipcl [help ActionClass] ActionClass=Action [ARGS]: ActionClass=Action [ARGS] ...

All action class are:
  source
  sink
  filter
  segmentation
  search
  feature
  registration
  misc

See'./pipcl help ActionClass' for more information on a specific action.
```

### Some examples

* File format conversion
```
./pipcl source=input.pcd: sink=output.ply
```

* Display point cloud
```
./pipcl source=input.pcd: sink=VIS
```

* Generate point clouds of geometric shapes
```
./pipcl source=line point1=0,0,0 point2=1,1,1: sink=VIS
./pipcl source=cube xrange=-1,1 yrange=-2,2 zrange=-3,3 density=2: sink=output.pcd
```

* Filter
```
./pipcl source=input.pcd: filter=voxelgrid leaf=0.01: sink=output.pcd
```

* The input multiple point clouds are mixed and output to a file
```
./pipcl source=in-1.pcd,in-2.pcd,in-3.pcd := sink=output.pcd ascii=1
```

* The input point cloud is divided after downsampling, and the divided point cloud is saved to out1.pcd, and the point cloud discarded after the division is saved to out2.pcd after kdtree search
```
./pipcl source=input.pcd: filter=VoxelGrid leaf=0.01: segmentation=SAC model=plane threshold=0.01 iterations=1000 :+ sink=out1.pcd :- search=kdtree: sink=out2.pcd
```

* Use ICP to register multiple input point clouds
```
./pipcl source=in-1.pcd,in-2.pcd,in-3.pcd,in-4.pcd: registration=icp dist=0.05 rans=0.05: sink=out.pcd
```

* Use passthrough filtering for in-1.pcd, kdtree search for in-2.pcd, and finally mix in-3.pcd with the two processed point clouds before output to a file
```
./pipcl source=in-1.pcd: filter=passthrough: source=in-2.pcd: search=kdtree: source=in-3.pcd := sink=out.pcd
```
