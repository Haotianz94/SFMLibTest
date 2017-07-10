# 外部依赖库

## OpenCV
用于图像I/O

## Boost
在Scene3DRecover.cpp中用于读取文件夹下的所有文件
注意：代码中使用的boost版本比较低，使用其它版本的boost时可能会出现语法不兼容的问题

## ceres-windows-master
注意：请使用主工程用的VS重新编译生成相关链接文件，避免版本不兼容的问题


# 项目结构

## 项目入口
SFMLibTest.cpp

## 类关系
StitchSolver(顶层类)
--|Scene3DReocver(前景重建相关)
----|BundlerFileLoader(读取Bunlder重建结果)
----|CameraModel(定义相机模型)
--|ViewGenerator(利用新视角模型生成控制点位置)
--|Warper(基于ASAP的网格变形)



# 运行方法

1. 所有视频帧放置在\origin下
2. 利用SFMWindows提取视频帧的所有特征点，sift文件在\origin下
3. 利用Adobe AE获得视频帧的所有前景mask，保存在\mask下
4. 运行StitchSolver::prepareForBundler()，这个函数读取mask和sift，将前景和背景sift分别存在\fg
和\bg下
5. 利用SFMWindows读取\bg下的所有视频帧，然后运行SfM->Pairwise Matching->compute missing match匹配所有sift，然后相继运行SfM->Reconstruct Sparse和Reconstruct Dense来完成背景重建
6. 运行StitchSolver::loadReconstruction()读取重建结果并重建前景
7. 运行StitchSolver::warpFGOnMesh()来得到变形的序列

注意：为了节省调参的编译时间，本项目中常用的配置参数都写在了config.ini中，包括一些输入输出路径和算法参数。程序将在开始运行时读取config中的所有参数值，请参考文件中的注释来理解每个参数的意义。