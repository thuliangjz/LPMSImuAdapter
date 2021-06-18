本项目基于[OpenZen](https://lpresearch.bitbucket.io/openzen/latest/)为[LPMS-B2](https://lp-research.com/9-axis-bluetooth-imu-lpmsb2-series/)传感器提供可供Matlab使用的高性能c++接口。支持在Matlab中实时读取数据，为行走模型的算法验证提供了支持。

+ 用visual studio 2019打开`LPMSAdapter/LPMSAdapter.sln`，构建模式为Release, x64，生成解决方案。

+ 在根目录下打开`matlab`，运行命令

  ~~~matlab
  adapter_lib_gen('gen')
  ~~~

  可能会提示有文件不存在的警告，不用管

  成功之后继续运行

  ~~~matlab
  adapter_lib_gen('build')
  ~~~

+ 测试是否能够连接上IMU。运行

  ~~~matlab
  visualize_imu
  ~~~

  按ctrl+c中断脚本执行，输入`clear`即可断开和IMU的连接

+ 以后每次进入文件夹时运行命令：

  ~~~matlab
  adapter_lib_gen('add_lib')
  ~~~

  来将dll文件夹加入到matlab的运行时PATH变量列表中。

