### Linux 使用 DYNAMIXEL MX-64

#### 1.使用DYNAMIXEL SDK提供的example驱动电机进行测试

​	注:下文假设使用64bit linux系统,若使用其他系统,则调整文件夹进行测试

- [DynamixelSDK-代码仓库](https://github.com/ROBOTIS-GIT/DynamixelSDK)获取源码

- 按照[官方提供的教程](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#cpp-linux)进行example编译

  官方教程分为两步：

  1. 在**[DynamixelSDK folder]/c++/build/linux64**下编译动态库
  
  2. 在**[DynamixelSDK folder]/c++/example/protocol1.0/read_write/linux64**中编译可执行文件
  
     ​	(测试发现,protocol2.0的read_write并不能正常驱动电机,**进一步了解后发现,不同型号Dynamixel MX-64使用不同版本protocol**)
  
- 连接MX-64

  设置串口权限

  ```shell
  sudo chmod a+rw /dev/ttyUSB0
  ```

  运行可执行文件

  ```shell
  ./read_write
  ```

  若与舵机连接成功,则输出

  ```
  Succeeded to open the port!
  Succeeded to change the baudrate!
  Dynamixel has been successfully connected 
  Press any key to continue! (or press ESC to quit!)
  ```

  若输出**[TxRxResult] There is no status packet!**,则需检查MX-64的连接状态

- 转动电机

  若连接成功后,按下任意键即可是舵机转动,且屏幕会输出目标位置与当前位置:

  ```shell
  [ID:001] GoalPos:100  PresPos:1579
  ```

  其中ID代表舵机的ID,若同时存在多个舵机,则需要进行进一步操作



#### 2.Dynamixel Protocol 协议

​	protocol1.0和protocol2.0 在电机的控制模式上存在区别:

​	从官方的**read_write.cpp**中查看差异:

protocol1.0

```
// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_SPEED                   32
#define ADDR_MX_PRESENT_POSITION        36
```

protocol2.0

```
// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611
```

​	不同的Dynamixel model会有不同的control table address

​	电机的协议版本与电机的型号相关,具体的协议版本可以通过win版本的SDK进行查看.



#### 3.在Cpp中调用相关SDK进行编程

​	根据DynamicSDK,编译好libdxl_x64_cpp.so之后,可以在自己的cpp文件中进行简单操作:

[以下代码用于Protocol1.0相关协议的电机]

​	1.读取当前Position

```C++
packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
```

​	2.设置目标Position

```c++
packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
```

​	3.设置目标Position以及Speed

```c++
packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_SPEED, speed, &dxl_error);
```

​	相关代码

