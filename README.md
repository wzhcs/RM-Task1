# RM-Task1
  ### 项目运行方式
 - 通过命令行运行以下命令：
```shell
sudo socat -d -d PTY,link=/dev/ttyV0,raw,echo=0 PTY,link=/dev/ttyV1,raw,echo=0
```
以此来创建虚拟串口，例如：本项目运行时虚拟端口号及波特率分别为**dev/pts/3,dev/pts/4**和**38400**，分别对应**publisher**和**subscriber**节点。

 - 节点运行方式:
```shell
cd RMM（如果有修改端口号需要重新进行colcon build）
source install/setup.bash
ros2 launch vision a.launch.py
```

 - 通过yaml文件修改消息发送内容：仅需修改yaml文件中的**send_message** 参数即可
