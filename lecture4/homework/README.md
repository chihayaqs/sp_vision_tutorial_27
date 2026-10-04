# nav_lecture4小作业：Qos_debugger
这道题的目标就是让你快速上手、理解什么是ros。抛开复杂的概念，ros本质上完成的任务就是便利的进程间通信。比如，我有两个进程，一个进程发布雷达数据，另一个进程接收。使用ros就可以方便的完成通讯。你可以搜索以下，发送信息有哪些类型，分别有何特点。特别注意，不同的信息传输方式有不同的质量要求。你不会允许送的外卖没到你手上，但是一个电话过来，也许漏接了也无所谓，可能只是个诈骗。ros2也是这样。重点关注这一点会对这道题有所帮助

> 环境要求：ROS2 Humble

## 包结构

```
src/
  nav_hw_interfaces/     # 接口包：只放 .msg，无业务代码
    msg/SensorData.msg   # 可以打开.msg文件查看接口详细内容
  qos_debugger/          # 业务节点包
    src/qos_debugger_pub.cpp   # 发布 /SensorData
    src/qos_debugger_sub.cpp   # 订阅 /SensorData
```

## 编译

```bash
cd lecture4/homework
colcon build 
source install/setup.bash
```


## 任务一：实现pub和sub的通信

```bash
ros2 -h     //有忘记的命令就输入-h去查询用法
```

**现象**：启动pub和sub节点后sub节点订阅不到任何消息
提示：如果两个节点不能通过话题通信，我们应该如何区查看话题的详细信息（有没有相关的命令）
任务一仅修复qos_debugger_pub.cpp的一处或几处代码即可完成


---

## 任务二：为什么收到的消息会丢包？/(ㄒoㄒ)/~~

第一问找到问题并修改代码后，记得重新
```colcon build```
```source install/setup.bash```
**现象**：sub会打印黄色的warning输出告诉你丢包的序列，每秒还会打印出丢包率

提示：
有没有什么命令可以查看节点的配置(ros2 param -h)
可以通过修复qos_debugger_sub.cpp中的一处或几处代码解决该问题（可能会有多种解决方法）

## 任务三：把收到的消息的帧率计算并打印出来（放在定时器回调函数中每秒打印一次即可）
补全qos_debugger_sub.cpp即可



在下面按顺序完成三个任务，要求把用到的命令放入代码块中并讲解命令，每一问最好加入自己的理解

## 任务一作答：实现pub和sub的通信

每个新终端先加载 ROS 环境，并在 `lecture4/homework` 目录下加载编译结果：

```bash
cd lecture4/homework
source install/setup.bash
```

分别启动发布者和订阅者，再查看话题信息：
```bash
# 终端1
ros2 run qos_debugger qos_debugger_pub

# 终端2
ros2 run qos_debugger qos_debugger_sub

# 终端3
ros2 node list
ros2 topic list
ros2 topic info /sensor_data -v
```

使用`node list` 查看节点，`topic list` 查看话题，
`topic info -v` 查看话题的消息类型、发布者、订阅者及其QoS配置。

原实现中，发布者和订阅者使用的都是 `/sensor_data` 话题，消息类型也相同。
但发布端的 Reliability 是 `BEST_EFFORT`，订阅端是 `RELIABLE`。
按照兼容性，发布端提供的可靠性需要满足订阅端的要求

因此，将 `qos_debugger_pub.cpp` 中 `reliability` 参数的默认值改为 `"reliable"`。
重新编译并重启节点后，两端都使用 Reliable，订阅者就能收到消息。

```bash
colcon build
source install/setup.bash
```

## 任务二作答：解决丢包

通过以下命令查看节点的参数：
```bash
ros2 param list /sensor_subscriber
ros2 param get /sensor_publisher rate
ros2 param get /sensor_subscriber callback_delay_ms
ros2 param get /sensor_subscriber depth
```

使用`param list` 列出节点的参数，`param get` 读取参数值。
原实现中，发布频率是100Hz，订阅回调延迟是30ms，队列深度是10。
100Hz相当于每10ms发布一条消息，而回调每处理一条消息就休眠30ms，
每秒最多处理约 `1000/30 = 33.3` 条。而`rclcpp::spin(node)`
使用单线程执行器，消息因此不断积压。`KeepLast(10)` 只保存最新的 10 条，旧消息被覆盖后，订阅端就会发现序号跳跃。

因此，将 `qos_debugger_sub.cpp` 中 `callback_delay_ms` 的默认值改为0，去掉等待让回调跟上发布速度。
同时，也可以用`param set` 修改运行中节点的参数，重新观察延迟的影响：
```bash
# 设为30ms
ros2 param set /sensor_subscriber callback_delay_ms 30
# 设为0ms
ros2 param set /sensor_subscriber callback_delay_ms 0
```

此外，增大队列深度可以缓解短时间积压，但消息产生速度一直高于处理速度时，队列仍然会满。因此这里的问题应该在于回调耗时，队列深度10可以保持不变。

原实现里sub打印黄色的warning输出丢包的序列，但累积统计好像没有累加丢失数量，导致每秒打印出的丢失条数和丢包率始终为0，
在计算 `lost` 后补上 `lost_count_ += lost`，每秒输出的累计丢包率就能正确反映结果了。

## 任务三作答：计算并打印接收帧率

在每秒执行的 `report()` 中，使用题目给出的 `last_received_count_` 和`last_report_time_` 记录上一次统计的数据：
本周期接收数量 = 当前累计接收数量 - 上次累计接收数量
本周期时长 = 当前时间 - 上次统计时间
接收帧率 = 本周期接收数量 / 本周期时长

因此，用 `std::chrono::steady_clock` 记录时间，`std::chrono::duration<double>`
将时间差转换为带小数的秒数。计算并打印帧率后，更新上次接收数量和上次统计时间。
定时器虽然设置为1秒，但回调繁忙时执行会推迟，所以这里用实际时间差计算。
修改后重新编译并重启节点，
终端能看到打印类似：
接收帧率: 100.00 Hz (最近 1.000 s 收到 100 条)