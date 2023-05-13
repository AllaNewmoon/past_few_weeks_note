# ROS学习
## 常用组件
### TF坐标变换
**坐标msg消息**：
1. geometry_msgs/PointStamped包含std_msgs/Header消息和geometry_msgs/Point消息。point中有x,y,z坐标
2. geometry_msg/TransformStamped包含Header和geometry_msgs/Transform
```
transform里包含:
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
```
分别用三维向量和四元数表示平移，旋转

**静态坐标变换**：两个坐标系相对位置固定
C++实现发布：
```
#include "ros/ros.h"
//创建静态发布方需包含
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
//四元数有关头文件
#include "tf2/LinearMath/Quaternion.h"
using namespace ros;
int main(int argc, char* argv[]){
    setlocale(LC_ALL,"");
    init(argc,argv,"static_pub");
    NodeHandle nh;
    //创建发布方
    tf2_ros::StaticTransformBroadcaster pub;
    //创建转换的矩阵
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = Time::now();
    tfs.header.frame_id = "base_link";
    tfs.child_frame_id = "laser";
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0;
    tfs.transform.translation.z = 0.5;
    //获取当前位置的四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    //发布消息
    pub.sendTransform(tfs);
    spin();
    return 0;
}
```
C++实现订阅：
```
#include "ros/ros.h"
//接收者的头文件
#include "tf2_ros/transform_listener.h"
//Buffer缓存头文件
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
using namespace ros;
int main(int argc, char* argv[]){
    setlocale(LC_ALL,"");
    init(argc, argv, "static_sub");
    NodeHandle nh;
    //创建缓存
    tf2_ros::Buffer buffer;
    //创建监听对象
    tf2_ros::TransformListener listener(buffer);
    //创建坐标点数据
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    Rate rate(10);
    Duration(2).sleep();
    //算法
    while(ok()){
        //创建转换后的坐标点
        geometry_msgs::PointStamped ps_out;
        //利用buffer.transform转换
        ps_out = buffer.transform(ps,"base_link");
        ROS_INFO("转换后的坐标值:(%.2f, %.2f. %.2f)", ps_out.point.x, ps_out.point.y, ps_out.point.z);
        rate.sleep();
        spinOnce();
    }
    return 0;
}
```
Python实现发布：
```
#! /usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
if __name__ == "__main__":
    rospy.init_node("static_pub_p")
    //创建发布者对象
    pub = tf2_ros.StaticTransformBroadcaster()
    //创建转换矩阵
    ts = TransformStamped()
    ts.header.stamp = rospy.Time.now()
    ts.header.frame_id = "base_link"
    ts.child_frame_id = "laser"
    ts.transform.translation.x = 0.2
    ts.transform.translation.y = 0.0
    ts.transform.translation.z = 0.5
    //从euler角得到四元数
    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    ts.transform.rotation.x = qtn[0]
    ts.transform.rotation.y = qtn[1]
    ts.transform.rotation.z = qtn[2]
    ts.transform.rotation.w = qtn[3]
    //发布信息
    pub.sendTransform(ts)
    rospy.spin()
    pass
```
Python实现订阅：
```
#! /usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs

if __name__ == "__main__":
    rospy.init_node("static_sub_p")
    //创建buffer对象
    buffer = tf2_ros.Buffer()
    sub = tf2_ros.TransformListener(buffer)
    ps = tf2_geometry_msgs.PointStamped
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "laser"
    ps.point.x = 2.0
    ps.point.y = 3.0
    ps.point.z = 5.0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    //利用buffer中的transform进行变换
        ps_out = buffer.transform(ps,"base_link")
        rate.sleep()
        rospy.loginfo("转换后的坐标为:(%.2f, %.2f. %.2f)", ps_out.point.x, ps_out.point.y, ps_out.point.z)  
    pass
```
**补充**：也可以使用封装好的ros命令实现静态坐标系相对信息发布
```
rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 /baselink /laser
```

**动态坐标变换**：
流程：
1. 包含头文件
2. 初始化，设置句柄
3. 创建订阅对象，订阅turtle1/pose
4. 利用回调函数将位姿信息转换成坐标并发布
5. spin

C++发布方实现(以Turtlesim为例)：
```
//回调函数：
void doPose(const turtlesim::Pose::ConstPtr& pose){
    //创建发布者
    static tf2_ros::TransformBroadcaster pub;
    //创建变换矩阵
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = "world";
    ts.header.stamp = Time::now();
    ts.child_frame_id = "turtle1";
    //将订阅到的pose赋值给ts
    ts.transform.translation.x = pose->x;
    ts.transform.translation.y = pose->y;
    ts.transform.translation.z = 0;
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    ts.transform.rotation.w = qtn.getW();
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    //发布ts
    pub.sendTransform(ts);
}
//订阅pose信息
Subscriber sub = nh.subscribe("/turtle1/pose",100,doPose);
```

C++订阅方实现：
注意时间戳设置为：
```
ps.header.stamp = ros::Time(0.0);
```
其余和静态类似

Python发布方实现：
```
//回调函数
def doPose(pose):
    pub = tf2_ros.TransformBroadcaster()
    ts = TransformStamped()
    ts.header.frame_id = "world"
    ts.header.stamp = rospy.Time.now()
    ts.child_frame_id = "turtle1"
    ts.transform.translation.x = pose.x
    ts.transform.translation.y = pose.y
    ts.transform.translation.z = 0
    qtn = tf.transformations.quaternion_from_euler(0,0,pose.theta)
    ts.transform.rotation.x = qtn[0]
    ts.transform.rotation.y = qtn[1]
    ts.transform.rotation.z = qtn[2]
    ts.transform.rotation.w = qtn[3]
    pub.sendTransform(ts)
    pass
```

**多坐标变换**：
流程：
1. 发布son1和son2相对world的坐标信息
2. 订阅消息，取出信息，借助tf2实现坐标转换

1.发布信息：
```
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="son1" args="5 1 2 0.9 0.8 0.7 /world /son1" output="screen"/>
    <node pkg="tf2_ros" type="static_transform_publisher" name="son2" args="4 1 3 1.1 0.9 0.7 /world /son2" output="screen"/>
</launch>
```
2.获取两坐标系相对关系：
```
//第一个参数是目标坐标系，参数2是原坐标系
geometry_msgs::TransformStamped son1Toson2 = buffer.lookupTransform("son2", "son1", Time(0));
tfs = buffer.lookup_transform("son2","son1",rospy.Time(0))
```

**其他**:
1.查看坐标系关系
```
rosrun tf2_tools view_frames.py
```

**实操流程**：
1. 通过参数服务器等生成坐标
2. 发布坐标信息（对于实现逻辑相同，只是话题名称有差异的，可以将差异通过参数args传入）
3. 订阅坐标信息并处理，再发布生成的速度等信息
4. 创建launch文件，包含启动节点，坐标发布节点，坐标处理节点

### ROSbag
进行数据的留存及读取实现
**步骤：**
1. 创建目录保存录制文件
2. 开始录制：
```
rosbag record -a -o bags/hello.bag
```
3. bag信息查询:
```
rosbag info bags/bag文件名
```
4. 回放：
```
rosbag play bags/文件名
```

**编码实现**：
C++写文件:
1.导包
2.初始化
3.创建rosbag对象
4.打开文件流
5.写数据
6.关闭文件流
```
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
using namespace ros;

int main(int argc, char *argv[]){
    setlocale(LC_ALL, "");
    init(argc, argv, "bag_write");
    NodeHandle nh;
    rosbag::Bag bag;
    std_msgs::String msg;
    msg.data = "hello xxxx";
    //以写的方式打开
    bag.open("hello.bag", rosbag::BagMode::Write);
    //传入话题，时间戳，msg
    bag.write("/chatter",Time::now(), msg);
    bag.write("/chatter",Time::now(), msg);
    bag.write("/chatter",Time::now(), msg);
    bag.close();
    return 0;
}
```
C++读文件：先获取消息的集合，再迭代出字段
```
#include "rosbag/view.h"
for(auto &&m : rosbag::View(bag)){
    //进行解析
    std::String topic = m.getTopic();
    Time time = m.getTime();
    std_msgs::StringPtr p = m.instantiate<std_msgs::String>();
}
```
Python写文件：
```
import rosbag
msg = String()
msg.data = "hello"
//write模式，不需要open
bag = rosbag.Bag("hello_p.bag", "w")
bag.write("/liaoTian",msg)
bag.close()
```
Python读文件：
```
bag = rosbag.Bag("hello_p.bag", "r")
msgs = bag.read_message("/liaoTIan")
for topic,msg,time in msgs:
    rospy.loginfo("话题:%s,消息:%s,时间:%s",topic,msg.data,time)
bag.close()
```

### rqt工具箱
调用工具时以图形化操作代替命令操作

1.rqt_plot:绘制发布在topic上的数据
2.rqt_console:显示日志信息
3.rqt_bag:实现录制与播放（红色按钮选择话题开始录制）