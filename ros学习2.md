# ROS学习
## API进阶：
**传递参数**：
```
rosrun plumbing_apis demo01_apis_pub _length:=2
```
传递了一个名为length值为2的参数到参数服务器

**init进阶**：
```
init(argc, argv, name, options)
```
其中options可填入init_options::AnonymousName，可以保证多次启动同节点且不重名。

Python:
```
rospy.init_node(name, argv, anonymous=false)
```

**发布对象进阶**：
```
NodeHandle nh;
Publisher pub = nh.advertise<std_msgs>
("内容", queue_size, latch)
```
如果latch为true，可以保存发布的最后一个消息，并发送给订阅方。

Python:
```
pub = rospy.Publisher(name, String, queue_size, latch=true)
```
**时刻**：
```
Time right_now = Time::now();//将当前时刻封装并返回
//参考系：1970年01月01日 00：00：00
right_now.toSec(); / right_now.sec;//转换为秒数

Time t1(sec, nsec)//设置指定时间

Duration du(4.5)//设置持续时间4.5s
du.sleep();
```
时刻和持续时间之间可以加减，时刻之间只能相加

Python:
```
time = rospy.Time.now()
time.to_sec() / time.secs

time1 = rospy.Time(100, 12314)

du = rospy.Duration(5, 0)
rospy.sleep(du)
```
**定时器**:
```
Timer timer = nh.createTimer(Duration period, 
    TimerCallBack
    bool oneshot=false,//一次性
    bool autostart=true//自动启动)

timer.start();//手动启动

void cb(const TimerEvent& event){
    ROS_INFO("函数调用时间:%.2f", event.current_real.toSec());
}//回调函数
```

Python:
```
timer = rospy.Timer(rospy.Duration(), 
    CallBack,
    bool oneshot=false,
    bool reset=false)

def doMsg(event):
    rospy.loginfo("调用时刻为：%.2f", event.current_real.to_sec())
```

## 自定义头文件
1.在头文件中声明namespace，class和method

2.在c_cpp_properties中添加include的路径

3.在源文件中编写method

4.配置库：在CMakeList中配置include_directories，add_library(写头文件和源文件路径)，add_dependencies和target_link_libraries(节点名字要一样)

5.配置可执行文件(target_link_libraries中要额外传入参数)

## Python模块导入
1.编写.py模块

2.设置环境变量:
```
sys.path.insert(0,"绝对路径")

import os
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/plumbing_pub_sub/scripts")
//动态获取路径
```

## 元功能包
将多个实现某个模块的功能包打包成一个功能包

**实现**：
1. 新建功能包(依赖项先不写)
2. 修改package.xml
```
<exec_depend>包名</exec_depend>

export下声明:
<metapackage />
```
3. 修改CMakeList
4. 保留前三行，第四行添加
```
catkin_metapackage()
```

## launch文件
**node标签**：包含pkg，type，name，args(传递参数给节点)，machine，respawn(true则节点退出后自动重启)，respawn_delay，required(true则该节点退出时杀死整个roslaunch)，ns，output

**include标签**：将另一个xml格式的launch文件导入当前文件。
```
<include file="$(find launch01_basic)/launch/start_turtle.launch">
```

**remap标签**：修改话题名称
```
<node>
    <remap from="原话题名" to="新话题名" />
</node>
```

**param标签**：用于在参数服务器上设置参数
```
<param name="参数名" type="参数类型" value="" />

<rosparam command="load/dump", file="yaml文件路径"/>
//通常新建dump.launch文件进行导出参数
<rosparam coommand="delete", param="参数名" />
```
当标签在< node >外时为全局参数，里面为私有参数

**group参数**：用于分组，可以在不同的组中启用相同的节点名称

**arg标签**：类似于宏定义，定义参数
```
<arg name="参数名0" default="默认值" />
<param name="参数名1" value="$(arg 参数名0) /">
```

## 工作空间覆盖
ROS中会自定义工作空间，工作空间内功能包不能重名，不同工作空间功能包可以重名。

在.bashrc里添加环境变量source，最后添加的优先级最高

重名的解决策略：使用命名空间或名称重映射

**节点名称重名**：
```
rosrun turtlesim turtlesim_node __ns:=name1
//设置命名空间
rosrun turtlesim turtlesim_node __name:=name1
//设置别名
```
也可以在launch文件中添加ns或者设置不同的name，或者在init中添加随机后缀

**话题重名**：
1. ros实现
```
rosrun 包名 节点 话题名:=新话题名
```

2. launch实现(使用remap标签)
3. 编码设置话题名称：话题可分为三类
   * 全局：在根目录下，和命名空间同级
   * 相对：在命名空间下，和节点同级
   * 私有：属于节点
```
//全局话题
Publisher pub = nh.advertise<std_msgs::String>("/话题名",10)
//相对话题
Publisher pub = nh.advertise<std_msgs::String>("话题名",10)
//私有话题
NodeHandle nh("~");
Publisher pub = nh.advertise<std_msgs::String>("话题名",10)
```
python中同理

参数名称设置的方法类似