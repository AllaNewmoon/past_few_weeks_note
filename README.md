# past_few_weeks_note
allanewmoon
# ROS
## 通信机制
**流程**：
1.Talker注册，通过端口使用RPC向ROS Master注册发布者信息，包括消息的话题名。
2.Listener注册，通过RPC注册订阅者信息，包括话题名
3.ROS Master匹配信息，若找到则通过RPC向Listener发送Talker的消息
4.Listener发送连接请求，传输订阅的话题名，消息类型和通信协议
5.Talker确认连接信息
6.Listener使用TCP建立网络连接
7.Talker向Listener发布数据
（服务通信比话题通信步骤少）

**发布方实现**：
1.初始化ROS节点
```
init(argc, argv, "Name")
```
2.创建节点句柄
```
NodeHandle nh;
```
3.创建发布者对象
```
Publisher pub = nh.advertise<String>("话题", 缓冲区大小)
```
4.编写发布逻辑，发布数据
```
Rate rate(10);//设置10hz频率
count ++
Duration(3).sleep();//休眠3s
while(ok){
    msg.data="发送内容";
    pub.publish(msg);
    ROS_INFO("发布的数据是:%s", ss.str().c_str())；//输出日志
    rate.sleep();//每隔0.1s发布一次
}//循环发布

```

**订阅方实现**：
1.初始化ros节点
2.创建节点句柄
3.创建订阅者对象
```
Subscribe sub = nh.subscribe("订阅的话题", 缓冲区大小，回调函数)
```
关于回调函数的一个例子：
```
void doMsg(const String::Constptr &msgs){
    ROS_INFO("翠花订阅的数据:%s", msgs->data.c_str());
}//subscribe返回的是对string的指针的引用
```
4.处理订阅的数据
```
spin()//不断执行回调函数
```

**显示计算图**：rqt_graph

**python实现的区别**：
发布方：
```
rospy.init_node("SanDai")\\初始化
    pub = rospy.Publisher("che", String, queue_size=10)
    msg = String()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg.data = "hello"
        pub.publish(msg)
        rate.sleep()
```
订阅方：
```
ef doMsg(msg):
    rospy.loginfo("订阅的数据:%s",msg.data)

if __name__ == "__main__":
    rospy.init_node("HuaHua")
    sub = rospy.Subscriber("che",String,doMsg,queue_size=10)
    rospy.spin()
```

**自定义Msg**：
1.定义Msg文件：（包含int，float，string，time，duration，other msg files，array，header）
```
string name
uint16 age
float64 height
```
2.编辑配置文件
在package.xml文件中添加
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
在CMakeList中添加
```
message_generation//find_package中
```
```
## 配置 msg 源文件
add_message_files(
  FILES
  Person.msg
)
```
```
# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
```
#执行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo02_talker_listener
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```
3.编译生成可以被python或C++调用的中间文件

4.使用自定义Msg：
* setting.json中添加dist-package的路径
* python导包：from plumbing_pub_sub.msg import Person
* c++导包：#include "demo02_talker_listener/Person.h"
* using namespace demo02_talker_listener

## 服务通信
服务通信是基于请求响应模式的，是一种应答机制。也即: 一个节点A向另一个节点B发送请求，B接收处理请求并产生响应结果返回给A。

**步骤**：
1.Server注册，包含提供服务的名字（服务端）
2.Client注册，包含需要请求的服务的名称（客户端）
3.ROS MASTER根据注册表信息匹配，并通过RPC向Client发送Server的TCP地址信息。
4.Client发送请求
5.Server发送响应

**自定义srv**：在功能包下新建文件夹srv，创建srv文件
```
int32 num1
int32 num2
---//请求和响应之间用横杠隔开
int32 sum
```
修改package.xml，修改cmakelist中的find_package和add_service_files和generate_messages和catkin_package

在c_cpp_properties.json和settings.json中添加相应路径

**srv调用**(服务端)：
1.包含头文件
```
#include "plumbing_server_client/Addints.h"
```
2.初始化ROS节点
3.创建节点句柄
4.创建服务对象
```
ServiceServer server = nh.advertiseService("addInts", doNums);
//addInts为服务名称
```
回调函数doNums:
```
bool doNums(plumbing_server_client::Addints::Request &request,
            plumbing_server_client::Addints::Response &response){
    int num1 = request.num1;
    int num2 = request.num2;
    ROS_INFO("收到的请求数据:num1 = %d, num2 = %d", num1, num2);
    int sum = num1 + num2;
    response.sum = sum;
    ROS_INFO("求和的结果:sum = %d", sum);
    return true;
}
```
注意返回类型是bool，接收参数类型是Addints下的Request和Response引用。

5.修改CMakeist
```
add_dependencies(AddInts_Server ${PROJECT_NAME}_gencpp)
```
6.测试:rosserveice call addInts ...

**srv调用**(客户端):
```
ServiceClient client = nh.serviceClient<plumbing_server_client::Addints>("addInts");
plumbing_server_client::Addints ai;
ai.request.num1 = 100;
ai.request.num2 = 200;//组织请求
bool flag = client.call(ai);//处理响应
if(flag){
        ROS_INFO("响应成功！");
        ROS_INFO("响应结果 = %d", ai.response.sum);
    }else{
        ROS_INFO("处理失败！");
    }
    return 0;
```
**动态提交参数**：
```
ai.request.num1 = atoi(argv[1]);
ai.request.num2 = atoi(argv[2]);
```

**客户端挂起**：
1.调用判断服务器状态的函数：
```
client.waitForExistence();
```

2.ros::service下的函数
```
service::waitForService("addInts");
```

**python实现**(服务端):
```
//定义回调函数
def doNums(request):
    num1 = request.num1
    num2 = request.num2
    sum = num1 + num2
    response = AddIntResponse()//将结果封装进响应
    response.sum = sum
    return response

server = rospy.Service("addInts", Addints.doNum)
```

**python实现**(客户端):
```
client = rospy.ServiceProxy("addInts", Addints)
client.call(12,34)

//动态传参
import sys
if len(sys.argv) != 3:
    rospy.logerr("传入个数不对")
    sys.exit(1)

num1 = int(sys.argv[1])
num2 - int(sys.argv[2])
response = client.call(num1,num2)

//客户端挂起
client.wait_for_service()
rospy.wait_for_service("addInts")
```

## 参数服务器
实现不同节点间的数据共享，相当于公共容器，可以将数据存在容器被调用。

**步骤**：
1.Talker设置参数，ROS Master将参数保存到参数列表
2.Listener向RPC服务器发送参数查找请求
3.ROS Master向Listener发送参数值
**

**实现增改数据**：
通过ros::NodeHandle下的setParam和ros::param下的set实现
```
nh.setParam(键，value)
//value也可以是数组，map等
```

**实现查询参数**：
```
param(键, 默认值)//存在则返回结果，否则返回默认值
getParam(键, output)//存在则返回true，且将结果赋给output
getParamNames(vector<string>)//将所有的键存在vector中
hasParam(键)//判断是否有该键，有则返回true
searchParam(键, output)
getParamCached//性能上好于getParam
```

**删除元素**：
```
nh.deleteParam(键)//返回是否删除成功
param::del(键)
```

**python中对应的API**：
```
rospy.set_param(键, value)

get_param(键,默认值)
get_param_cached
get_param_names
has_param
search_param

rospy.delete_param
```

## 常用API
* rosnode list / ping / info / machine / kill / cleanup
* rostopic list / echo / info / pub
* rosservice list / call / info / type 
* rosmsg list | grep -i / show / info / package / packages / md5
* rossrv list / info plumbing_server_client/AddInts / md5 / package / packages
* rosparam set / get / dump output.yaml / load input.yaml

**话题与消息获取**：
```
rostopic list//列出话题
rostopic type /turtle1/cmd_vel//找到想要话题的消息类型
rosmsg info geometry_msgs/Twist//获取消息格式
```

**服务与消息获取**：
```
rosservice list
rosservice info /spawn//得到消息类型，节点，参数
rossrv info turtlesim/Spawn//获取srv服务器的请求，响应参数
```
