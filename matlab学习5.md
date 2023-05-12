# SerialLink补充
## 绘制想要的轨迹
1.绘制圆路径示例：
```
p0 = [0, 0, 0];//圆的圆心
p1 = [0.4, 0.4, 0.4];//圆的起始点
p2 = [-0.5, 0.5, 0.5];//终点

X = p1 - p0;
B = p2 -p0;
Z = cross(X,B);//计算X和B的叉乘，Z垂直圆弧平面
Y = corss(Z,X);//Y垂直Z，X平面
x1 = X/norm(X);
y1 = Y/norm(Y);
z1 = Z/norm(Z);
R = [x1.', y1.', z1.'];//构建旋转矩阵
Js = [R, p0.'];
Js = [Js; 0 0 0 1];//构建从原点p0到起始点p1的齐次变换矩阵，其中原点不变
r = norm(p1 - p0)//计算半径

t = 0:0.05:5
[s,sd,~] = tpoly(0, 2*pi, t)//5s内对0到2pi插值，s表示位置，sd表示速度
q1=[-12.7,98.9,-107,158,-114,11]; 
k = q1;//初始关节角度向量赋给k
vs = []//储存关节速度
pd = []//储存笛卡尔坐标系下坐标位置
for i = 1:length(s)
    T=[1 0 0 r*cos(s(i));0 1 0 r*sin(s(i));0 0 1 0;0 0 0 1];//构建从p1坐标系到圆上坐标系的齐次变换矩阵
    T1=Js*T;//求出圆上点的坐标系       
    q=ZU3.ikunc(T1,k); //求解逆运动
    k=q;//迭代，给k复制当前关节向量
    pd=[pd;T1(1:3,4).'];
    vs=[vs;q];
end

close all;
figure('NumberTitle', 'off', 'Name', '运动图像');
plot2(pd,'r','LineWidth',2);%圆的线宽
hold on;
ZU3.plot(vs,'fps',20);%输出运动图像
%绘制运动曲线
figure('NumberTitle', 'off', 'Name', '关节值随时间变化曲线');
    subplot(4,1,1);%图像布局
    plot(t,pd);  %输出空间位置曲线
    title('空间位置曲线')
    xlabel('秒(s)');
    ylabel('路程(m)')
    
    subplot(4,1,2);   
    plot(t,vs);%输出关节角度曲线
    title('关节角度曲线')  
    xlabel('秒(s)');  
    ylabel('角度(rad)');
    legend('θ1','θ2','θ3','θ4','θ5','θ6');
    
    %计算空间速度并输出空间速度曲线
    vx = -r*sin(s).*sd;//对圆的参数方程求导得出
    vy = r*cos(s).*sd;
    vv = r*sd;
    subplot(4,1,3);   
    plot(t,[vx,vy,vv]);
    title('空间速度曲线')
    xlabel('秒(s)');   
    ylabel('速度(m/s)')
    
    %计算关节速度
    omiga=[];
    for i =1:length(s)
        J0=ZU3.jacob0(vs(i,:)');
        dq=J0\[vx(i);vy(i);0;0;0;0];//J*关节速度 = 末端空间速度，利用矩阵除法计算关节速度
        omiga=[omiga;dq'];
    end
    subplot(4,1,4);  
    plot(t,rad2deg(omiga));
    title('关节速度曲线')%输出关节速度曲线
    xlabel('秒(s)');  
    ylabel('角速度(rad/s)')
```

2.绘制直线路径示例：
```
% 定义直线的起点和终点
p1 = [1 1 1];
p2 = [2 2 2];

% 计算直线上的点
nPoints = 10;
x = linspace(p1(1), p2(1), nPoints);
y = linspace(p1(2), p2(2), nPoints);
z = linspace(p1(3), p2(3), nPoints);

% 计算机器人在每个点处的关节角度
q = zeros(nPoints, robot.n);
for i = 1:nPoints
    T = transl(x(i), y(i), z(i));
    q(i,:) = robot.ikine(T);
end

% 绘制机器人的运动
figure('NumberTitle', 'off', 'Name', '运动图像')
for i = 1:nPoints
    robot.plot(q(i,:))
    pause(0.1)
end
```
3.绘制多段轨迹
```
//指定路径的关节坐标（可以由齐次矩阵反解得出）
qsq1=[0.46088 0.37699 0 1.31 0 1.4451 0];
qsq2=[.81681 0.56549 0 1.0681 0 1.2566 0 ];
qsq3=[2.36 0.69115 0 0.848 0 1.4451 0 ];
qsq4=[2.66 0.37699 0 1.31 0 1.4451 0];
qsq5=[pi/2 0.62831 0 1.5708 0 0.94249 0];
qsq6=[0 0.62831 0 1.5708 0 0.94249 0];
//利用jtraj对关节坐标插值
t=0:0.04:2;
sqtraj1=jtraj(q0,qsq1,t); 
sqtraj2=jtraj(qsq1,qsq2,t); 
sqtraj3=jtraj(qsq2,qsq3,t); 
sqtraj4=jtraj(qsq3,qsq4,t);
sqtraj5=jtraj(qsq4,qsq5,t);
sqtraj6=jtraj(qsq5,qsq6,t);
sqtraj7=jtraj(qsq6,q0,t);

hold on
atj=zeros(4,4);
view(-35,40)
xlim([-40,40])
ylim([-40,40])
zlim([0,60])

for i=1:1:51
    atj=Rbt.fkine(sqtraj1(i,:));//计算正解得到se(3)转换矩阵
    jta=transpose(atj.T);//atj.T得到矩阵部分
    JTA(i,:)=jta(4,1:3);//得到平移部分，即每个点的坐标
    jta=JTA;//将坐标点再赋给jta
    plot2(jta(i,:),'r.')//画出运动过程中的坐标点
    Rbt.plot(sqtraj1(i,:))//画出机器人运动
    plot2(JTA,'b')//将坐标点连成线表示轨迹
end
```

**总结**：1.设定空间中图形的参数和参数方程，设定从原点到图形原点的旋转矩阵。
2.构建齐次变换矩阵Js，用pd，vs储存空间坐标和关节速度
3.在迭代中由参数方程求得齐次矩阵，pd与vs。

## 绘制空间中的点云图
1.绘制球形的散点图
```
% 设置球心的位置
center = [5, 10, 3];

% 生成随机的球状点云数据
numPoints = 1000;
radius = 10;
x = randn(numPoints, 1);
y = randn(numPoints, 1);
z = randn(numPoints, 1);
r = sqrt(x.^2 + y.^2 + z.^2);
x = center(1) + radius * x ./ r;
y = center(2) + radius * y ./ r;
z = center(3) + radius * z ./ r;

% 绘制球状散点图
figure;
scatter3(x, y, z, 20, 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'none');
axis equal;
xlim([-10 10]);
ylim([-5 5]);
zlim([0 20]);//自定义显示区域
view(3);
```
2.创建能够实时调整的图像:利用surf和uicontrol
```
function sphere_slider()
% Create the figure and axis
fig = figure;
ax = axes('Parent', fig, 'Position', [0.2 0.2 0.6 0.6]);

% Create the initial sphere
r = 1;
[x, y, z] = sphere(50);
h = surf(ax, r*x, r*y, r*z);
xlim([-5 5]);
ylim([-5 5]);
zlim([-5 5]);

% Create the slider
slider = uicontrol('Style', 'slider', 'Min', 0.1, 'Max', 2, ...
    'Value', r, 'Position', [100 50 200 20], 'Callback', @slider_callback);

% Define the slider callback function
function slider_callback(source, event)
    r = get(source, 'Value');
    set(h, 'XData', r*x, 'YData', r*y, 'ZData', r*z);
end

end
```
其中surf(ax, X, Y, Z)在ax坐标区绘制指定的曲面，x,y,z可由类似[x, y, z] = sphere(50)得到。也可以利用patch(surf2patch(x,y,z))，可以指定透明度等。