### TPA

Throttle PID Attenuation

#### 作用

- 在快速油门下，适当降低PID的增益，减少由于PID高增益带来的抖动
- 作用在P和D上
- 代码实现上，在PID基础上乘以TPA系数，相对没有复杂的过程



#### 应用

1、在kiss飞控上的使用

- 根据油门值来调整PID
- 根据电池电量来调整PID
- 有两个breakpoint
- 根据下图猜测：
  - 可以将油门值分为三段区间，每段区间的TPA系数不同；
  - 将电池电压划分为三段区间，<voltage1, < voltage2, <voltage3,   这个有点类似低电压补偿
  - 原始的PID * 油门TPA * 电压TPA  = 实际PID输出

<img src="C:\Users\wenzh\AppData\Roaming\Typora\typora-user-images\image-20200806191140711.png" alt="image-20200806191140711" style="zoom:80%;" />



2、在betaflight上的使用

- 根据油门值来调整PID，可以选择降低P term或者 P term + D term 带来的高增益
- ![image-20200807092649600](https://i.loli.net/2020/08/07/GK8F9wQ6IHXDY45.png)
- <img src="C:\Users\wenzh\AppData\Roaming\Typora\typora-user-images\image-20200807092731998.png" alt="image-20200807092731998" style="zoom:80%;" />
- 有一个breakpoint, 油门值高于这个breakpoint时，tpa才会起作用
- ![image-20200807094056703](https://i.loli.net/2020/08/07/EKHox3OR1aLsJkW.png)

<img src="C:\Users\wenzh\AppData\Roaming\Typora\typora-user-images\image-20200806191038748.png" alt="image-20200806191038748" style="zoom:80%;" />