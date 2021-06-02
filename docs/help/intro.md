## Introduce

### 文件目录

- docs: 存放软件相关文档，包括使用、计划功能、已实现功能等文档
- lib: 存放mm32f103相关的库文件
- tools: 存放相关使用工具
- project: keil MDK工程目录
- src: 存放程序代码，其中
  - config: 配置相关头文件
  - drivers: 外设驱动文件
  - osd: osd实现
  - sensors: 传感器读取等文件
  - startup: 启动文件
  - vcp: 虚拟串口
  - log: 飞行数据记录
  - mavlink: 通信协议，预留调试
  - flight: 姿态控制等实现
  - rx: 接收机相关代码
  - math: 存放滤波等数学相关文件
  - battery: 电压监测部分
  - main.c: 用户主程序
- developLog: 记录开发进度
- README.md





### branch

- master: 用于验证后发布的代码，合并到这个分支

- develop: 调试后的没有bug的代码合并到这个分支，不用于发布

- develop_feature: 用于添加一些新的特性，调试好后合并到develop分支

- develop_iap:这个作用类似于bootloader,更新固件，mm32芯片目前不支持DFU

- old_struct:这个主要还是移植liteSilverware的代码，姿态控制部分没有变，

  以后会逐渐转移到develop分支提交代码，逐渐解耦现有的LiteSilverware结构