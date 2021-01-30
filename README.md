# Flight Control
飞控学习笔记，在这个文件里面主要解释我们所碰到的部分

---

## 巡线例程

### [drv_SDI.cpp](./document/drv_SDI.md)

数字分量串行接口，:outbox_tray:用来处理输入的数据。

### [M35_Auto1.cpp](./document/M35_Auto1.md)

用来存放自动飞行时的任务，自主飞行就是通过这个来实现的，在这里我们也需要放实现的代码