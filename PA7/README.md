
I've finished task ...

---

> 以下内容是完成过程中的踩坑点，并非作业提交说明

- Windows 平台低于 GCC 9.2 的 [`std::random_device`](https://zh.cppreference.com/w/cpp/numeric/random/random_device) 对象为确定性的，无法生成随机数，且原有的随机数写法性能较差
```cpp
// Test code
#include <iostream>
#include <random>

int main()
{
    std::random_device rd;
    std::cout << rd();
    return 0;
}
```
- 光源区域为黑色的原因：着色点和光源采样点在一个平面上（因为场景中只有一个光源），因此 `dot(-ws, NN) == 0`，解决办法是判断通过着色点的 `emit` 属性来直接给光源赋值
- 光线与包围盒相交的判断条件：作业 6 中写为 `(t_enter < t_exit) && (t_exit >= 0)`，但在作业 7 场景中，光线进出墙壁的包围盒的时间是相等的，因为墙壁与坐标轴平面平行且厚度为 0（从 obj 文件中可以看出，墙壁仅由一个平面组成，并非长方体；左边的墙壁包围盒不平行于任何坐标轴，因此它不受上述条件的影响）。更多讨论参考[此帖](http://games-cn.org/forums/topic/zuoyeqiguangxianxiangjiaoceshiyichang/)