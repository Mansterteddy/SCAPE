# SCAPE: Shape Completion and Animation of People
This repository includes the implementation of SCAPE: Shape Completion and Animation of People, the project includes training part and generating part. I use matlab to implement training part, and use python to implement generating part. If you want to know more details, please feel free to contact me.

小八卦：这篇文章作者博士答辩团里有Andrew Ng。

## Main Contribution

1、一个包含不同姿势和不同体型的人体数据库，每个人体mesh都很好地进行了预处理，达到了同拓扑的结构。

2、通过调整参数，包括姿势参数和体型参数，可以生成具有真实感的人体mesh。

## 为什么如此有效？

1、作者发现通过R矩阵，使用线性回归，去预测Q矩阵的效果很好，和真实得到的Q矩阵相差不大，因此这篇文章如此经典，很多工作都是基于SCAPE进行改进。

2、对于不同体型的人体，使用PCA主成分分析，得到S矩阵，从而调整人体体型。

## 主要内容

1、bodyseg文件夹中包含的是一个标准人体模型的拓扑结构，整个mesh模型包括12500个顶点和24995个面，对应mesh文件来说，有了点的坐标和面的连接关系，才能构造出一个完整的mesh结构。这个拓扑结构包含以下几个部分的帮助文件：

* partidx文件夹：
  * 指代人体的文件，如all.txt，chest.txt等，指出了mesh每个点所属的人体部位；
  * *neighbortri.txt，指出了mesh上每一个点的相邻点id；
  * tri.txt，指出了mesh上每一个三角面片，对应的点id；
  * tripart.txt，指出了每一个三角面片所属的人体部位；
  * vertpartidx.txt，指出了每一个点所属的人体部位；
* partvert文件夹：
  * 指代人体的文件，如chest.txt等，指出了部位中每一个点对应的坐标；
  * *tri.txt，指出了每个点所属的三角面片id；
* obj和txt文件：
  * obj文件，指的是依次去除对应部位后的人体模型；
  * txt文件，指的是上述模型对应的人体mesh点坐标；
* blendposemodel.txt保存了训练得到的A矩阵，即文中4式；

2、scape文件夹中包含了原论文提供的71个不同姿势的人体模型obj文件；

3、SPRING_MALE_2文件夹中包含了原论文提供的150个不同体型的人体模型obj文件；

程序文件：

其中部分文件路径名需要修改。

helper function:

* rotation2twist.m将旋转矩阵转化为twist坐标，即文中Ma Yi的公式；
* mySVD.m提供了矩阵SVD分解的一个快速实现；

Q矩阵相关：

* generatedeltar.m将第一个人体模型作为标准模型，计算了其他70个不同姿势的人体模型，对应该标准模型的twist矩阵；
* generateQmatrix.m计算了其他70个不同姿势的人体模型，对应该标准模型的Q矩阵；
* newQmatrix_triangle.m负责具体计算，即计算文中5式；
* regression_twist2Qmatrix.m计算得到了A矩阵，即文中4式；
* posemodel_training.m将计算得到的A矩阵，保存到了blendposemodel.txt这个文件中；

S矩阵相关：

* generateSmatrix.m计算了150个不同体型人体，对应标准模型的S矩阵，当然可以扩展到更多的人体模型；
* newSmatrix_triangle.m在给定旋转矩阵、变换前后点的对应关系等条件下计算得到了S矩阵，即文中8式；

生成模型相关：

* scapetracking_withoutshape.m用于生成不同pose的人体，但是无法改变人体shape；
* scapetracking_shape_pose.m用于生成不同pose、不同shape的人体；

训练过程：

```matlab
trainQ = generateQmatrix() %生成Q矩阵
deltar = generatedeltar() %生成twist矩阵
pose_model = posemodel_training() %生成A矩阵，保存在了blendposemodel.txt中
U, mu, trainS1 = generateSmatrix() %生成特征值、平均值和特征向量
scapetracking_withoutshape() %生成不同姿势的人体
scapetracking_shape_pose() %生成不同姿势不同体型的人体
save train pose_model U eigen_value amu %将训练结果保存到train中
```

使用以上matlab代码，就可以训练出所需要使用的数据，我们将所有需要的数据保存在了train这个矩阵中，随后我们转用python来调用这些数据，生成人体模型。

train中的变量：

* pose_model：文中式3中的矩阵A；
* eigen_value：S矩阵分解后得到的特征值，使用时先开根号；
* U：文中式7中的矩阵U；
* amu：文中式7中的$\mu$；

python代码：

* generate_model.py 主程序，用于生成不同姿势、不同体型的人体模型；
* handle.py，用于处理各种读写文件问题；
* load_mat.py，用于载入mat文件；
* procrustes.py，用于计算旋转矩阵等；
* rotation2twist.py，用于换算旋转矩阵和twist；
* test_proc.py，用于测试procrustes.py；
* write_obj.py，用于写入obj文件；

为了生成目标人体，需要调用如下程序：

```shell
python generate_model.py
```

