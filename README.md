# SCAPE: Shape Completion and Animation of People
小八卦：这篇文章作者博士答辩团里有Andrew Ng。

## Main Contribution

1、一个包含不同姿势和不同体型的人体数据库，每个人体mesh都很好地进行了预处理，达到了同拓扑的结构。

2、通过调整参数，可以生成具有真实感的人体mesh，对比数据库中的人体，具有不同姿势和不同体型。

## 为什么如此有效？

作者发现通过R矩阵，使用线性回归，去预测Q矩阵的效果很好，和真实得到的Q矩阵相差不大，因此这篇文章如此经典，很多工作都是基于SCAPE进行改进。

## 主要内容

本仓库内，没有提供训练过程，只提供了训练结果，以及使用训练结果，生成人体模型的相关程序。



This repository includes the implementation of SCAPE: Shape Completion and Animation of People using python. I didn't provide a good document to explain the program. If you want to know more, please feel free to contact me.





