# rxs-3dgrabber

多相机 3D 点云采集与标定上位机 -- 苏州锐新视科技有限公司

基于 PCL (Point Cloud Library) 的多相机三维点云标定、合并与分析工具集，包含标定核心 DLL、圆柱体点云分析以及批量文件采集分类三个子模块。

---

## 子模块概览

| 模块 | 类型 | 说明 |
|------|------|------|
| **BallCalibration DLL** | 共享库 (`.dll`) | 多相机标定核心库，通过标定球实现多相机坐标系对齐与点云合并，导出 6 个 C API |
| **Cylinder** | 可执行程序 | 圆柱体点云分析：RANSAC 圆柱拟合、曲面展开（展平）、半径滤波、间隙检测 |
| **fileGet** | 可执行程序 | 批量文件采集分类工具，按配置从子目录中收集文件并按标定板编号分组归档 |

---

## BallCalibration DLL -- 多相机标定核心库

### 标定工作流

```
采集标定球点云 --> RANSAC 拟合球心 --> SVD 配准 --> 应用变换合并点云
```

1. **采集标定球**: 多个相机在不同时序下拍摄已知半径的标定球，获取各组点云
2. **RANSAC 拟合球心**: 对每组标定球点云使用 RANSAC + 最小二乘球面拟合，精确提取球心坐标
3. **SVD 配准**: 以某一相机为基准坐标系，利用各相机对应的球心对应点集，通过 SVD (奇异值分解) 求解刚性变换矩阵 (旋转 + 平移)
4. **应用变换合并点云**: 将各相机的点云通过各自的变换矩阵统一转换到基准坐标系下，完成多相机点云合并

### DLL 导出 API

DLL 通过 `Source.def` 导出以下 6 个函数：

#### `updateCalibrationObj`

```cpp
void* updateCalibrationObj(vector<vector<CP>> ball_camera)
```

创建并更新标定对象。输入为多时序、多相机的标定球点云，自动完成球心拟合与 SVD 配准。

| 参数 | 类型 | 说明 |
|------|------|------|
| `ball_camera` | `vector<vector<CP>>` | 外层 vector 为不同时序的标定球；内层 vector 为同一时序下各相机的标定球点云，各时序的相机顺序须一致 |
| **返回值** | `void*` | 标定对象指针，后续 API 通过此指针引用标定结果 |

#### `mergeCloud`

```cpp
CP mergeCloud(void* obj, vector<CP> clouds)
```

使用标定结果合并多相机点云。

| 参数 | 类型 | 说明 |
|------|------|------|
| `obj` | `void*` | 由 `updateCalibrationObj` 或 `loadObj` 返回的标定对象 |
| `clouds` | `vector<CP>` | 各相机的点云，顺序须与标定时一致 |
| **返回值** | `CP` | 合并后的点云 |

#### `saveObj`

```cpp
void saveObj(void* obj, int num, string file_name)
```

将标定对象（变换矩阵组）序列化保存到本地文件。

| 参数 | 类型 | 说明 |
|------|------|------|
| `obj` | `void*` | 标定对象指针 |
| `num` | `int` | 相机数目 |
| `file_name` | `string` | 保存路径 |

#### `loadObj`

```cpp
void* loadObj(int num, string file_name)
```

从本地文件加载已保存的标定对象。

| 参数 | 类型 | 说明 |
|------|------|------|
| `num` | `int` | 相机数目 |
| `file_name` | `string` | 标定文件路径 |
| **返回值** | `void*` | 标定对象指针 |

#### `fitPlane`

```cpp
Eigen::Vector4f fitPlane(CP& cloud)
```

给定点云拟合平面方程 `ax + by + cz + d = 0`，至少需要三个不共线的点。输入的 `cloud` 会被替换为拟合出的平面采样点云。

| 参数 | 类型 | 说明 |
|------|------|------|
| `cloud` | `CP&` | 输入选定的平面区域点云；调用后被替换为平面采样点云 |
| **返回值** | `Eigen::Vector4f` | 平面方程系数 `(a, b, c, d)` |

#### `pointDisPlane`

```cpp
float pointDisPlane(Eigen::Vector4f plane, PointT point)
```

计算点到平面的距离。

| 参数 | 类型 | 说明 |
|------|------|------|
| `plane` | `Eigen::Vector4f` | 平面方程系数 `(a, b, c, d)` |
| `point` | `PointT` | 三维点 `(x, y, z)` |
| **返回值** | `float` | 点到平面的距离 |

> **类型定义**: `CP` = `pcl::PointCloud<pcl::PointXYZ>::Ptr`，`PointT` = `pcl::PointXYZ`，`VCP` = `std::vector<CP>`

### DLL 调用示例

```cpp
#include <Windows.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CP;

// 动态加载 DLL
HMODULE hDLL = LoadLibrary(L"BallCalibrationDLL.dll");

// 获取函数指针
auto updateCalibrationObj = (void*(*)(vector<vector<CP>>))
    GetProcAddress(hDLL, "updateCalibrationObj");
auto mergeCloud = (CP(*)(void*, vector<CP>))
    GetProcAddress(hDLL, "mergeCloud");
auto loadObj = (void*(*)(int, string))
    GetProcAddress(hDLL, "loadObj");

// 加载已有标定并合并点云
void* bc = loadObj(4, "cal.txt");
vector<CP> fan_clouds;  // 各相机点云
// ... 加载点云 ...
CP merged = mergeCloud(bc, fan_clouds);
pcl::io::savePCDFileBinary("merge.pcd", *merged);
delete bc;
```

---

## Ransac3D 可扩展框架

`BallCalibrationDLL` 内部实现了一套可扩展的 RANSAC 三维拟合框架 (`Ransac3D`)，采用抽象基类 + 具体模型的继承模式：

```
Ransac3D (抽象基类)
  +-- RansacBall (球体拟合)
```

### 核心机制

- **采样策略**: 按贴合计数排序的智能采样 -- 历史贴合次数高的点优先被选中，贴合差的点逐步淘汰
- **拟合方法**: RANSAC 随机采样 + 最小二乘 (SVD) 球面拟合
- **评估与早停**: 当贴合点数超过总点数的 80% 时提前终止迭代
- **参考半径约束**: 可设定参考半径 `ref_r` 和容差 `threshold`，在半径偏差超限时快速拒绝无效拟合
- **多线程支持**: `RansacBall::runM()` 提供 OpenMP 并行化版本

### 扩展方式

继承 `Ransac3D` 基类，实现以下三个虚函数即可扩展新的几何体拟合（如圆柱、圆锥等）：

| 虚函数 | 职责 |
|--------|------|
| `fit()` | 从采样点云拟合几何模型参数 |
| `evaluate()` | 评估当前拟合结果，返回 `true` 表示满足早停条件 |
| `useBest()` | 将历史最优参数写回当前结果 |

---

## Cylinder -- 圆柱体点云分析

Cylinder 模块提供圆柱体点云的完整分析流程：

| 功能 | 说明 |
|------|------|
| **圆柱拟合** | 法向量估计 + RANSAC 圆柱模型分割 (`SACMODEL_CYLINDER`)，提取轴心点、轴方向、半径共 7 个参数 `(x0, y0, z0, a, b, c, r)` |
| **曲面展开** | 将三维圆柱面点云映射到二维展开平面：X 轴为角度弧长，Y 轴为沿轴距离，Z 轴为到轴面距离 |
| **噪声滤波** | 半径滤波器 (`RadiusOutlierRemoval`) 去除展开后的离群噪声 |
| **间隙检测** | 将展开点云按 X 轴分段，在每段中查找 Y 方向最大间隙，统计指定区域内的平均间隙值 |
| **可视化** | PCL 3D 可视化器显示圆柱拟合结果 |
| **参数持久化** | 圆柱参数的保存与加载（文本格式） |

---

## fileGet -- 批量文件采集分类

fileGet 是轻量级的批量文件采集分类工具，用于从多层子目录结构中收集采集文件并按标定板编号分组归档。

**工作流程**:

1. 读取配置文件 `conf.czx`，获取源目录 `root`、文件名模板 `file_name`、标定板数量 `board_num`
2. 扫描源目录下的所有子目录
3. 在各子目录中查找匹配文件名的文件
4. 按轮转方式 (`i % board_num`) 将文件复制到以标定板编号命名的输出目录中

---

## 项目结构

```
rxs-3dgrabber/
+-- CMakeLists.txt                    # 顶层 CMake，C++17，三个子模块可选构建
+-- LICENSE                           # BSL 1.1 许可证
+-- README.md
+-- SECURITY.md
+-- 3d-detect/
|   +-- BallCalibration/
|   |   +-- CMakeLists.txt            # BallCalibration 构建：DLL + 测试程序
|   |   +-- BallCalibration.sln       # Visual Studio 解决方案
|   |   +-- BallCalibration.vcxproj
|   |   +-- conf.czx                  # 标定运行配置
|   |   +-- 源.cpp                    # 测试/调用示例：动态加载 DLL 执行标定与合并
|   |   +-- BallCalibrationDLL/
|   |       +-- BallCalibration.h     # BallCalibration 类声明
|   |       +-- BallCalibration.cpp   # BallCalibration 类实现
|   |       +-- Ransac3D.h            # Ransac3D 框架 + RansacBall 声明
|   |       +-- Ransac3D.cpp          # Ransac3D 框架 + RansacBall 实现
|   |       +-- Source.def            # DLL 导出定义 (6 个导出函数)
|   |       +-- dllmain.cpp           # DLL 入口 + 导出函数实现 + fitPlane/pointDisPlane
|   |       +-- pch.h                 # 预编译头
|   |       +-- framework.h           # Windows 头文件
|   +-- Cylinder/
|       +-- CMakeLists.txt            # Cylinder 构建
|       +-- Cylinder.sln              # Visual Studio 解决方案
|       +-- cylinderCoeff.txt         # 圆柱参数文件
|       +-- 源.cpp                    # 圆柱拟合、展开、间隙检测主程序
+-- fileGet/
    +-- CMakeLists.txt                # fileGet 构建
    +-- fileGet.sln                   # Visual Studio 解决方案
    +-- conf.czx                      # 文件采集配置
    +-- 源.cpp                        # 批量采集分类主程序
```

---

## 构建指南

### 前置条件

| 依赖 | 版本要求 | 说明 |
|------|----------|------|
| CMake | >= 3.16 | 构建系统 |
| C++ 编译器 | 支持 C++17 | 主要支持 MSVC (Windows) |
| PCL | 1.x | Point Cloud Library -- 需包含 `common`, `io`, `segmentation`, `visualization` 模块 |
| Eigen3 | 3.x | 线性代数库（通常随 PCL 安装） |
| OpenMP | -- | 可选，用于 RANSAC 并行化 |

### czxDependence 外部依赖 (重要)

> **注意**: 本项目的所有源文件均通过相对路径引用了位于仓库外部的 `czxDependence` 库。这是当前版本的一个架构问题，后续应将该依赖纳入仓库管理或替换为标准库。

引用该依赖的文件及路径：

| 源文件 | include 路径 | 使用的功能 |
|--------|-------------|-----------|
| `BallCalibrationDLL/pch.h` | `../../../czxDependence/czxTool.h` | 类型定义 (`CP`, `VCP`, `PointT`, `CloudT`)、序列化工具 (`czxTool::saveMatrix4f`/`loadMatrix4f`)、可视化工具 (`Tool::show`/`showComparison`) |
| `BallCalibrationDLL/Ransac3D.h` | `../../../czxDependence/czxTool.h` | 类型定义、基础工具 |
| `BallCalibration/源.cpp` (测试) | `../../czxDependence/czxTool.h` | 文件工具 (`czx_file::readProfile`/`getSubdictories`/`pathGather`) |
| `Cylinder/源.cpp` | `../../czxDependence/czxTool.h` | 可视化工具 (`Tool::showComparison`) |
| `fileGet/源.cpp` | `../czxDependence/czxTool_std.h` | 文件工具 (`czx_file::readProfile`/`getSubdictories`) |

**构建前需要确保** `czxDependence` 目录位于仓库根目录的上级目录中，即目录结构应为：

```
parent/
+-- czxDependence/
|   +-- czxTool.h
|   +-- czxTool_std.h
+-- rxs-3dgrabber/
    +-- 3d-detect/
    +-- fileGet/
```

### CMake 构建

```bash
# 在仓库根目录执行
cmake -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

可通过 CMake 选项选择性地构建子模块：

```bash
# 仅构建 BallCalibration
cmake -B build -DRXS_BUILD_CYLINDER=OFF -DRXS_BUILD_FILEGET=OFF

# 仅构建 Cylinder
cmake -B build -DRXS_BUILD_BALLCALIBRATION=OFF -DRXS_BUILD_FILEGET=OFF
```

| CMake 选项 | 默认值 | 说明 |
|------------|--------|------|
| `RXS_BUILD_BALLCALIBRATION` | `ON` | 构建 BallCalibration DLL 及测试程序 |
| `RXS_BUILD_CYLINDER` | `ON` | 构建 Cylinder 检测程序 |
| `RXS_BUILD_FILEGET` | `ON` | 构建 fileGet 工具 |

### Visual Studio 构建

各子模块均提供独立的 `.sln` / `.vcxproj` 文件，可直接在 Visual Studio 中打开构建（需手动配置 PCL 和 Eigen 的包含路径与库路径）。

---

## 依赖总览

| 依赖 | BallCalibration | Cylinder | fileGet | 来源 |
|------|:-:|:-:|:-:|------|
| PCL | 必需 | 必需 | -- | 外部安装 |
| Eigen3 | 必需 | 必需 (via PCL) | -- | 外部安装 |
| OpenMP | 可选 | -- | -- | 编译器内置 |
| czxDependence (`czxTool.h`) | 必需 | 必需 | -- | **外部仓库** (见上方说明) |
| czxDependence (`czxTool_std.h`) | -- | -- | 必需 | **外部仓库** |
| Windows SDK | 必需 | -- | 必需 | 系统 |

---

## 许可证

**Business Source License 1.1 (BSL 1.1)**

- **许可方**: 苏州锐新视科技有限公司 (Suzhou Ruixin Vision Technology Co., Ltd.)
- **变更日期 (Change Date)**: 2030-01-01
- **变更许可证 (Change License)**: GNU General Public License v2.0 (GPLv2)

变更日期之前，非生产用途（开发、测试、评估、学术研究）可免费使用；生产用途需联系许可方获取商业授权。

商业授权联系: rain3dmetrology@gmail.com
