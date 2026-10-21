# 来源、复用与授权说明

## 团队代码

本仓库来自停下吃花生队伍U308的r3参赛交付包，团队自有代码及本次整理文档按根目录MIT许可证提供。来源记录不是第三方授权的替代物；具体外部资源遵循下面各自条件。

### 自动驾驶控制底座

`EmbodiedGuard-v14/embodiedguard/legacy/`复用队长提供的既有自动驾驶项目`bev_occupancy_pilot_20260911`。记录的源commit为`564517e8041767f904e13369d0a20741e6d59db3`；源文件路径、哈希及命名空间适配见`provenance_control.json`。这不是把全部底层算法宣称为此次从零开发。

源项目的`THIRD_PARTY_NOTICE.md`说明其某些运行流程调用独立OpenDriveVLA源码和基础模型。本公开目录不打包该上游仓库、权重或AutoVLA码本；不能把源项目的外部运行依赖误认为本目录全部包含的代码。CARLA命名的适配器也不意味着附带CARLA二进制或训练模型。

### 状态处理

`embodiedguard/state.py`参考用户提供RAVAL提交材料中`observation_time.py`的历史采样/插值语义，形成新的严格观测接口。来源文件哈希、保留规则和改动见`provenance_state.json`。不分发RAVAL Docker镜像、模型或其他未选入材料。

## Moving AI Lab公开地图

目录：`sources/movingai-public-maps-20261013/`。

Contains information from Moving AI Lab 2D Grid Benchmarks, made available under the Open Data Commons Attribution License (ODC-By) v1.0.

- 数据及人工maze/room地图作者：Nathan Sturtevant / Moving AI Lab / HOG2。
- 官方数据页面：https://www.movingai.com/benchmarks/grids.html
- 许可全文：https://opendatacommons.org/licenses/by/1-0/
- 本仓库只保留三张参赛所用人工room/maze图及其场景文件；原始内容和署名文件保持原字节，下载URL和指纹见该目录`manifest.json`。
- 引用：N. Sturtevant, “Benchmarks for Grid-Based Pathfinding,” IEEE Transactions on Computational Intelligence and AI in Games, 4(2), 144–148, 2012。

地图不适用本仓库MIT许可证；展示缩放和运动实验假设由本项目说明。未分发游戏地图、其他补充数据集或其素材。2026-10-21已核对官方数据页的ODC-By声明。

## 外部依赖

| 项目 | 用途 | 分发方式 |
|---|---|---|
| Agnes Harness | 模型与MCP会话运行 | 外部安装；本机核查的官方源码LICENSE为Apache-2.0，不将AGH源码并入本仓库 |
| Agnes 3.0 Flash | 参赛真实会话决策模型 | 通过官方服务使用，遵循账号及服务条款，不附模型权重 |
| NumPy | 公开格网几何 | 通过pip安装，保留其上游许可 |
| pytest及构建工具 | 测试/安装 | 通过pip安装，保留各自上游许可 |

AGH：https://github.com/AgnesAI-Labs/agnes-harness 。本仓库不附Key、登录态、AGH home、node_modules或外部依赖二进制。

## 公开整理新增文件

`reports/agent-validation/20261011-task-contract-r1/task_policy.py`从本地历史测试材料补入，用于原`test_task_contract.py`的导入依赖；它不在r3提交ZIP中。哈希和身份单列于`provenance/submitted-files.json`。`tools/`为公开整理辅助脚本，不计入原参赛算法。
