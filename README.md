# VLA-on-agilex-test
采集 训练 部署
目录	说明	主要内容
📂 controller	机器人控制器封装	机械臂、底盘等设备的控制 class
📂 sensor	传感器封装	目前仅 RealSense 相机封装
📂 utils	工具函数库	辅助功能封装（如数学计算、日志等）
📂 data	数据采集模块	数据记录、处理的 class
📂 my_robot	机器人集成封装	完整机器人系统的组合 class
📂 policy	VLA 模型策略	Vision-Language-Action 模型相关代码
📂 scripts	实例化脚本	主要运行入口、测试代码
📂 third_party	第三方依赖	需要编译的外部库
📂 planner	路径规划模块	curobo 规划器封装 + 仿真机械臂代码
📂 example	示例代码	数据采集、模型部署等示例
📂 docs	文档索引	机器人相关文档链接
