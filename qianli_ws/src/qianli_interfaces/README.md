# qianli_interfaces

集中维护 QianLi 自定义 **msg / srv / action** 接口。

- 状态：✅ 骨架（可编译，暂无自定义接口）
- 目录：`msg/` `srv/` `action/`（.gitkeep 占位）
- 需要自定义接口时（如语义实体消息、任务 Action），将文件加入对应目录，并取消注释 [CMakeLists.txt](CMakeLists.txt) 中的 `rosidl_generate_interfaces` 段。
