# SSH — 连接 QianLi 虚拟机

> 当前开发 VM：Ubuntu 22.04.5 / ROS 2 Humble，别名 `qianli-humble`。旧 `qianli-vm` 对应保留的 Ubuntu 24.04/Jazzy VM。连接信息变更时同步更新 [ENVIRONMENT.md](ENVIRONMENT.md)。

## 1. 打开与连接当前 VM

Windows 桌面打开 **QianLi - Ubuntu22 Humble**，或在 VMware 打开：

```text
D:\VMs\QianLi-Ubuntu22-Humble\qianli-humble.vmx
```

然后在 Windows PowerShell 执行：

```powershell
ssh qianli-humble
ssh -o BatchMode=yes -o ConnectTimeout=10 qianli-humble "hostname; cat /etc/os-release; ls /opt/ros"
```

项目位于 `/home/ros/QianLi`。桌面、构建、训练与设备操作见 [VM_HUMBLE.md](VM_HUMBLE.md)。

## 2. 当前连接配置

| 项 | 值 |
|---|---|
| 地址 | `192.168.26.22`（本机 VMware NAT 安装记录，其他机器须核对） |
| SSH 端口 / 用户 | `22` / `ros` |
| 别名 | `qianli-humble`，Windows `C:\Users\sez18\.ssh\config` |
| 认证 | 已配置的 `id_ed25519` 密钥；SSH 密码登录关闭 |
| 客体本地登录 | 按用户要求移除本地密码并自动登录；与 SSH 密钥认证分别配置 |

如需重建 Windows 别名：

```sshconfig
Host qianli-humble
    HostName 192.168.26.22
    User ros
    IdentityFile C:/Users/sez18/.ssh/id_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 30
    ServerAliveCountMax 3
```

## 3. 连接排查

1. 确认上述 Humble VM 已开机，Ubuntu 已完成启动。
2. Windows 执行 `Test-NetConnection 192.168.26.22 -Port 22`。地址变化时先从 VM 的 `hostname -I` 核对，再更新别名。
3. VM 内执行 `systemctl is-enabled ssh`、`systemctl is-active ssh`；需要时执行 `sudo systemctl enable --now ssh`。
4. 执行 `ssh -o BatchMode=yes -o ConnectTimeout=10 qianli-humble "true"`。认证失败时核对指定私钥及 VM 的 `authorized_keys`；当前 VM 没有密码 SSH 回退。
5. 主机密钥变化时先在 VM 核实指纹；确认重装等原因后再更新相应 `known_hosts` 项。

连接健康不代表 GPU 或真机运动验收通过。Humble VM 使用虚拟显卡，CUDA 训练运行于 Windows 宿主机。

## 4. 旧 VM 与历史脚本

`qianli-vm` 对应旧 Ubuntu 24.04/Jazzy VM（此前地址 `192.168.26.128`，文件 `D:\Ubuntu-VM\ubuntu24-ros2.vmx`），仅用于资产核对和回退。

仓库中的 `scripts/tools/vm_ssh.sh`、`vm_check.sh` 仍连接该旧 VM。连接当前 Humble VM 使用本文的 `ssh qianli-humble`，不套用旧脚本的地址或账户说明。

## 5. 更新记录

- 2026-10-09：将当前入口统一为 Humble VM，标记旧 VM 脚本用途，移除旧账户密码说明。
- 2026-10-04：最初记录旧 Jazzy VM 的连接信息与检查流程。
