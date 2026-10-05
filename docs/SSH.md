# SSH — 连接开发虚拟机（权威指南）

> 本文件是连接 QianLi 开发虚拟机（Ubuntu 24.04 + ROS 2 Jazzy）的**唯一权威入口**。
> 任何"连不上 / 找不到虚拟机"的问题，先按 §4 排查。
> 变更连接信息时，必须同步更新本文件与 [ENVIRONMENT.md](ENVIRONMENT.md) §2.2。

## 1. 快速连接（推荐）

在 **Windows 开发主机**（SUERZONG / sez18）上，SSH 别名已配置在
`C:\Users\sez18\.ssh\config`（Host `qianli-vm`），可直接免密登录：

```bash
ssh qianli-vm
```

或使用项目自带的一键脚本（在主机或虚拟机内均可运行）：

```bash
# Windows 主机（PowerShell / CMD）
bash scripts/tools/vm_ssh.sh

# 或直接执行命令（等价）
ssh qianli-vm
```

## 2. 连接信息

| 项 | 值 |
|---|---|
| 主机 | 192.168.26.128（VMware NAT，hostname `ros2-ubuntu`） |
| 端口 | 22 |
| 用户 | `ros` |
| 密码 | `ros1234`（密码登录可用，但**推荐免密**） |
| 免密 | 已授权公钥：`suerzong@outlook.com`（= 主机 `~/.ssh/id_ed25519`）、`ros2-vm` |
| 别名 | `qianli-vm`（配置于主机 `C:\Users\sez18\.ssh\config`，IdentityFile = `~/.ssh/id_ed25519`） |

> 免密登录实测通过（2026-10-04）：主机私钥 `id_ed25519` 与 VM `~/.ssh/authorized_keys`
> 中的 `suerzong@outlook.com` 匹配。

## 3. 不用别名时的手动连接

```bash
# 方式 A：指定私钥（推荐）
ssh -i C:/Users/sez18/.ssh/id_ed25519 ros@192.168.26.128

# 方式 B：密码
ssh ros@192.168.26.128     # 提示输入密码 ros1234
```

## 4. "连不上 / 找不到"排查清单

按顺序检查，遇到哪一步失败就停在那一行：

```bash
# ① 虚拟机是否开机？（VMware 中确认 "Ubuntu 24.04 ROS2 Jazzy" 在运行）
# ② 端口 22 是否可达（Windows 主机执行）
Test-NetConnection 192.168.26.128 -Port 22     # TcpTestSucceeded 应为 True

# ③ sshd 服务状态（VM 内执行，或经任意可用通道）
systemctl is-enabled ssh    # 应为 enabled（开机自启）
systemctl is-active ssh     # 应为 active（运行中）
# 若 inactive：sudo systemctl start ssh；若 disabled：sudo systemctl enable ssh

# ④ 免密是否可用（Windows 主机执行）
ssh -o BatchMode=yes -o ConnectTimeout=10 qianli-vm "echo OK"
# 若失败改用密码：ssh qianli-vm

# ⑤ 别名是否存在于主机配置
Get-Content "$env:USERPROFILE\.ssh\config" | Select-String qianli-vm
```

### 常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| `No route to host` / ping 不通 | 虚拟机未开机，或 VMware NAT 未启动 | 打开虚拟机，确认 VMware 网络服务运行 |
| `Connection refused` | sshd 未运行 | VM 内 `sudo systemctl start ssh` |
| `Permission denied (publickey,password)` | 密钥不匹配或密码错误 | 用 `-i` 指定正确私钥，或改用密码 `ros1234` |
| `Host key verification failed` | known_hosts 记录过期（VM 重装/重建） | `ssh-keygen -R 192.168.26.128` 后重连 |
| 主机上找不到 `~/.ssh/config` | 配置文件缺失 | 将 §2 的别名块写入 `C:\Users\sez18\.ssh\config` |

### 别名配置块（如需重建）

```sshconfig
# === QianLi dev VM ===
Host qianli-vm
    HostName 192.168.26.128
    User ros
    IdentityFile C:/Users/sez18/.ssh/id_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 30
    ServerAliveCountMax 3
```

## 5. 健康检查

```bash
# 一键健康检查：连通性 + sshd 状态 + 磁盘 + GPU + ROS
bash scripts/tools/vm_check.sh
```

## 6. 更新记录

- 2026-10-04：新建本文件，汇总连接信息、别名、排查清单；确认 sshd 已 `enabled`（开机自启）。
