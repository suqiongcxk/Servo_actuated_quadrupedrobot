# Servo-actuated quadruped robot

基于 STM32H723 主控、两块 STM32F103C8T6 从机和舵机的四足机器人固件。

## 工程目录

| 目录 | 用途 | Keil 工程 |
| --- | --- | --- |
| `F103C8T6` | 前腿从机，RS485 ID `0x01` | `MDK-ARM/F103C8T6.uvprojx` |
| `F103C8T6后` | 后腿从机，RS485 ID `0x02` | `MDK-ARM/F103C8T6.uvprojx` |
| `H723zhukong` | 主控：蓝牙、步态、逆运动学、RS485 | `MDK-ARM/H723zhukong.uvprojx` |

源码、驱动、第三方库、CubeMX `.ioc` 和 Keil 工程文件纳入版本管理。
编译输出、HEX、调试缓存与本机 IDE 状态不上传；驱动自带的 `.lib`、`.a` 库保留。
每块板请使用对应工程生成的固件。第三方代码的版权和许可说明保留在原目录中。

## 编译与动作

本机使用 Keil MDK，ARM Compiler 5.06 update 5。安装对应 STM32 设备包后打开相应 `.uvprojx` 编译。

主控蓝牙使用 USART6、9600 波特率。现有协议中的命令字段 `USART6_dma_buffer[1]`：

| 命令 | 动作 |
| --- | --- |
| 1 / 2 / 3 | 行走档位，目标高度 16 / 13 / 10 cm；零速、极小速度保持站立 |
| 4 | 坐下 |
| 5 / 6 | 使能 / 失能 |
| 7 | 左前腿招手 |
| 8 | 简单舞蹈 |
| 9 | 原地扭身 |

命令 7、8、9 按命令变化触发，连续发送相同命令不会无限重复动作。
动作幅度位于 `H723zhukong/Core/Inc/Dog_gait.h`；低速起停阈值位于 `remote_contol.h`。

## 上传到 GitHub

先安装 Git for Windows，并准备好目标仓库的完整 URL。脚本使用 Git Credential Manager 的登录，不保存访问令牌。

首次上传，在本目录打开 PowerShell：

```powershell
.\upload-github.ps1 -RepoUrl 'https://github.com/你的用户名/你的仓库名.git' -Message 'Initial firmware import'
```

也可以双击 `upload-github.bat`，按提示填写仓库地址和缺失的提交身份。
配置过 `origin` 后，日常更新只需双击脚本，或运行：

```powershell
.\upload-github.ps1 -Message 'Update robot actions'
```

若执行策略阻止 `.ps1`，使用 `.bat` 启动即可。只查看待提交变化、不上传：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\upload-github.ps1 -CheckOnly
```

脚本仅提交三个工程及本目录的说明、忽略规则和上传脚本；默认使用远程默认分支，空仓库使用 `main`。
目标仓库必须已创建。脚本先检查远程访问，再提交、拉取、合并远程历史并推送，不使用强制推送。
若远程已有同名文件导致合并冲突，脚本停止；解决冲突并完成合并提交后再运行。
这也适用于 GitHub 提示认证失败、分支保护或没有写入权限的情况。
