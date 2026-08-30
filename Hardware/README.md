# Hardware

存放本项目的自定义硬件/外设驱动代码（如传感器、电机、LED、按键等）。

## 使用约定
- 与 `Core/`、`Drivers/`、`Middlewares/` 平级，位于工程根目录。
- 每个硬件模块通常为 `xxx.c` + `xxx.h`，放在这一层目录下。
- 头文件在工程中已加入包含路径：`../Hardware`（Keil 与 EIDE 均已配置）。
- 新增源文件后，请在 EIDE 工程（`.eide/eide.yml`）的 `virtualFolder` 中把对应的 `.c` 文件加入，或在 Keil 工程中把文件加入 `Hardware` 分组。
