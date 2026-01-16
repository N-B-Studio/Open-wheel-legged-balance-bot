# Open-wheel-legged-balance-bot

Fully 3D-printed, open-source 8-DOF wheel-legged balancing robot — a mixed repository
containing STM32 firmware, 3D-print models, and higher-level AI/ROS experiments.

**This README** gives a quick orientation for the mixed project, how to open and build
the STM32 firmware with STM32CubeIDE/CubeMX, and where to find the other components.

*Full Build guid availiable**
<img width="924" height="926" alt="image" src="https://github.com/user-attachments/assets/9723ac1a-3a28-47f2-9543-97611dfd1277" />

**3D print available **
<img width="869" height="736" alt="image" src="https://github.com/user-attachments/assets/20bd7444-01cf-497f-9a01-75412e5f8c37" />

**Contents**
- **STM32 firmware**: [STM32-H725-wheel-legged-bot](STM32-H725-wheel-legged-bot)
- **3D models / prints**: [3D-print](3D-print)
- **Higher-level AI experiments**: [Linux-AI](Linux-AI)
- **ROS2 experiments**: [ROS2](ROS2)

**Repository layout**
- `STM32-H725-wheel-legged-bot/` — STM32CubeIDE project and generated code. Key files:
	- `STM32-H725-wheel-legged-bot.ioc` — CubeMX configuration (open with CubeMX/STM32CubeIDE)
	- `Core/` — application sources (Inc/ and Src/)
	- `Debug/` — generated build artifacts and a makefile for command-line builds
- `3D-print/` — CAD and 3D-print files
- `Linux-AI/`, `ROS2/` — placeholders and higher-level components

Opening and building the STM32 firmware

1. Open the project in STM32CubeIDE (recommended):
	 - Launch STM32CubeIDE and choose File → Open Projects from File System, or open the
		 `STM32-H725-wheel-legged-bot.ioc` with CubeMX then open the generated CubeIDE project.
2. Build inside CubeIDE: use Project → Build All or the hammer toolbar button.
3. Command-line build (optional): a makefile exists under `STM32-H725-wheel-legged-bot/Debug`.
	 On Windows you need an environment with `make` available (MSYS2, MinGW, WSL, etc.).

	 Example (PowerShell):

```powershell
cd STM32-H725-wheel-legged-bot/Debug
make
```

Flashing and debugging
- Use the integrated debugger in STM32CubeIDE (requires ST-Link) or use STM32CubeProgrammer
	to flash the generated ELF/bin/hex files.

Notes and recommendations
- Keep CubeMX-generated files under version control only when you need to track changes.
	The `.ioc` file is included so the project can be regenerated.
- Use the provided `.gitignore` to avoid committing workspace and build artifacts.
- If you regenerate code from the `.ioc`, re-open the CubeIDE project before building.

Quick pointers
- 3D models: [3D-print](3D-print)
- Firmware sources: [STM32-H725-wheel-legged-bot/Core](STM32-H725-wheel-legged-bot/Core)
- Debug makefile: [STM32-H725-wheel-legged-bot/Debug/makefile](STM32-H725-wheel-legged-bot/Debug/makefile)

License
- See the top-level `LICENSE` file for repository license information.

If you'd like, I can also:
- add a short build script for Windows (`build.ps1`) that runs the makefile via MSYS2,
- or create a CONTRIBUTING.md with developer setup steps.
