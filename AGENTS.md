# Repository Guidelines

## Agent Operating Principles

This section overrides the repository-specific guidance below on conflict.

- **Think before coding.** State assumptions and material tradeoffs. If interpretations change scope or behavior, explain them and request clarification; do not choose silently. Challenge unnecessary complexity.
- **Keep the solution minimal.** Do not add unrequested features, configuration, abstractions, or impossible-case handling. Prefer the smallest implementation that meets the goal.
- **Make surgical changes.** Touch only relevant files and lines; preserve local style and leave unrelated code, comments, formatting, and dead code untouched. Remove only items your change makes unused. Every line must trace to the request.
- **Work toward verifiable goals.** Define success criteria before implementation. For multi-step work, state a plan with a check for each step and continue until build, test, or hardware validation confirms it.

## Project Structure & Module Organization

`FcSrc/` holds application, scheduling, config (`SysConfig.h`), and `AnoPTv8/` code. `DriversBsp/` holds board/sensor integrations; `DriversMcu/STM32F4xx/` holds active MCU drivers and dependencies. `DriversMcu/TM4C123/` is not part of the STM32 target. `ProjectSTM32F429/` holds the Keil project, debugger settings, and outputs. Pair new `.c` and `.h` files and add compiled sources to `ANO_LX_STM32F429.uvprojx`.

## Build, Test, and Development Commands

Use Keil MDK 5 with ARM Compiler 5.06 and the STM32F4 device pack specified by the project. Open `ProjectSTM32F429/ANO_LX_STM32F429.uvprojx`, select `Ano_LX`, and build. From a shell with `UV4.exe` on `PATH`:

```powershell
UV4.exe -b ProjectSTM32F429\ANO_LX_STM32F429.uvprojx -t Ano_LX
```

The build emits `ProjectSTM32F429/build/ANO_LX.axf`, `.hex`, and `ProjectSTM32F429/ANO-LX.bin`. The cleanup batch file removes build outputs and local `.bak` files.

## Coding Style & Naming Conventions

Compile as C99 and match the edited file. Existing code puts braces on their own line and mixes tabs with four-space indentation; do not reformat neighboring code. Use `Drv_*.c` for drivers, `LX_*.c` for flight-control modules, `AnoPTv8*.c` for protocol code, and matching headers/include guards. No formatter/linter is configured; inspect the diff.

## Testing Guidelines

There is no automated test framework or coverage target. Build without errors, then verify every change on the intended STM32F429 hardware. Exercise affected initialization, scheduler paths, peripherals, and protocol messages; record the board/configuration and observed result in the pull request.

## Commits and Pull Requests

History currently contains only `init`; use short, imperative, scoped subjects such as `driver: validate INA228 startup`. Keep commits focused. Pull requests should explain the behavior change, configuration flags or hardware assumptions, linked issue when available, successful build result, and relevant serial logs or captures.
