# Copilot / AI Agent Instructions — MCP23017 ESP-IDF project

Purpose
- Short: give an AI code agent the minimal, actionable knowledge to be productive modifying the MCP23017 driver and example plugin.

Big picture
- This repo contains an ESP-IDF component (MCP23017) and a small example app/plugin under `main/` that exercises it.
- Component sources live in `components/mcp23017/` (public headers in `components/mcp23017/include/`, impl in `components/mcp23017/src/`).
- Example plugin: `main/io_MCP23017.c` — uses `mcp23017_auto_setup()`, config helpers and the ISR/worker pattern.
- Entry point: `main/main.c` calls the plugin start function (see `io_MCP23017_start()` in `main/io_MCP23017.c`).

Build / flash / debug (quick)
- Typical flow from repo root:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```
- Use the workspace "ESP-IDF Flash" and "ESP-IDF Monitor" terminals when available.
- Add new sources by updating `main/CMakeLists.txt` (SRCS/INCLUDE_DIRS) or the component CMake under `components/mcp23017/`.

Key APIs & patterns (do these exactly)
- Discovery: use `mcp23017_auto_setup(&devices, true, &bus_cfg)` to probe/attach devices. It may retry waiting for an I2C master.
- Port config: prefer `mcp23017_config_port(..., MCP_CFG_BATCH_WRITE)` — driver attempts a sequential block RMW to cut down transactions.
- ISR/worker: register handlers with `mcp23017_isr_register()`; ISR notifies a worker task via `vTaskNotifyGiveFromISR`. Worker tasks should use `ulTaskNotifyTake()` to consume notifications (see `main/io_MCP23017.c`).
- I2C ownership: the driver can own/tear down the bus. Respect `owns_bus` semantics when deleting devices (`mcp23017_delete()`).

Where to look (quick links)
- Discovery, defaults, batch RMW, ISR registry: `components/mcp23017/src/MCP23017.c` (search within that file for `auto_setup`, `batch`, `ISR`).
- Example plugin/ISR worker: `main/io_MCP23017.c` — shows task-notify based worker and use of `mcp23017_reg_read8()` / `mcp23017_reg_read8()`.
- App registration: `main/CMakeLists.txt` controls which `main/` sources are compiled into the example.

Practical editing rules
- Preserve `esp_err_t` semantics and use `ESP_LOG*` macros for diagnostics.
- When adding public APIs: add headers to `components/mcp23017/include/` and include them from `main/` or other components.
- When changing register layouts or batch RMW ranges, validate that changes do not break the fallback per-register RMW path.

Search tips
- Find all driver APIs: `rg "mcp23017_"` from repo root.
- Find ISR registration: `rg "mcp23017_isr_register|mcp23017_isr_unregister"`

If you need more
- Ask for targeted extracts: full `MCP23017.c` ISR registry, batch-write offsets, or runtime logs from the monitor. I can also update `main/CMakeLists.txt` or add a small test harness on request.

Minimal changelog for this file
- Merged existing guidance with concrete repo paths and clarified component vs example layout.

-- End
