# Copilot / AI Agent Instructions — MCP23017 ESP-IDF scaffold

Purpose
- Short: help an AI agent be productive editing and extending this ESP-IDF MCP23017 driver + example plugin.

Big picture (quick)
- This is an ESP-IDF project/component implementing an MCP23017 I2C GPIO expander driver and a small example plugin in `main/`.
- Driver core: [main/src/MCP23017.c](main/src/MCP23017.c#L1-L120) (discovery, read/write, batch RMW, ISR registry).
- Example plugin: [main/io_MCP23017.c](main/io_MCP23017.c#L1-L200) (auto-setup, ISR worker using task notifications).
- Entry point: [main/main.c](main/main.c#L1-L40) calls `io_MCP23017_start()`.

Build / flash / debug
- Standard ESP-IDF flow: from project root run:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

- The workspace contains ESP-IDF terminals (e.g. "ESP-IDF Flash", "ESP-IDF Monitor"); use them if available.
- CMake registration lives in [main/CMakeLists.txt](main/CMakeLists.txt#L1-L20). Update `SRCS` / `INCLUDE_DIRS` there for new sources.

Project-specific patterns & conventions
- Component layout: public headers in `main/include/`, implementation in `main/src/` and plugin glue in `main/`.
- Use `mcp23017_auto_setup()` to discover and attach devices on an existing I2C master bus (prefers I2C_NUM_0 then I2C_NUM_1).
- Prefer `mcp23017_config_port(..., MCP_CFG_BATCH_WRITE)` when possible — driver tries a sequential block RMW to reduce transactions.
- Interrupt flow: driver exposes `mcp23017_isr_register()` / `mcp23017_isr_unregister()` and uses a global ISR registry that notifies worker tasks with `vTaskNotifyGiveFromISR`.
  - Worker tasks should call `ulTaskNotifyTake()` (see `io_MCP23017.c`).
- I2C layer: code uses `i2c_master_*` helpers (see `main/src/MCP23017.c`). Ensure any changes respect the existing bus ownership semantics: `mcp23017_delete()` will remove devices and optionally delete the bus when `owns_bus` is true.

Files to consult for common edits
- Discovery and defaults: [main/src/MCP23017.c](main/src/MCP23017.c#L120-L260)
- Batch RMW and register layout: [main/src/MCP23017.c](main/src/MCP23017.c#L400-L520)
- ISR registration / gpio handler: [main/src/MCP23017.c](main/src/MCP23017.c#L520-L760)
- Example usage and worker pattern: [main/io_MCP23017.c](main/io_MCP23017.c#L1-L120)

Quick search shortcuts (examples)
- Find all driver APIs: `rg "mcp23017_"` in repo root.
- Find ISR usage: `rg "mcp23017_isr_register|mcp23017_isr_unregister"`.

Behavioral notes / gotchas (discovered from code)
- `mcp23017_auto_setup()` waits and retries for an existing I2C master; the project assumes some other initialization may create the bus.
- Batch writes attempt a sequential write of registers 0x00..0x15; fall back to per-register RMW on failure — be careful changing register indices.
- GPIO ISR registry supports up to `MCP_ISR_MAX_WORKERS` workers per MCU GPIO and uses a mutex for registration; follow same pattern when adding alternate handlers.

When editing or adding features
- Update `main/CMakeLists.txt` for any new C sources or headers.
- If exposing public APIs, place headers in `main/include/` and update includes accordingly.
- Preserve `esp_err_t` return conventions and use `ESP_LOG*` macros for diagnostic messages.

If you need further details
- Ask for targeted extracts (e.g. show the batch-write register offsets, full ISR registry functions, or example runtime logs) and I will include snippets or add unit/usage tests.

— End
