# Copilot / AI Agent Instructions — MCP23017 ESP-IDF component (concise)

Purpose: minimal, actionable guidance to modify the MCP23017 driver and example plugin.

Where sources live
- Component: `components/mcp23017/src/MCP23017.c` and header `components/mcp23017/include/MCP23017.h`.
- Example plugin: `main/io_MCP23017.c`; app entry in `main/main.c`.

Quick build
- From repo root:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Core patterns & API notes (current implementation)
- Auto-discovery: `mcp23017_auto_setup(out_devices, apply_defaults, user_bus_cfg)`
	- If `user_bus_cfg` is provided the function creates the bus and `owns_bus` will be true.
	- Otherwise it retries detecting existing master buses (I2C_NUM_0/1) and attaches devices at 0x20..0x27.
- Register cache: the driver mirrors registers 0x00..0x15 (16 bytes)
	- Cached when `populate_register_cache()` succeeds; cache used to avoid reads/writes for config registers.
	- Use `mcp23017_sync_registers()` to refresh and `mcp23017_invalidate_register_cache()` to mark invalid.
- Batched config: `mcp23017_config_port(..., cfg)` supports `cfg->flags & MCP_CFG_BATCH_WRITE`
	- When set, driver attempts to modify a local cache and write registers 0x00..0x15 in a single sequential I2C transaction.
	- On failure it falls back to per-register RMW using `mcp23017_reg_read8()`/`mcp23017_reg_write8()`.
- I2C bus serialization: a bus registry provides a per-bus mutex (see `bus_registry_*`, `bus_lock()`/`bus_unlock()`) so multi-register transactions are serialized across handles sharing the same bus.
	- `mcp23017_delete()` releases registry entry and will delete the bus if `owns_bus` was set.
- Low-level helpers: `mcp23017_reg_read8()`, `mcp23017_reg_write8()`, and `mcp23017_reg_read_block()` perform the actual I2C transfers; prefer them for precise control.

Interrupt/ISR pattern
- ISR registry exposes `mcp23017_isr_register(cfg, worker_task)` and `mcp23017_isr_unregister(...)`.
	- ISR uses `vTaskNotifyGiveFromISR()` to notify worker tasks; workers should call `ulTaskNotifyTake()`.
	- Limits: supports up to 40 MCU GPIOs (`MCP_ISR_MAX_GPIO`) and up to 8 worker tasks per GPIO (`MCP_ISR_MAX_WORKERS`).
	- `install_isr_service` in `mcp23017_isr_cfg_t` lets callers request `gpio_install_isr_service()`.

Practical editing rules
- Preserve `esp_err_t` return semantics and `ESP_LOG*` usage.
- Add public headers to `components/mcp23017/include/` when exposing APIs and update component CMake accordingly.
- When changing batch RMW behavior, update tests and ensure the per-register fallback still works when cache is invalid.

Quick searches
- `rg "mcp23017_auto_setup|mcp23017_config_port|MCP_CFG_BATCH_WRITE|bus_registry_add|mcp23017_isr_register"`

If you want, I can merge these notes into the root agent guide or add short example snippets showing a worker task that allocates a `pool_msg_t` and posts to the dispatcher.

-- End
