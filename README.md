# MCP23017 Component (scaffold)

This scaffold contains the MCP23017 component sources and an example plugin app.

Layout:
- `components/mcp23017/include/MCP23017.h` - public header
- `components/mcp23017/src/MCP23017.c` - implementation
- `examples/plugin/` - example ESP-IDF project that uses the component

Usage:
1. Copy the `components/mcp23017/` folder into your ESP-IDF project `components/` directory.
2. Copy `examples/plugin/` to a new project folder (or adapt your `CMakeLists.txt`).
3. From the example folder run `idf.py set-target <target>` and `idf.py build`.

Replace or adapt `CMakeLists.txt` and `idf_component.yml` as needed for your environment.

License: MIT (see LICENSE)

## Quick Usage Example

Example showing how to use the component in a simple app plugin:

```c
#include "MCP23017.h"
// ... other includes ...

void app_main(void)
{
	// Discover and attach devices on an existing I2C master bus
	mcp23017_attached_devices_t devices = {0};
	esp_err_t rc = mcp23017_auto_setup(&devices, true);
	if (rc != ESP_OK || devices.handle == NULL) {
		// handle error
		return;
	}

	// Configure port B as inputs with pull-ups and open-drain INT
	mcp23017_pin_cfg_t cfg = {
		.port = MCP_PORT_B,
		.mask = MCP_PORT_ALL,
		.pin_mode = MCP_PIN_INPUT,
		.pullup = MCP_PULLUP_ENABLE,
		.int_mode = MCP_INT_ANYEDGE,
		.int_polarity = MCP_INT_OPENDRAIN,
		.initial_level = 0x00,
		.flags = MCP_CFG_BATCH_WRITE,
	};
	(void)mcp23017_config_port(devices.handle, 0, &cfg);

	// Start plugin ISR worker (example plugin provides io_MCP23017_start())
	// io_MCP23017_start();

	// Use `mcp23017_port_read()` / `mcp23017_port_write()` as needed
}
```

Build the example with the ESP-IDF toolchain from `repo/examples/plugin/`.

