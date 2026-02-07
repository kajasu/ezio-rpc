#pragma once
#include "driver/uart.h"

// Start the Modbus RTU slave task (creates its own UART driver)
void start_modbus_slave_task();
