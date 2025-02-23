#include "main.h"
#include "nrf24l01.hpp"
#include "spi_device.hpp"
#include "system_clock.h"
#include "usart.h"

int main()
{
    HAL_Init();
    SystemClock_Config();

    auto nrf24l01 = nRF24L01::nRF24L01{};

    while (true) {
    }

    return 0;
}
