#ifndef NRF24L01_HPP
#define NRF24L01_HPP

#include "spi_device.hpp"
#include "utility.hpp"

namespace nRF24L01 {

    struct nRF24L01 {
    public:
        using SPIDevice = Utility::SPIDevice;

        nRF24L01() noexcept = default;
        nRF24L01(SPIDevice&& spi_device) noexcept;

        nRF24L01(nRF24L01 const& other) = delete;
        nRF24L01(nRF24L01&& other) noexcept = default;

        nRF24L01& operator=(nRF24L01 const& other) = delete;
        nRF24L01& operator=(nRF24L01&& other) noexcept = default;

        ~nRF24L01() noexcept;

    private:
        SPIDevice spi_device_{};
    };

}; // namespace nRF24L01

#endif // NRF24L01_HPP