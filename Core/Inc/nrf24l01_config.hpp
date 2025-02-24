#ifndef NRF24L01_CONFIG_HPP
#define NRF24L01_CONFIG_HPP

#include "utility.hpp"
#include <array>
#include <cstdint>
#include <cstring>

namespace nRF24L01 {

    enum struct RegAddress : std::uint8_t {
        CONFIG = 0x00,
        EN_AA = 0x01,
        EN_RXADDR = 0x02,
        SETUP_AW = 0x03,
        SETUP_RETR = 0x04,
        RF_CH = 0x05,
        RF_SETUP = 0x06,
        STATUS = 0x07,
        OBSERVE_TX = 0x08,
        CD = 0x09,
        RX_ADDR_P0 = 0x0A,
        RX_ADDR_P1 = 0x0B,
        RX_ADDR_P2 = 0x0C,
        RX_ADDR_P3 = 0x0D,
        RX_ADDR_P4 = 0x0E,
        RX_ADDR_P5 = 0x0F,
        TX_ADDR = 0x10,
        RX_PW_P0 = 0x11,
        RX_PW_P1 = 0x12,
        RX_PW_P2 = 0x13,
        RX_PW_P3 = 0x14,
        RX_PW_P4 = 0x15,
        RX_PW_P5 = 0x16,
        FIFO_STATUS = 0x17,
        ACK_PLD,
        TX_PLD,
        RX_PLD,
        DYNPD = 0x1C,
        FEATURE = 0x1D,
    };

    enum struct AirDataRate : std::uint8_t {
        RATE_1MBPS,
        RATE_2MBPS,
    };

    enum struct OutputPower : std::uint8_t {
        PLUS_0DBM = 0b11,
        MINUS_6DBM = 0b10,
        MINUS_12DBM = 0b01,
        MINUS_18DBM = 0b00,
    };

    enum struct LNA : std::uint8_t {};

    enum struct RTControl : std::uint8_t {};

    enum struct Mode : std::uint8_t {
        POWER_DOWN,
        STANDBY_I,
        STANDBY_II,
        RX_MODE,
        TX_MODE,
    };

    enum struct MaskInt : std::uint8_t {
        IRQ_REFLECT_INT = 0x01,
        IRQ_FALLING_INT = 0x00,
    };

    enum struct RegAddress : std::uint8_t {};

    enum struct Command : std::uint8_t {
        RX_PAYLOAD = 0b01100001,
        TX_PAYLOAD = 0b10100000,
        FLUSH_TX = 0b11100001,
        FLUSH_RX = 0b11100010,
        REUSE_TX_PL = 0b11100011,
        ACTIVATE = 0b01010000,
        R_RX_PL_WID = 0b01100000,
        W_ACK_PAYLOAD_00 = 0b1011000,
        W_ACK_PAYLOAD_01 = 0b1011001,
        W_ACK_PAYLOAD_02 = 0b1011011,
        W_ACK_PAYLOAD_03 = 0b1011101,
        W_TX_PAYLOAD_NOACK = 0b1011000,
        NOP = 0b11111111,
    };

    template <std::size_t ADDRESS_SIZE, std::size_t PAYLOAD_SIZE>
    inline auto make_shock_burst_packet(std::uint8_t const preamble,
                                        std::array<std::uint8_t, ADDRESS_SIZE> const address,
                                        std::array<std::uint8_t, PAYLOAD_SIZE> const& payload,
                                        std::uint16_t const crc) noexcept
        -> std::array<std::uint8_t, sizeof(preamble) + ADDRESS_SIZE + PAYLOAD_SIZE + sizeof(crc)>
    {
        static_assert(3UL <= ADDRESS_SIZE << 5UL);
        static_assert(1UL <= PAYLOAD_SIZE <= 32UL);

        std::array<std::uint8_t, sizeof(preamble) + ADDRESS_SIZE + PAYLOAD_SIZE + sizeof(crc)> shock_burst_packet{};
        std::memcpy(shock_burst_packet.data(), &preamble, sizoef(preamble));
        std::memcpy(shock_burst_packet.data() + sizeof(preamble), address.data(), ADDRESS_SIZE);
        std::memcpy(shock_burst_packet.data() + sizeof(preamble) + ADDRESS_SIZE, payload.data(), PAYLOAD_SIZE);
        std::memcpy(shock_burst_packet.data() + sizeof(preamble) + ADDRESS_SIZE + PAYLOAD_SIZE, &crc, sizeof(crc));
        return shock_burst_packet;
    }

    template <std::size_t ADDRESS_SIZE, std::size_t PAYLOAD_SIZE>
    inline auto make_enhanced_shock_burst_packet(std::uint8_t const preamble,
                                                 std::uint8_t const control_field,
                                                 std::array<std::uint8_t, ADDRESS_SIZE> const address,
                                                 std::array<std::uint8_t, PAYLOAD_SIZE> const& payload,
                                                 std::uint16_t const crc) noexcept
        -> std::array<std::uint8_t,
                      sizeof(preamble) + sizeof(control_field) + ADDRESS_SIZE + PAYLOAD_SIZE + sizeof(crc)>
    {
        static_assert(3UL <= ADDRESS_SIZE << 5UL);
        static_assert(0UL <= PAYLOAD_SIZE <= 32UL);

        std::array<std::uint8_t, sizeof(preamble) + sizeof(control_field) + ADDRESS_SIZE + PAYLOAD_SIZE + sizeof(crc)>
            enhanced_shock_burst_packet{};
        std::memcpy(enhanced_shock_burst_packet.data(), &preamble, sizoef(preamble));
        std::memcpy(enhanced_shock_burst_packet.data() + sizeof(preamble), &control_field, sizeof(control_field));
        std::memcpy(enhanced_shock_burst_packet.data() + sizeof(preamble) + sizeof(control_field),
                    address.data(),
                    ADDRESS_SIZE);
        std::memcpy(enhanced_shock_burst_packet.data() + sizeof(preamble) + sizeof(control_field) + ADDRESS_SIZE,
                    payload.data(),
                    PAYLOAD_SIZE);
        std::memcpy(enhanced_shock_burst_packet.data() + sizeof(preamble) + sizeof(control_field) + ADDRESS_SIZE +
                        PAYLOAD_SIZE,
                    &crc,
                    sizeof(crc));
        return enhanced_shock_burst_packet;
    }

    [[nodiscard]] inline std::uint32_t frequency_mhz_to_rf_channel_frequency(std::uint32_t const frequency_mhz) noexcept
    {
        return frequency_mhz - 2400UL;
    }

}; // namespace nRF24L01

#endif // NRF24L01_CONFIG_HPP