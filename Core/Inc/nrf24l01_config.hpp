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
        RATE_1MBPS = 0x00,
        RATE_2MBPS = 0x01,
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

    enum struct MaskInterrupt : std::uint8_t {
        INT_DISABLED = 0x01,
        INT_ACTIVE_LOW = 0x00,
    };

    enum struct RTCtrl : std::uint8_t {
        PRX = 0x01,
        PTX = 0x00,
    };

    enum struct RTAddress : std::uint8_t {
        LEN_3BYTES = 0b01,
        LEN_4BYTES = 0b10,
        LEN_5BYTES = 0b11,
    };

    enum struct RetransmitDelay : std::uint8_t {
        WAIT_250US = 0b0000,
        WAIT_500US = 0b0001,
        WAIT_750US = 0b0010,
        WAIT_4000US = 0b1111,
    };

    enum struct RetransmitCount : std::uint8_t {
        DISABLED = 0b0000,
        RETRY_X1 = 0b0001,
        RETRY_X15 = 0b1111,
    };

    enum struct DataPipe : std::uint8_t {
        PIPE_NUM0 = 0b000,
        PIPE_NUM1 = 0b001,
        PIPE_NUM2 = 0b011,
        PIPE_NUM3 = 0b101,
        NOT_USED = 0b110,
        RX_FIFO_EMPTY = 0b111,
    };

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

    enum struct PayloadLen : std::uint8_t {
        LEN_0BYTES = 0b000000,
        LEN_32BYTES = 0b100000,
        DONT_CARE = 0b100001,
    };

    struct ControlField {
        uint8_t payload_length : 6;
        uint8_t pid : 2;
    } __attribute__((packed));

    static constexpr std::uint32_t TIME_STANDBY_2A_NS = 130000UL;

    inline std::uint32_t air_data_rate_to_bps(AirDataRate const air_data_rate) noexcept
    {
        switch (air_data_rate) {
            case AirDataRate::RATE_1MBPS:
                return 1000000UL;
            case AirDataRate::RATE_2MBPS:
                return 2000000UL;
            default:
                return 0UL;
        }
    }

    inline std::uint32_t air_data_rate_to_irq_time_ns(AirDataRate const air_data_rate) noexcept
    {
        switch (air_data_rate) {
            case AirDataRate::RATE_1MBPS:
                return 8200UL;
            case AirDataRate::RATE_2MBPS:
                return 6000UL;
            default:
                return 0UL;
        }
    }

    template <std::size_t ADDRESS_SIZE, std::size_t PAYLOAD_SIZE, std::size_t CRC_SIZE>
    inline auto make_shock_burst_packet(std::uint8_t const preamble,
                                        std::array<std::uint8_t, ADDRESS_SIZE> const address,
                                        std::array<std::uint8_t, PAYLOAD_SIZE> const& payload,
                                        std::array<std::uint8_t, CRC_SIZE> const crc) noexcept
        -> std::array<std::uint8_t, sizeof(preamble) + ADDRESS_SIZE + PAYLOAD_SIZE + sizeof(crc)>
    {
        static_assert(3UL <= ADDRESS_SIZE << 5UL);
        static_assert(1UL <= PAYLOAD_SIZE <= 32UL);

        std::array<std::uint8_t, sizeof(preamble) + ADDRESS_SIZE + PAYLOAD_SIZE + sizeof(crc)> shock_burst_packet{};
        std::memcpy(shock_burst_packet.data(), &preamble, sizeof(preamble));
        std::memcpy(shock_burst_packet.data() + sizeof(preamble), address.data(), ADDRESS_SIZE);
        std::memcpy(shock_burst_packet.data() + sizeof(preamble) + ADDRESS_SIZE, payload.data(), PAYLOAD_SIZE);
        std::memcpy(shock_burst_packet.data() + sizeof(preamble) + ADDRESS_SIZE + PAYLOAD_SIZE, crc.data(), crc.size());
        return shock_burst_packet;
    }

    template <std::size_t ADDRESS_SIZE, std::size_t PAYLOAD_SIZE, std::size_t CRC_SIZE>
    inline auto make_enhanced_shock_burst_packet(std::uint8_t const preamble,
                                                 std::uint8_t const control_field,
                                                 std::array<std::uint8_t, ADDRESS_SIZE> const address,
                                                 std::array<std::uint8_t, PAYLOAD_SIZE> const& payload,
                                                 std::array<std::uint8_t, CRC_SIZE> const crc) noexcept
        -> std::array<std::uint8_t, sizeof(preamble) + sizeof(control_field) + ADDRESS_SIZE + PAYLOAD_SIZE + CRC_SIZE>
    {
        static_assert(3UL <= ADDRESS_SIZE << 5UL);
        static_assert(0UL <= PAYLOAD_SIZE <= 32UL);

        std::array<std::uint8_t, sizeof(preamble) + sizeof(control_field) + ADDRESS_SIZE + PAYLOAD_SIZE + CRC_SIZE>
            enhanced_shock_burst_packet{};
        std::memcpy(enhanced_shock_burst_packet.data(), &preamble, sizeof(preamble));
        std::memcpy(enhanced_shock_burst_packet.data() + sizeof(preamble), &control_field, sizeof(control_field));
        std::memcpy(enhanced_shock_burst_packet.data() + sizeof(preamble) + sizeof(control_field),
                    address.data(),
                    address.size());
        std::memcpy(enhanced_shock_burst_packet.data() + sizeof(preamble) + sizeof(control_field) + address.size(),
                    payload.data(),
                    payload.size());
        std::memcpy(enhanced_shock_burst_packet.data() + sizeof(preamble) + sizeof(control_field) + address.size() +
                        payload.size(),
                    crc.data(),
                    crc.size());
        return enhanced_shock_burst_packet;
    }

    inline std::uint32_t frequency_mhz_to_rf_channel_frequency(std::uint32_t const frequency_mhz) noexcept
    {
        return frequency_mhz - 2400UL;
    }

    inline std::uint32_t get_time_on_air_ns(std::size_t const address_size,
                                            std::size_t const payload_size,
                                            std::size_t const crc_size,
                                            AirDataRate const air_data_rate) noexcept
    {
        return 1000000000UL * (8UL * (1UL + address_size + payload_size + crc_size) + 9UL) /
               air_data_rate_to_bps(air_data_rate);
    }

    inline std::uint32_t get_time_on_air_ack_ns(std::size_t const address_size,
                                                std::size_t const payload_size,
                                                std::size_t const crc_size,
                                                AirDataRate const air_data_rate) noexcept
    {
        return get_time_on_air_ns(address_size, payload_size, crc_size, air_data_rate);
    }

    inline std::uint32_t get_time_upload_ns(std::size_t const payload_size,
                                            std::uint32_t const spi_data_rate_bps) noexcept
    {
        return 1000000000UL * 8UL * payload_size / spi_data_rate_bps;
    }

    inline std::uint32_t get_time_enhcanced_shock_burst_cycle_ns(std::size_t const address_size,
                                                                 std::size_t const payload_size,
                                                                 std::size_t const crc_size,
                                                                 AirDataRate const air_data_rate,
                                                                 std::uint32_t const spi_data_rate_bps) noexcept
    {
        return get_time_upload_ns(payload_size, spi_data_rate_bps) + 2 * TIME_STANDBY_2A_NS +
               get_time_on_air_ns(address_size, payload_size, crc_size, air_data_rate) +
               get_time_on_air_ack_ns(address_size, payload_size, crc_size, air_data_rate) +
               air_data_rate_to_irq_time_ns(air_data_rate);
    }

}; // namespace nRF24L01

#endif // NRF24L01_CONFIG_HPP