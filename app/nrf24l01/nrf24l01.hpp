#ifndef NRF24L01_HPP
#define NRF24L01_HPP

#include "gpio.hpp"
#include "nrf24l01_config.hpp"
#include "nrf24l01_registers.hpp"
#include "spi_device.hpp"
#include "utility.hpp"
#include <optional>

namespace nRF24L01 {

    struct nRF24L01 {
    public:
        using SPIDevice = Utility::SPIDevice;
        using GPIO = Utility::GPIO;

        nRF24L01() noexcept = default;
        nRF24L01(SPIDevice&& spi_device, GPIO const chip_enable, Config const& config) noexcept;

        nRF24L01(nRF24L01 const& other) = delete;
        nRF24L01(nRF24L01&& other) noexcept = default;

        nRF24L01& operator=(nRF24L01 const& other) = delete;
        nRF24L01& operator=(nRF24L01&& other) noexcept = default;

        ~nRF24L01() noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> pipe_rx(PipeNum const pipe_num) const noexcept;

        template <std::size_t SIZE>
        void pipe_tx(PipeNum const pipe_num, std::array<std::uint8_t, SIZE> const& data) const noexcept;

        std::uint8_t get_allowed_payload_size(PipeNum const pipe_num, std::size_t const proposed_size) const noexcept;

        void start_listening() const noexcept;
        void stop_listening() const noexcept;

        bool is_payload_available(PipeNum const pipe_num) const noexcept;

        void open_writing_pipe(PipeAddress const pipe_address) const noexcept;
        void open_reading_pipe(PipeNum const pipe_num, PipeAddress const pipe_address) const noexcept;

        void close_reading_pipe(PipeNum const pipe_num) const noexcept;
        void close_writing_pipe() const noexcept;

        void update_common_address_bits(PipeAddress const pipe_address) const noexcept;

        void power_up() const noexcept;
        void power_down() const noexcept;

        void reuse_tx() const noexcept;

        void print_status() const noexcept;

    private:
        std::uint8_t read_byte(std::uint8_t const reg_address) const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read_bytes(std::uint8_t const reg_address) const noexcept;

        void write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept;

        template <std::size_t SIZE>
        void write_bytes(std::uint8_t const reg_address, std::array<std::uint8_t, SIZE> const& bytes) const noexcept;

        std::uint8_t receive_byte() const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> receive_bytes() const noexcept;

        void transmit_byte(std::uint8_t const byte) const noexcept;

        template <std::size_t SIZE>
        void transmit_bytes(std::array<std::uint8_t, SIZE> const& bytes) const noexcept;

        void initialize(Config const& config) noexcept;

        void deinitialize() noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> pipe_rx_payload(PipeNum const pipe_num) const noexcept;

        template <std::size_t SIZE>
        void pipe_tx_payload(PipeNum const pipe_num, std::array<std::uint8_t, SIZE> const& payload) const noexcept;

        void set_chip_enable() const noexcept;
        void reset_chip_enable() const noexcept;

        CONFIG get_config_register() const noexcept;
        void set_config_register(CONFIG const config) const noexcept;

        EN_AA get_en_aa_register() const noexcept;
        void set_en_aa_register(EN_AA const en_aa) const noexcept;

        EN_RXADDR get_en_rxaddr_register() const noexcept;
        void set_en_rxaddr_register(EN_RXADDR const en_rxaddr) const noexcept;

        SETUP_AW get_setup_aw_register() const noexcept;
        void set_setup_aw_register(SETUP_AW const setup_aw) const noexcept;

        SETUP_RETR get_setup_retr_register() const noexcept;
        void set_setup_retr_register(SETUP_RETR const setup_retr) const noexcept;

        RF_CH get_rf_ch_register() const noexcept;
        void set_rf_ch_register(RF_CH const rf_ch) const noexcept;

        RF_SETUP get_rf_setup_register() const noexcept;
        void set_rf_setup_register(RF_SETUP const rf_setup) const noexcept;

        STATUS get_status_register() const noexcept;
        void set_status_register(STATUS const status) const noexcept;

        OBSERVE_TX get_observe_tx_register() const noexcept;

        RPD get_rpd_register() const noexcept;

        RX_ADDR_P0 get_rx_addr_p0_register() const noexcept;
        void set_rx_addr_p0_register(RX_ADDR_P0 const rx_addr_p0) const noexcept;

        RX_ADDR_P1 get_rx_addr_p1_register() const noexcept;
        void set_rx_addr_p1_register(RX_ADDR_P1 const rx_addr_p1) const noexcept;

        RX_ADDR_P2 get_rx_addr_p2_register() const noexcept;
        void set_rx_addr_p2_register(RX_ADDR_P2 const rx_addr_p2) const noexcept;

        RX_ADDR_P3 get_rx_addr_p3_register() const noexcept;
        void set_rx_addr_p3_register(RX_ADDR_P3 const rx_addr_p3) const noexcept;

        RX_ADDR_P4 get_rx_addr_p4_register() const noexcept;
        void set_rx_addr_p4_register(RX_ADDR_P4 const rx_addr_p4) const noexcept;

        RX_ADDR_P5 get_rx_addr_p5_register() const noexcept;
        void set_rx_addr_p5_register(RX_ADDR_P5 const rx_addr_p5) const noexcept;

        TX_ADDR get_tx_addr_register() const noexcept;
        void set_tx_addr_register(TX_ADDR const tx_addr) const noexcept;

        RX_PW_P0 get_rx_pw_p0_register() const noexcept;
        void set_rx_pw_p0_register(RX_PW_P0 const rx_pw_p0) const noexcept;

        RX_PW_P1 get_rx_pw_p1_register() const noexcept;
        void set_rx_pw_p1_register(RX_PW_P1 const rx_pw_p1) const noexcept;

        RX_PW_P2 get_rx_pw_p2_register() const noexcept;
        void set_rx_pw_p2_register(RX_PW_P2 const rx_pw_p2) const noexcept;

        RX_PW_P3 get_rx_pw_p3_register() const noexcept;
        void set_rx_pw_p3_register(RX_PW_P3 const rx_pw_p3) const noexcept;

        RX_PW_P4 get_rx_pw_p4_register() const noexcept;
        void set_rx_pw_p4_register(RX_PW_P4 const rx_pw_p4) const noexcept;

        RX_PW_P5 get_rx_pw_p5_register() const noexcept;
        void set_rx_pw_p5_register(RX_PW_P5 const rx_pw_p5) const noexcept;

        FIFO_STATUS get_fifo_status_register() const noexcept;

        void set_ack_pld_registers(ACK_PLD const& ack_pld) const noexcept;

        void set_tx_pld_registers(TX_PLD const& tx_pld) const noexcept;

        RX_PLD get_rx_pld_registers() const noexcept;

        DYNPD get_dynpd_register() const noexcept;
        void set_dynpd_register(DYNPD const dynpd) const noexcept;

        FEATURE get_feature_register() const noexcept;
        void set_feature_register(FEATURE const feature) const noexcept;

        void send_rx_payload_command() const noexcept;
        void send_tx_payload_command() const noexcept;

        void send_flush_tx_command() const noexcept;
        void send_flush_rx_command() const noexcept;

        void send_reuse_tx_pl_command() const noexcept;

        void send_activate_command() const noexcept;

        void send_r_rx_pl_wid_command() const noexcept;

        void send_w_ack_payload_p0_command() const noexcept;
        void send_w_ack_payload_p1_command() const noexcept;
        void send_w_ack_payload_p2_command() const noexcept;
        void send_w_ack_payload_p3_command() const noexcept;
        void send_w_ack_payload_p4_command() const noexcept;
        void send_w_ack_payload_p5_command() const noexcept;

        void send_w_tx_payload_noack_command() const noexcept;

        void send_nop_command() const noexcept;

        bool initialized_{false};

        GPIO chip_enable_{};

        SPIDevice spi_device_{};
    };

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> nRF24L01::read_bytes(std::uint8_t const reg_address) const noexcept
    {
        return this->spi_device_.read_bytes<SIZE>(reg_address);
    }

    template <std::size_t SIZE>
    inline void nRF24L01::write_bytes(std::uint8_t const reg_address,
                                      std::array<std::uint8_t, SIZE> const& bytes) const noexcept
    {
        this->spi_device_.write_bytes(reg_address, bytes);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> nRF24L01::receive_bytes() const noexcept
    {
        return this->spi_device_.receive_bytes<SIZE>();
    }

    template <std::size_t SIZE>
    inline void nRF24L01::transmit_bytes(std::array<std::uint8_t, SIZE> const& bytes) const noexcept
    {
        return this->spi_device_.transmit_bytes(bytes);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> nRF24L01::pipe_rx(PipeNum const pipe_num) const noexcept
    {
        auto result = this->pipe_rx_payload<SIZE>(pipe_num);

        auto status = this->get_status_register();
        status.rx_dr = true;
        this->set_status_register(status);

        return result;
    }

    template <std::size_t SIZE>
    inline void nRF24L01::pipe_tx(PipeNum const pipe_num, std::array<std::uint8_t, SIZE> const& data) const noexcept
    {
        this->pipe_tx_payload(data);

        auto status = this->get_status_register();
        status.tx_ds = true;
        this->set_status_register(status);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> nRF24L01::pipe_rx_payload(PipeNum const pipe_num) const noexcept
    {
        this->send_rx_payload_command();

        auto const payload_size = this->get_allowed_payload_size(pipe_num, SIZE);
        auto payload = std::array<std::uint8_t, SIZE>{};
        this->receive_bytes(payload.data(), payload_size);
        return payload;
    }

    template <std::size_t SIZE>
    inline void nRF24L01::pipe_tx_payload(PipeNum const pipe_num,
                                          std::array<std::uint8_t, SIZE> const& payload) const noexcept
    {
        this->send_tx_payload_command();

        auto const payload_size = this->get_allowed_payload_size(pipe_num, SIZE);
        this->transmit_bytes(payload.data(), payload_size);
    }

}; // namespace nRF24L01

#endif // NRF24L01_HPP