#include "nrf24l01.hpp"
#include "nrf24l01_config.hpp"
#include "nrf24l01_registers.hpp"
#include "utility.hpp"
#include <algorithm>
#include <cstdio>
#include <utility>

using namespace Utility;

namespace nRF24L01 {

    nRF24L01::nRF24L01(SPIDevice&& spi_device, GPIO const chip_enable, Config const& config) noexcept :
        chip_enable_{chip_enable}, spi_device_{std::forward<SPIDevice>(spi_device)}
    {
        this->initialize(config);
    }

    nRF24L01::~nRF24L01() noexcept
    {
        this->deinitialize();
    }

    std::uint8_t nRF24L01::read_byte(std::uint8_t const reg_address) const noexcept
    {
        return this->spi_device_.read_byte(reg_address);
    }

    void nRF24L01::write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept
    {
        this->spi_device_.write_byte(reg_address, byte);
    }

    std::uint8_t nRF24L01::receive_byte() const noexcept
    {
        return this->spi_device_.receive_byte();
    }

    void nRF24L01::transmit_byte(std::uint8_t const byte) const noexcept
    {
        this->spi_device_.transmit_byte(byte);
    }

    void nRF24L01::initialize(Config const& config) noexcept
    {
        if (config.config.pwr_up) {
            this->set_config_register(config.config);
            this->set_en_aa_register(config.en_aa);
            this->set_en_rxaddr_register(config.en_rxaddr);
            this->set_setup_aw_register(config.setup_aw);
            this->set_setup_retr_register(config.setup_retr);
            this->set_rf_ch_register(config.rf_ch);
            this->set_rf_setup_register(config.rf_setup);
            this->set_dynpd_register(config.dynpd);
            this->set_feature_register(config.feature);

            this->send_flush_rx_command();
            this->send_flush_tx_command();
            this->power_up();

            this->initialized_ = true;
        }
    }

    void nRF24L01::deinitialize() noexcept
    {
        if (this->initialized_) {
            this->send_flush_rx_command();
            this->send_flush_tx_command();
            this->power_down();

            this->initialized_ = false;
        }
    }

    std::uint8_t nRF24L01::get_allowed_payload_size(PipeNum const pipe_num,
                                                    std::size_t const proposed_size) const noexcept
    {
        if (std::bit_cast<std::uint8_t>(this->get_dynpd_register()) & (1U << std::to_underlying(pipe_num))) {
            return std::min(proposed_size, 32UZ);
        } else {
            switch (pipe_num) {
                case PipeNum::PIPE_NUM0:
                    return this->get_rx_pw_p0_register().rx_pw_p0;
                case PipeNum::PIPE_NUM1:
                    return this->get_rx_pw_p1_register().rx_pw_p1;
                case PipeNum::PIPE_NUM2:
                    return this->get_rx_pw_p2_register().rx_pw_p2;
                case PipeNum::PIPE_NUM3:
                    return this->get_rx_pw_p3_register().rx_pw_p3;
                case PipeNum::PIPE_NUM4:
                    return this->get_rx_pw_p4_register().rx_pw_p4;
                case PipeNum::PIPE_NUM5:
                    return this->get_rx_pw_p5_register().rx_pw_p5;
            }
        }
    }

    void nRF24L01::reuse_tx() const noexcept
    {
        this->reset_chip_enable();

        auto config = this->get_config_register();
        config.mask_max_rt = true;
        this->set_config_register(config);

        this->send_reuse_tx_pl_command();
        this->set_chip_enable();
    }

    void nRF24L01::start_listening() const noexcept
    {
        this->power_up();

        auto config = this->get_config_register();
        config.prim_rx = true;
        this->set_config_register(config);

        auto status = this->get_status_register();
        status.rx_dr = true;
        status.tx_ds = true;
        status.max_rt = true;
        this->set_status_register(status);

        this->set_chip_enable();

        auto const rx_addr_p0 = this->get_rx_addr_p0_register();
        if (std::bit_width(rx_addr_p0.rx_addr_p0)) {
            this->open_reading_pipe(PipeNum::PIPE_NUM0, std::bit_cast<PipeAddress>(rx_addr_p0));
        } else {
            this->close_reading_pipe(PipeNum::PIPE_NUM0);
        }
    }

    void nRF24L01::stop_listening() const noexcept
    {
        this->reset_chip_enable();
        HAL_Delay(1UL);

        if (this->get_feature_register().en_ack_pay) {
            this->send_flush_tx_command();
        }

        auto config = this->get_config_register();
        config.prim_rx = false;
        this->set_config_register(config);

        this->close_reading_pipe(PipeNum::PIPE_NUM0);
    }

    bool nRF24L01::is_payload_available(PipeNum const pipe_num) const noexcept
    {
        return this->get_fifo_status_register().rx_empty != 0
                   ? this->get_status_register().rx_p_no == std::to_underlying(pipe_num)
                   : false;
    }

    void nRF24L01::open_writing_pipe(PipeAddress const pipe_address) const noexcept
    {
        this->set_tx_addr_register(TX_ADDR{.tx_addr = pipe_address.address});
    }

    void nRF24L01::open_reading_pipe(PipeNum const pipe_num, PipeAddress const pipe_address) const noexcept
    {
        auto en_rxaddr = std::bit_cast<std::uint8_t>(this->get_en_rxaddr_register());
        en_rxaddr |= (1U << std::to_underlying(pipe_num));
        this->set_en_rxaddr_register(std::bit_cast<EN_RXADDR>(en_rxaddr));

        switch (pipe_num) {
            case PipeNum::PIPE_NUM0:
                this->set_rx_addr_p0_register(RX_ADDR_P0{.rx_addr_p0 = pipe_address.address});
                break;
            case PipeNum::PIPE_NUM1:
                this->set_rx_addr_p1_register(RX_ADDR_P1{.rx_addr_p1 = pipe_address.address});
                break;
            case PipeNum::PIPE_NUM2:
                this->set_rx_addr_p2_register(RX_ADDR_P2{.rx_addr_p2 = pipe_address.address});
                this->update_common_address_bits(pipe_address);
                break;
            case PipeNum::PIPE_NUM3:
                this->set_rx_addr_p3_register(RX_ADDR_P3{.rx_addr_p3 = pipe_address.address});
                this->update_common_address_bits(pipe_address);
                break;
            case PipeNum::PIPE_NUM4:
                this->set_rx_addr_p4_register(RX_ADDR_P4{.rx_addr_p4 = pipe_address.address});
                this->update_common_address_bits(pipe_address);
                break;
            case PipeNum::PIPE_NUM5:
                this->set_rx_addr_p5_register(RX_ADDR_P5{.rx_addr_p5 = pipe_address.address});
                this->update_common_address_bits(pipe_address);
                break;
        }
    }

    void nRF24L01::close_reading_pipe(PipeNum const pipe_num) const noexcept
    {
        auto en_rxaddr = std::bit_cast<std::uint8_t>(this->get_en_rxaddr_register());
        en_rxaddr &= ~(1U << std::to_underlying(pipe_num));
        this->set_en_rxaddr_register(std::bit_cast<EN_RXADDR>(en_rxaddr));
    }

    void nRF24L01::close_writing_pipe() const noexcept
    {
        this->set_tx_addr_register(TX_ADDR{.tx_addr = 0UL});
    }

    void nRF24L01::update_common_address_bits(PipeAddress const pipe_address) const noexcept
    {
        std::uint8_t const common_bits_num = this->get_setup_aw_register().aw * 8U - 8U;
        std::uint8_t const unique_bits_num = 8U;

        std::uint64_t const common_bits = Utility::read_bits(pipe_address.address, common_bits_num, 0U);
        std::uint8_t const unique_bits = Utility::read_bits(pipe_address.address, 8U, common_bits_num);

        std::uint64_t rx_addr_p1 = Utility::read_bits(this->get_rx_addr_p1_register().rx_addr_p1, common_bits_num, 0U);
        Utility::write_bits(rx_addr_p1, common_bits, common_bits_num, 0U);
        this->set_rx_addr_p1_register(RX_ADDR_P1{.rx_addr_p1 = rx_addr_p1});
    }

    void nRF24L01::print_status() const noexcept
    {
        auto const status = this->get_status_register();
        printf("Status: MAX_RT = %d, RX_DR = %d, RX_P_NO = %d, TX_DS = %d, TX_FULL = %d\n\r",
               status.max_rt,
               status.rx_dr,
               status.rx_p_no,
               status.tx_ds,
               status.tx_full);
    }

    void nRF24L01::power_up() const noexcept
    {
        auto config = this->get_config_register();
        config.pwr_up = true;
        this->set_config_register(config);
        HAL_Delay(200);
    }

    void nRF24L01::power_down() const noexcept
    {
        auto config = this->get_config_register();
        config.pwr_up = false;
        this->set_config_register(config);
        this->reset_chip_enable();
    }

    void nRF24L01::set_chip_enable() const noexcept
    {
        gpio_write_pin(this->chip_enable_, GPIO_PIN_SET);
    }

    void nRF24L01::reset_chip_enable() const noexcept
    {
        gpio_write_pin(this->chip_enable_, GPIO_PIN_RESET);
    }

    CONFIG nRF24L01::get_config_register() const noexcept
    {
        return std::bit_cast<CONFIG>(this->read_byte(std::to_underlying(RA::CONFIG)));
    }

    void nRF24L01::set_config_register(CONFIG const config) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CONFIG), std::bit_cast<std::uint8_t>(config));
    }

    EN_AA nRF24L01::get_en_aa_register() const noexcept
    {
        return std::bit_cast<EN_AA>(this->read_byte(std::to_underlying(RA::EN_AA)));
    }

    void nRF24L01::set_en_aa_register(EN_AA const en_aa) const noexcept
    {
        this->write_byte(std::to_underlying(RA::EN_AA), std::bit_cast<std::uint8_t>(en_aa));
    }

    EN_RXADDR nRF24L01::get_en_rxaddr_register() const noexcept
    {
        return std::bit_cast<EN_RXADDR>(this->read_byte(std::to_underlying(RA::EN_RXADDR)));
    }

    void nRF24L01::set_en_rxaddr_register(EN_RXADDR const en_rxaddr) const noexcept
    {
        this->write_byte(std::to_underlying(RA::EN_RXADDR), std::bit_cast<std::uint8_t>(en_rxaddr));
    }

    SETUP_AW nRF24L01::get_setup_aw_register() const noexcept
    {
        return std::bit_cast<SETUP_AW>(this->read_byte(std::to_underlying(RA::SETUP_AW)));
    }

    void nRF24L01::set_setup_aw_register(SETUP_AW const setup_aw) const noexcept
    {
        this->write_byte(std::to_underlying(RA::SETUP_AW), std::bit_cast<std::uint8_t>(setup_aw));
    }

    SETUP_RETR nRF24L01::get_setup_retr_register() const noexcept
    {
        return std::bit_cast<SETUP_RETR>(this->read_byte(std::to_underlying(RA::SETUP_RETR)));
    }

    void nRF24L01::set_setup_retr_register(SETUP_RETR const setup_retr) const noexcept
    {
        this->write_byte(std::to_underlying(RA::SETUP_RETR), std::bit_cast<std::uint8_t>(setup_retr));
    }

    RF_CH nRF24L01::get_rf_ch_register() const noexcept
    {
        return std::bit_cast<RF_CH>(this->read_byte(std::to_underlying(RA::RF_CH)));
    }

    void nRF24L01::set_rf_ch_register(RF_CH const rf_ch) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RF_CH), std::bit_cast<std::uint8_t>(rf_ch));
    }

    RF_SETUP nRF24L01::get_rf_setup_register() const noexcept
    {
        return std::bit_cast<RF_SETUP>(this->read_byte(std::to_underlying(RA::RF_SETUP)));
    }

    void nRF24L01::set_rf_setup_register(RF_SETUP const rf_setup) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RF_SETUP), std::bit_cast<std::uint8_t>(rf_setup));
    }

    STATUS nRF24L01::get_status_register() const noexcept
    {
        return std::bit_cast<STATUS>(this->read_byte(std::to_underlying(RA::STATUS)));
    }

    void nRF24L01::set_status_register(STATUS const status) const noexcept
    {
        return this->write_byte(std::to_underlying(RA::STATUS), std::bit_cast<std::uint8_t>(status));
    }

    OBSERVE_TX nRF24L01::get_observe_tx_register() const noexcept
    {
        return std::bit_cast<OBSERVE_TX>(this->read_byte(std::to_underlying(RA::OBSERVE_TX)));
    }

    RPD nRF24L01::get_rpd_register() const noexcept
    {
        return std::bit_cast<RPD>(this->read_byte(std::to_underlying(RA::RPD)));
    }

    RX_ADDR_P0 nRF24L01::get_rx_addr_p0_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P0>(this->read_bytes<sizeof(RX_ADDR_P0)>(std::to_underlying(RA::RX_ADDR_P0)));
    }

    void nRF24L01::set_rx_addr_p0_register(RX_ADDR_P0 const rx_addr_p0) const noexcept
    {
        this->write_bytes(std::to_underlying(RA::RX_ADDR_P0),
                          std::bit_cast<std::array<std::uint8_t, sizeof(RX_ADDR_P0)>>(rx_addr_p0));
    }

    RX_ADDR_P1 nRF24L01::get_rx_addr_p1_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P1>(this->read_bytes<sizeof(RX_ADDR_P1)>(std::to_underlying(RA::RX_ADDR_P1)));
    }

    void nRF24L01::set_rx_addr_p1_register(RX_ADDR_P1 const rx_addr_p1) const noexcept
    {
        this->write_bytes(std::to_underlying(RA::RX_ADDR_P1),
                          std::bit_cast<std::array<std::uint8_t, sizeof(RX_ADDR_P1)>>(rx_addr_p1));
    }

    RX_ADDR_P2 nRF24L01::get_rx_addr_p2_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P2>(this->read_byte(std::to_underlying(RA::RX_ADDR_P2)));
    }

    void nRF24L01::set_rx_addr_p2_register(RX_ADDR_P2 const rx_addr_p2) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_ADDR_P2), std::bit_cast<std::uint8_t>(rx_addr_p2));
    }

    RX_ADDR_P3 nRF24L01::get_rx_addr_p3_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P3>(this->read_byte(std::to_underlying(RA::RX_ADDR_P3)));
    }

    void nRF24L01::set_rx_addr_p3_register(RX_ADDR_P3 const rx_addr_p3) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_ADDR_P3), std::bit_cast<std::uint8_t>(rx_addr_p3));
    }

    RX_ADDR_P4 nRF24L01::get_rx_addr_p4_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P4>(this->read_byte(std::to_underlying(RA::RX_ADDR_P4)));
    }

    void nRF24L01::set_rx_addr_p4_register(RX_ADDR_P4 const rx_addr_p4) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_ADDR_P4), std::bit_cast<std::uint8_t>(rx_addr_p4));
    }

    RX_ADDR_P5 nRF24L01::get_rx_addr_p5_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P5>(this->read_byte(std::to_underlying(RA::RX_ADDR_P5)));
    }

    void nRF24L01::set_rx_addr_p5_register(RX_ADDR_P5 const rx_addr_p5) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_ADDR_P5), std::bit_cast<std::uint8_t>(rx_addr_p5));
    }

    TX_ADDR nRF24L01::get_tx_addr_register() const noexcept
    {
        return std::bit_cast<TX_ADDR>(this->read_bytes<sizeof(TX_ADDR)>(std::to_underlying(RA::TX_ADDR)));
    }

    void nRF24L01::set_tx_addr_register(TX_ADDR const tx_addr) const noexcept
    {
        this->write_bytes(std::to_underlying(RA::TX_ADDR),
                          std::bit_cast<std::array<std::uint8_t, sizeof(TX_ADDR)>>(tx_addr));
    }

    RX_PW_P0 nRF24L01::get_rx_pw_p0_register() const noexcept
    {
        return std::bit_cast<RX_PW_P0>(this->read_byte(std::to_underlying(RA::RX_PW_P0)));
    }

    void nRF24L01::set_rx_pw_p0_register(RX_PW_P0 const rx_pw_p0) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_PW_P0), std::bit_cast<std::uint8_t>(rx_pw_p0));
    }

    RX_PW_P1 nRF24L01::get_rx_pw_p1_register() const noexcept
    {
        return std::bit_cast<RX_PW_P1>(this->read_byte(std::to_underlying(RA::RX_PW_P1)));
    }

    void nRF24L01::set_rx_pw_p1_register(RX_PW_P1 const rx_pw_p1) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_PW_P1), std::bit_cast<std::uint8_t>(rx_pw_p1));
    }

    RX_PW_P2 nRF24L01::get_rx_pw_p2_register() const noexcept
    {
        return std::bit_cast<RX_PW_P2>(this->read_byte(std::to_underlying(RA::RX_PW_P2)));
    }

    void nRF24L01::set_rx_pw_p2_register(RX_PW_P2 const rx_pw_p2) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_PW_P2), std::bit_cast<std::uint8_t>(rx_pw_p2));
    }

    RX_PW_P3 nRF24L01::get_rx_pw_p3_register() const noexcept
    {
        return std::bit_cast<RX_PW_P3>(this->read_byte(std::to_underlying(RA::RX_PW_P3)));
    }

    void nRF24L01::set_rx_pw_p3_register(RX_PW_P3 const rx_pw_p3) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_PW_P3), std::bit_cast<std::uint8_t>(rx_pw_p3));
    }

    RX_PW_P4 nRF24L01::get_rx_pw_p4_register() const noexcept
    {
        return std::bit_cast<RX_PW_P4>(this->read_byte(std::to_underlying(RA::RX_PW_P4)));
    }

    void nRF24L01::set_rx_pw_p4_register(RX_PW_P4 const rx_pw_p4) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_PW_P4), std::bit_cast<std::uint8_t>(rx_pw_p4));
    }

    RX_PW_P5 nRF24L01::get_rx_pw_p5_register() const noexcept
    {
        return std::bit_cast<RX_PW_P5>(this->read_byte(std::to_underlying(RA::RX_PW_P5)));
    }

    void nRF24L01::set_rx_pw_p5_register(RX_PW_P5 const rx_pw_p5) const noexcept
    {
        this->write_byte(std::to_underlying(RA::RX_PW_P5), std::bit_cast<std::uint8_t>(rx_pw_p5));
    }

    FIFO_STATUS nRF24L01::get_fifo_status_register() const noexcept
    {
        return std::bit_cast<FIFO_STATUS>(this->read_byte(std::to_underlying(RA::FIFO_STATUS)));
    }

    void nRF24L01::set_ack_pld_registers(ACK_PLD const& ack_pld) const noexcept
    {
        this->write_bytes(std::to_underlying(RA::ACK_PLD),
                          std::bit_cast<std::array<std::uint8_t, sizeof(ACK_PLD)>>(ack_pld));
    }

    void nRF24L01::set_tx_pld_registers(TX_PLD const& tx_pld) const noexcept
    {
        this->write_bytes(std::to_underlying(RA::TX_PLD),
                          std::bit_cast<std::array<std::uint8_t, sizeof(TX_PLD)>>(tx_pld));
    }

    RX_PLD nRF24L01::get_rx_pld_registers() const noexcept
    {
        return std::bit_cast<RX_PLD>(this->read_bytes<sizeof(RX_PLD)>(std::to_underlying(RA::RX_PLD)));
    }

    DYNPD nRF24L01::get_dynpd_register() const noexcept
    {
        return std::bit_cast<DYNPD>(this->read_byte(std::to_underlying(RA::DYNPD)));
    }

    void nRF24L01::set_dynpd_register(DYNPD const dynpd) const noexcept
    {
        this->write_byte(std::to_underlying(RA::DYNPD), std::bit_cast<std::uint8_t>(dynpd));
    }

    FEATURE nRF24L01::get_feature_register() const noexcept
    {
        return std::bit_cast<FEATURE>(this->read_byte(std::to_underlying(RA::FEATURE)));
    }

    void nRF24L01::set_feature_register(FEATURE const feature) const noexcept
    {
        this->write_byte(std::to_underlying(RA::FEATURE), std::bit_cast<std::uint8_t>(feature));
    }

    void nRF24L01::send_rx_payload_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::RX_PAYLOAD));
    }

    void nRF24L01::send_tx_payload_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::TX_PAYLOAD));
    }

    void nRF24L01::send_flush_tx_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::FLUSH_TX));
    }

    void nRF24L01::send_flush_rx_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::FLUSH_RX));
    }

    void nRF24L01::send_reuse_tx_pl_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::REUSE_TX_PL));
    }

    void nRF24L01::send_activate_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::ACTIVATE));
    }

    void nRF24L01::send_r_rx_pl_wid_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::R_RX_PL_WID));
    }

    void nRF24L01::send_w_ack_payload_p0_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_P0));
    }

    void nRF24L01::send_w_ack_payload_p1_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_P1));
    }

    void nRF24L01::send_w_ack_payload_p2_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_P2));
    }

    void nRF24L01::send_w_ack_payload_p3_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_P3));
    }

    void nRF24L01::send_w_ack_payload_p4_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_P4));
    }

    void nRF24L01::send_w_ack_payload_p5_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_P5));
    }

    void nRF24L01::send_w_tx_payload_noack_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::W_TX_PAYLOAD_NOACK));
    }

    void nRF24L01::send_nop_command() const noexcept
    {
        this->transmit_byte(std::to_underlying(CMD::NOP));
    }

}; // namespace nRF24L01