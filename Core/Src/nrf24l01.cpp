#include "nrf24l01.hpp"
#include "nrf24l01_config.hpp"
#include "nrf24l01_register.hpp"

namespace nRF24L01 {

    nRF24L01::nRF24L01(SPIDevice&& spi_device,
                       CONFIG const config,
                       EN_AA const en_aa,
                       EN_RXADDR const en_rxaddr,
                       SETUP_AW const setup_aw,
                       SETUP_RETR const setup_retr,
                       RF_CH const rf_ch,
                       RF_SETUP const rf_setup,
                       DYNPD const dynpd,
                       FEATURE const feature) noexcept :
        spi_device_{std::forward<SPIDevice>(spi_device)}
    {
        this->initialize(config, en_aa, en_rxaddr, setup_aw, setup_retr, rf_ch, rf_setup, dynpd, feature);
    }

    nRF24L01::~nRF24L01() noexcept
    {}

    void nRF24L01::initialize(CONFIG const config,
                              EN_AA const en_aa,
                              EN_RXADDR const en_rxaddr,
                              SETUP_AW const setup_aw,
                              SETUP_RETR const setup_retr,
                              RF_CH const rf_ch,
                              RF_SETUP const rf_setup,
                              DYNPD const dynpd,
                              FEATURE const feature) noexcept
    {
        if (1) {
            this->set_config_register(config);
            this->set_en_aa_register(en_aa);
            this->set_en_rx_addr_register(en_rxaddr);
            this->set_setup_aw_register(setup_aw);
            this->set_setup_retr_register(setup_retr);
            this->set_rf_ch_register(rf_ch);
            this->set_rf_setup_register(rf_setup);
            this->set_dynpd_register(dynpd);
            this->set_feature_register(feature);
            this->initialized_ = true;
        }
    }

    void nRF24L01::deinitialize() noexcept
    {
        if (1) {
            this->initialized_ = false;
        }
    }

    CONFIG nRF24L01::get_config_register() const noexcept
    {
        return std::bit_cast<CONFIG>(this->spi_device_.read_byte(std::to_underlying(RA::CONFIG)));
    }

    void nRF24L01::set_config_register(CONFIG const config) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::CONFIG), std::bit_cast<std::uint8_t>(config));
    }

    EN_AA nRF24L01::get_en_aa_register() const noexcept
    {
        return std::bit_cast<EN_AA>(this->spi_device_.read_byte(std::to_underlying(RA::EN_AA)));
    }

    void nRF24L01::set_en_aa_register(EN_AA const en_aa) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::EN_AA), std::bit_cast<std::uint8_t>(en_aa));
    }

    EN_RXADDR nRF24L01::get_en_rxaddr_register() const noexcept
    {
        return std::bit_cast<EN_RXADDR>(this->spi_device_.read_byte(std::to_underlying(RA::EN_RXADDR)));
    }

    void nRF24L01::set_en_rx_addr_register(EN_RXADDR const en_rxaddr) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::EN_RXADDR), std::bit_cast<std::uint8_t>(en_rxaddr));
    }

    SETUP_AW nRF24L01::get_setup_aw_register() const noexcept
    {
        return std::bit_cast<SETUP_AW>(this->spi_device_.read_byte(std::to_underlying(RA::SETUP_AW)));
    }

    void nRF24L01::set_setup_aw_register(SETUP_AW const setup_aw) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::SETUP_AW), std::bit_cast<std::uint8_t>(setup_aw));
    }

    SETUP_RETR nRF24L01::get_setup_retr_register() const noexcept
    {
        return std::bit_cast<SETUP_RETR>(this->spi_device_.read_byte(std::to_underlying(RA::SETUP_RETR)));
    }

    void nRF24L01::set_setup_retr_register(SETUP_RETR const setup_retr) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::SETUP_RETR), std::bit_cast<std::uint8_t>(setup_retr));
    }

    RF_CH nRF24L01::get_rf_ch_register() const noexcept
    {
        return std::bit_cast<RF_CH>(this->spi_device_.read_byte(std::to_underlying(RA::RF_CH)));
    }

    void nRF24L01::set_rf_ch_register(RF_CH const rf_ch) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RF_CH), std::bit_cast<std::uint8_t>(rf_ch));
    }

    RF_SETUP nRF24L01::get_rf_setup_register() const noexcept
    {
        return std::bit_cast<RF_SETUP>(this->spi_device_.read_byte(std::to_underlying(RA::RF_SETUP)));
    }

    void nRF24L01::set_rf_setup_register(RF_SETUP const rf_setup) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RF_SETUP), std::bit_cast<std::uint8_t>(rf_setup));
    }

    STATUS nRF24L01::get_status_register() const noexcept
    {
        return std::bit_cast<STATUS>(this->spi_device_.read_byte(std::to_underlying(RA::STATUS)));
    }

    OBSERVE_TX nRF24L01::get_observe_tx_register() const noexcept
    {
        return std::bit_cast<OBSERVE_TX>(this->spi_device_.read_byte(std::to_underlying(RA::OBSERVE_TX)));
    }

    RPD nRF24L01::get_rpd_register() const noexcept
    {
        return std::bit_cast<RPD>(this->spi_device_.read_byte(std::to_underlying(RA::RPD)));
    }

    RX_ADDR_P0 nRF24L01::get_rx_addr_p0_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P0>(
            this->spi_device_.read_bytes<sizeof(RX_ADDR_P0)>(std::to_underlying(RA::RX_ADDR_P0)));
    }

    void nRF24L01::set_rx_addr_p0_register(RX_ADDR_P0 const rx_addr_p0) const noexcept
    {
        this->spi_device_.write_bytes(std::to_underlying(RA::RX_ADDR_P0),
                                      std::bit_cast<std::array<std::uint8_t, sizeof(RX_ADDR_P0)>>(rx_addr_p0));
    }

    RX_ADDR_P1 nRF24L01::get_rx_addr_p1_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P1>(
            this->spi_device_.read_bytes<sizeof(RX_ADDR_P1)>(std::to_underlying(RA::RX_ADDR_P1)));
    }

    void nRF24L01::set_rx_addr_p1_register(RX_ADDR_P1 const rx_addr_p1) const noexcept
    {
        this->spi_device_.write_bytes(std::to_underlying(RA::RX_ADDR_P1),
                                      std::bit_cast<std::array<std::uint8_t, sizeof(RX_ADDR_P1)>>(rx_addr_p1));
    }

    RX_ADDR_P2 nRF24L01::get_rx_addr_p2_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P2>(this->spi_device_.read_byte(std::to_underlying(RA::RX_ADDR_P2)));
    }

    void nRF24L01::set_rx_addr_p2_register(RX_ADDR_P2 const rx_addr_p2) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_ADDR_P2), std::bit_cast<std::uint8_t>(rx_addr_p2));
    }

    RX_ADDR_P3 nRF24L01::get_rx_addr_p3_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P3>(this->spi_device_.read_byte(std::to_underlying(RA::RX_ADDR_P3)));
    }

    void nRF24L01::set_rx_addr_p3_register(RX_ADDR_P3 const rx_addr_p3) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_ADDR_P3), std::bit_cast<std::uint8_t>(rx_addr_p3));
    }

    RX_ADDR_P4 nRF24L01::get_rx_addr_p4_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P4>(this->spi_device_.read_byte(std::to_underlying(RA::RX_ADDR_P4)));
    }

    void nRF24L01::set_rx_addr_p4_register(RX_ADDR_P4 const rx_addr_p4) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_ADDR_P4), std::bit_cast<std::uint8_t>(rx_addr_p4));
    }

    RX_ADDR_P5 nRF24L01::get_rx_addr_p5_register() const noexcept
    {
        return std::bit_cast<RX_ADDR_P5>(this->spi_device_.read_byte(std::to_underlying(RA::RX_ADDR_P5)));
    }

    void nRF24L01::set_rx_addr_p5_register(RX_ADDR_P5 const rx_addr_p5) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_ADDR_P5), std::bit_cast<std::uint8_t>(rx_addr_p5));
    }

    TX_ADDR nRF24L01::get_tx_addr_register() const noexcept
    {
        return std::bit_cast<TX_ADDR>(this->spi_device_.read_bytes<sizeof(TX_ADDR)>(std::to_underlying(RA::TX_ADDR)));
    }

    void nRF24L01::set_tx_addr_register(TX_ADDR const tx_addr) const noexcept
    {
        this->spi_device_.write_bytes(std::to_underlying(RA::TX_ADDR),
                                      std::bit_cast<std::array<std::uint8_t, sizeof(TX_ADDR)>>(tx_addr));
    }

    RX_PW_P0 nRF24L01::get_rx_pw_p0_register() const noexcept
    {
        return std::bit_cast<RX_PW_P0>(this->spi_device_.read_byte(std::to_underlying(RA::RX_PW_P0)));
    }

    void nRF24L01::set_rx_pw_p0_register(RX_PW_P0 const rx_pw_p0) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_PW_P0), std::bit_cast<std::uint8_t>(rx_pw_p0));
    }

    RX_PW_P1 nRF24L01::get_rx_pw_p1_register() const noexcept
    {
        return std::bit_cast<RX_PW_P1>(this->spi_device_.read_byte(std::to_underlying(RA::RX_PW_P1)));
    }

    void nRF24L01::set_rx_pw_p1_register(RX_PW_P1 const rx_pw_p1) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_PW_P1), std::bit_cast<std::uint8_t>(rx_pw_p1));
    }

    RX_PW_P2 nRF24L01::get_rx_pw_p2_register() const noexcept
    {
        return std::bit_cast<RX_PW_P2>(this->spi_device_.read_byte(std::to_underlying(RA::RX_PW_P2)));
    }

    void nRF24L01::set_rx_pw_p2_register(RX_PW_P2 const rx_pw_p2) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_PW_P2), std::bit_cast<std::uint8_t>(rx_pw_p2));
    }

    RX_PW_P3 nRF24L01::get_rx_pw_p3_register() const noexcept
    {
        return std::bit_cast<RX_PW_P3>(this->spi_device_.read_byte(std::to_underlying(RA::RX_PW_P3)));
    }

    void nRF24L01::set_rx_pw_p3_register(RX_PW_P3 const rx_pw_p3) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_PW_P3), std::bit_cast<std::uint8_t>(rx_pw_p3));
    }

    RX_PW_P4 nRF24L01::get_rx_pw_p4_register() const noexcept
    {
        return std::bit_cast<RX_PW_P4>(this->spi_device_.read_byte(std::to_underlying(RA::RX_PW_P4)));
    }

    void nRF24L01::set_rx_pw_p4_register(RX_PW_P4 const rx_pw_p4) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_PW_P4), std::bit_cast<std::uint8_t>(rx_pw_p4));
    }

    RX_PW_P5 nRF24L01::get_rx_pw_p5_register() const noexcept
    {
        return std::bit_cast<RX_PW_P5>(this->spi_device_.read_byte(std::to_underlying(RA::RX_PW_P5)));
    }

    void nRF24L01::set_rx_pw_p5_register(RX_PW_P5 const rx_pw_p5) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::RX_PW_P5), std::bit_cast<std::uint8_t>(rx_pw_p5));
    }

    FIFO_STATUS nRF24L01::get_fifo_status_register() const noexcept
    {
        return std::bit_cast<FIFO_STATUS>(this->spi_device_.read_byte(std::to_underlying(RA::FIFO_STATUS)));
    }

    void nRF24L01::set_ack_pld_registers(ACK_PLD const& ack_pld) const noexcept
    {
        this->spi_device_.write_bytes(std::to_underlying(RA::ACK_PLD),
                                      std::bit_cast<std::array<std::uint8_t, sizeof(ACK_PLD)>>(ack_pld));
    }

    void nRF24L01::set_tx_pld_registers(TX_PLD const& tx_pld) const noexcept
    {
        this->spi_device_.write_bytes(std::to_underlying(RA::TX_PLD),
                                      std::bit_cast<std::array<std::uint8_t, sizeof(TX_PLD)>>(tx_pld));
    }

    RX_PLD nRF24L01::get_rx_pld_registers() const noexcept
    {
        return std::bit_cast<RX_PLD>(this->spi_device_.read_bytes<sizeof(RX_PLD)>(std::to_underlying(RA::RX_PLD)));
    }

    DYNPD nRF24L01::get_dynpd_register() const noexcept
    {
        return std::bit_cast<DYNPD>(this->spi_device_.read_byte(std::to_underlying(RA::DYNPD)));
    }

    void nRF24L01::set_dynpd_register(DYNPD const dynpd) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::DYNPD), std::bit_cast<std::uint8_t>(dynpd));
    }

    FEATURE nRF24L01::get_feature_register() const noexcept
    {
        return std::bit_cast<FEATURE>(this->spi_device_.read_byte(std::to_underlying(RA::FEATURE)));
    }

    void nRF24L01::set_feature_register(FEATURE const feature) const noexcept
    {
        this->spi_device_.write_byte(std::to_underlying(RA::FEATURE), std::bit_cast<std::uint8_t>(feature));
    }

    void nRF24L01::send_rx_payload_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::RX_PAYLOAD));
    }

    void nRF24L01::send_tx_payload_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::TX_PAYLOAD));
    }

    void nRF24L01::send_flush_tx_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::FLUSH_TX));
    }

    void nRF24L01::send_flush_rx_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::FLUSH_RX));
    }

    void nRF24L01::send_reuse_tx_pl_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::REUSE_TX_PL));
    }

    void nRF24L01::send_activate_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::ACTIVATE));
    }

    void nRF24L01::send_r_rx_pl_wid_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::R_RX_PL_WID));
    }

    void nRF24L01::send_w_ack_payload_00_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_00));
    }

    void nRF24L01::send_w_ack_payload_01_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_01));
    }

    void nRF24L01::send_w_ack_payload_02_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_02));
    }

    void nRF24L01::send_w_ack_payload_03_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::W_ACK_PAYLOAD_03));
    }

    void nRF24L01::send_w_tx_payload_noack_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::W_TX_PAYLOAD_NOACK));
    }

    void nRF24L01::send_nop_command() const noexcept
    {
        this->spi_device_.transmit_byte(std::to_underlying(CMD::NOP));
    }

}; // namespace nRF24L01