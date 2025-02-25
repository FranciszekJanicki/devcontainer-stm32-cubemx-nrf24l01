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
        return std::bit_cast<CONFIG>(this->spi_device_.read_byte(std::to_underlying(RegAddress::CONFIG)));
    }

    void nRF24L01::set_config_register(CONFIG const config) const noexcept
    {}

    EN_AA nRF24L01::get_en_aa_register() const noexcept
    {
        return std::bit_cast<EN_AA>(this->spi_device_.read_byte(std::to_underlying(RegAddress::EN_AA)));
    }

    void nRF24L01::set_en_aa_register(EN_AA const en_aa) const noexcept
    {}

    EN_RXADDR nRF24L01::get_en_rxaddr_register() const noexcept
    {}

    void nRF24L01::set_en_rx_addr_register(EN_RXADDR const en_rxaddr) const noexcept
    {}

    SETUP_AW nRF24L01::get_setup_aw_register() const noexcept
    {}

    void nRF24L01::set_setup_aw_register(SETUP_AW const setup_aw) const noexcept
    {}

    SETUP_RETR nRF24L01::get_setup_retr_register() const noexcept
    {}

    void nRF24L01::set_setup_retr_register(SETUP_RETR const setup_retr) const noexcept
    {}

    RF_CH nRF24L01::get_rf_ch_register() const noexcept
    {}

    void nRF24L01::set_rf_ch_register(RF_CH const rf_ch) const noexcept
    {}

    RF_SETUP nRF24L01::get_rf_setup_register() const noexcept
    {}

    void nRF24L01::set_rf_setup_register(RF_SETUP const rf_setup) const noexcept
    {}

    STATUS nRF24L01::get_status_register() const noexcept
    {}

    OBSERVE_TX nRF24L01::get_observe_tx_register() const noexcept
    {}

    RPD nRF24L01::get_rpd_register() const noexcept
    {}

    RX_ADDR_P0 nRF24L01::get_rx_addr_p0_register() const noexcept
    {}

    void nRF24L01::set_rx_addr_p0_register(RX_ADDR_P0 const rx_addr_p0) const noexcept
    {}

    RX_ADDR_P1 nRF24L01::get_rx_addr_p1_register() const noexcept
    {}

    void nRF24L01::set_rx_addr_p1_register(RX_ADDR_P1 const rx_addr_p1) const noexcept
    {}

    RX_ADDR_P2 nRF24L01::get_rx_addr_p2_register() const noexcept
    {}

    void nRF24L01::set_rx_addr_p2_register(RX_ADDR_P2 const rx_addr_p2) const noexcept
    {}

    RX_ADDR_P3 nRF24L01::get_rx_addr_p3_register() const noexcept
    {}

    void nRF24L01::set_rx_addr_p4_register(RX_ADDR_P4 const rx_addr_p4) const noexcept
    {}

    void nRF24L01::set_rx_addr_p3_register(RX_ADDR_P3 const rx_addr_p3) const noexcept
    {}

    RX_ADDR_P4 nRF24L01::get_rx_addr_p4_register() const noexcept
    {}

    RX_ADDR_P5 nRF24L01::get_rx_addr_p5_register() const noexcept
    {}

    void nRF24L01::set_rx_addr_p5_register(RX_ADDR_P5 const rx_addr_p5) const noexcept
    {}

    TX_ADDR nRF24L01::get_tx_addr_register() const noexcept
    {}

    void nRF24L01::set_tx_addr_register(TX_ADDR const tx_addr) const noexcept
    {}

    RX_PW_P0 nRF24L01::get_rx_pw_p0_register() const noexcept
    {}

    void nRF24L01::set_rx_pw_p0_register(RX_PW_P0 const rx_pw_p0) const noexcept
    {}

    RX_PW_P1 nRF24L01::get_rx_pw_p1_register() const noexcept
    {}

    void nRF24L01::set_rx_pw_p1_register(RX_PW_P1 const rx_pw_p1) const noexcept
    {}

    RX_PW_P2 nRF24L01::get_rx_pw_p2_register() const noexcept
    {}

    void nRF24L01::set_rx_pw_p2_register(RX_PW_P2 const rx_pw_p2) const noexcept
    {}

    RX_PW_P3 nRF24L01::get_rx_pw_p3_register() const noexcept
    {}

    void nRF24L01::set_rx_pw_p3_register(RX_PW_P3 const rx_pw_p3) const noexcept
    {}

    RX_PW_P4 nRF24L01::get_rx_pw_p4_register() const noexcept
    {}

    void nRF24L01::set_rx_pw_p4_register(RX_PW_P4 const rx_pw_p4) const noexcept
    {}

    RX_PW_P5 nRF24L01::get_rx_pw_p5_register() const noexcept
    {}

    void nRF24L01::set_rx_pw_p5_register(RX_PW_P5 const rx_pw_p5) const noexcept
    {}

    FIFO_STATUS nRF24L01::get_fifo_status_register() const noexcept
    {}

    void nRF24L01::set_ack_pld_registers(ACK_PLD const& ack_pld) const noexcept
    {}

    void nRF24L01::set_tx_pld_registers(TX_PLD const& tx_pld) const noexcept
    {}

    RX_PLD nRF24L01::get_rx_pld_registers() const noexcept
    {}

    DYNPD nRF24L01::get_dynpd_register() const noexcept
    {}

    void nRF24L01::set_dynpd_register(DYNPD const dynpd) const noexcept
    {}

    FEATURE nRF24L01::get_feature_register() const noexcept
    {}

    void nRF24L01::set_feature_register(FEATURE const feature) const noexcept
    {}

}; // namespace nRF24L01