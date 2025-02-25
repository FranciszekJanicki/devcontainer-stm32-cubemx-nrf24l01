#ifndef NRF24L01_HPP
#define NRF24L01_HPP

#include "nrf24l01_config.hpp"
#include "nrf24l01_register.hpp"
#include "spi_device.hpp"
#include "utility.hpp"

namespace nRF24L01 {

    struct nRF24L01 {
    public:
        using SPIDevice = Utility::SPIDevice;

        nRF24L01() noexcept = default;
        nRF24L01(SPIDevice&& spi_device,
                 CONFIG const config,
                 EN_AA const en_aa,
                 EN_RXADDR const en_rxaddr,
                 SETUP_AW const setup_aw,
                 SETUP_RETR const setup_retr,
                 RF_CH const rf_ch,
                 RF_SETUP const rf_setup,
                 DYNPD const dynpd,
                 FEATURE const feature) noexcept;

        nRF24L01(nRF24L01 const& other) = delete;
        nRF24L01(nRF24L01&& other) noexcept = default;

        nRF24L01& operator=(nRF24L01 const& other) = delete;
        nRF24L01& operator=(nRF24L01&& other) noexcept = default;

        ~nRF24L01() noexcept;

    private:
        void initialize(CONFIG const config,
                        EN_AA const en_aa,
                        EN_RXADDR const en_rxaddr,
                        SETUP_AW const setup_aw,
                        SETUP_RETR const setup_retr,
                        RF_CH const rf_ch,
                        RF_SETUP const rf_setup,
                        DYNPD const dynpd,
                        FEATURE const feature) noexcept;
        void deinitialize() noexcept;

        CONFIG get_config_register() const noexcept;
        void set_config_register(CONFIG const config) const noexcept;

        EN_AA get_en_aa_register() const noexcept;
        void set_en_aa_register(EN_AA const en_aa) const noexcept;

        EN_RXADDR get_en_rxaddr_register() const noexcept;
        void set_en_rx_addr_register(EN_RXADDR const en_rxaddr) const noexcept;

        SETUP_AW get_setup_aw_register() const noexcept;
        void set_setup_aw_register(SETUP_AW const setup_aw) const noexcept;

        SETUP_RETR get_setup_retr_register() const noexcept;
        void set_setup_retr_register(SETUP_RETR const setup_retr) const noexcept;

        RF_CH get_rf_ch_register() const noexcept;
        void set_rf_ch_register(RF_CH const rf_ch) const noexcept;

        RF_SETUP get_rf_setup_register() const noexcept;
        void set_rf_setup_register(RF_SETUP const rf_setup) const noexcept;

        STATUS get_status_register() const noexcept;

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

        bool initialized_{false};

        SPIDevice spi_device_{};
    };

}; // namespace nRF24L01

#endif // NRF24L01_HPP