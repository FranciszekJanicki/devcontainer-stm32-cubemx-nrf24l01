#ifndef NRF24L01_REGISTER_HPP
#define NRF24L01_REGISTER_HPP

#include <cstdint>
#include <utility>

#define PACKED __attribute__((__packed__))

namespace nRF24L01 {

    struct CONFIG {
        std::uint8_t : 1;
        std::uint8_t mask_rx_dr : 1;
        std::uint8_t mask_tx_ds : 1;
        std::uint8_t mask_max_rt : 1;
        std::uint8_t en_crc : 1;
        std::uint8_t crco : 1;
        std::uint8_t pwr_up : 1;
        std::uint8_t prim_rx : 1;
    } PACKED;

    struct EN_AA {
        std::uint8_t : 2;
        std::uint8_t enaa_p5 : 1;
        std::uint8_t enaa_p4 : 1;
        std::uint8_t enaa_p3 : 1;
        std::uint8_t enaa_p2 : 1;
        std::uint8_t enaa_p1 : 1;
        std::uint8_t enaa_p0 : 1;
    } PACKED;

    struct EN_RXADDR {
        std::uint8_t : 2;
        std::uint8_t erx_p5 : 1;
        std::uint8_t erx_p4 : 1;
        std::uint8_t erx_p3 : 1;
        std::uint8_t erx_p2 : 1;
        std::uint8_t erx_p1 : 1;
        std::uint8_t erx_p0 : 1;
    } PACKED;

    struct SETUP_AW {
        std::uint8_t : 6;
        std::uint8_t aw : 2;
    } PACKED;

    struct SETUP_RETR {
        std::uint8_t ard : 4;
        std::uint8_t arc : 4;
    } PACKED;

    struct RF_CH {
        std::uint8_t : 1;
        std::uint8_t rf_ch : 7;
    } PACKED;

    struct RF_SETUP {
        std::uint8_t cont_wave : 1;
        std::uint8_t : 1;
        std::uint8_t rf_dr_low : 1;
        std::uint8_t pll_lock : 1;
        std::uint8_t rf_dr_high : 1;
        std::uint8_t rd_pwr : 2;
        std::uint8_t : 1;
    } PACKED;

    struct STATUS {
        std::uint8_t : 1;
        std::uint8_t rx_dr : 1;
        std::uint8_t tx_ds : 1;
        std::uint8_t max_rt : 1;
        std::uint8_t rx_p_no : 3;
        std::uint8_t tx_full : 1;
    } PACKED;

    struct OBSERVE_TX {
        std::uint8_t plos_cnt : 4;
        std::uint8_t arc_cnt : 4;
    } PACKED;

    struct RPD {
        std::uint8_t : 7;
        std::uint8_t rpd : 1;
    } PACKED;

    struct RX_ADDR_P0 {
        std::uint64_t rx_addr_p0 : 40;
    } PACKED;

    struct RX_ADDR_P1 {
        std::uint64_t rx_addr_p1 : 40;
    } PACKED;

    struct RX_ADDR_P2 {
        std::uint8_t rx_addr_p2 : 8;
    } PACKED;

    struct RX_ADDR_P3 {
        std::uint8_t rx_addr_p3 : 8;
    } PACKED;

    struct RX_ADDR_P4 {
        std::uint8_t rx_addr_p4 : 8;
    } PACKED;

    struct RX_ADDR_P5 {
        std::uint8_t rx_addr_p5 : 8;
    } PACKED;

    struct TX_ADDR {
        std::uint64_t tx_addr : 40;
    } PACKED;

    struct RX_PW_P0 {
        std::uint8_t : 2;
        std::uint8_t rx_pw_p0 : 6;
    } PACKED;

    struct RX_PW_P1 {
        std::uint8_t : 2;
        std::uint8_t rx_pw_p1 : 6;
    } PACKED;

    struct RX_PW_P2 {
        std::uint8_t : 2;
        std::uint8_t rx_pw_p2 : 6;
    } PACKED;

    struct RX_PW_P3 {
        std::uint8_t : 2;
        std::uint8_t rx_pw_p3 : 6;
    } PACKED;

    struct RX_PW_P4 {
        std::uint8_t : 2;
        std::uint8_t rx_pw_p4 : 6;
    } PACKED;

    struct RX_PW_P5 {
        std::uint8_t : 2;
        std::uint8_t rx_pw_p5 : 6;
    } PACKED;

    struct FIFO_STATUS {
        std::uint8_t : 1;
        std::uint8_t tx_reuse : 1;
        std::uint8_t tx_full : 1;
        std::uint8_t tx_empty : 1;
        std::uint8_t : 2;
        std::uint8_t rx_full : 1;
        std::uint8_t rx_empty : 1;
    } PACKED;

    using ACK_PLD = std::array<std::uint8_t, 32UL>;

    using TX_PLD = std::array<std::uint8_t, 32UL>;

    using RX_PLD = std::array<std::uint8_t, 32UL>;

    struct DYNPD {
        std::uint8_t : 2;
        std::uint8_t dpl_p5 : 1;
        std::uint8_t dpl_p4 : 1;
        std::uint8_t dpl_p3 : 1;
        std::uint8_t dpl_p2 : 1;
        std::uint8_t dpl_p1 : 1;
        std::uint8_t dpl_p0 : 1;
    } PACKED;

    struct FEATURE {
        std::uint8_t : 4;
        std::uint8_t en_dpl : 1;
        std::uint8_t en_ack_pay : 1;
        std::uint8_t en_dyn_ack : 1;
    } PACKED;

    struct Config {
        CONFIG config{};
        EN_AA en_aa{};
        EN_RXADDR en_rxaddr{};
        SETUP_AW setup_aw{};
        SETUP_RETR setup_retr{};
        RF_CH rf_ch{};
        RF_SETUP rf_setup{};
        DYNPD dynpd{};
        FEATURE feature{};
    };

}; // namespace nRF24L01

#undef PACKED

#endif // NRF24L01_REGISTER_HPP