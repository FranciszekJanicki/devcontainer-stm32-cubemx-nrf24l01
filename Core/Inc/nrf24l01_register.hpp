#ifndef NRF24L01_HPP
#define NRF24L01_HPP

#include <cstdint>
#include <utility>

#define packed __attribute__((__packed__))

namespace nRF24L01 {

    struct CONFIG {
        bool : 1;
        bool mask_rx_dr : 1;
        bool mask_tx_ds : 1;
        bool mask_max_rt : 1;
        bool en_crc : 1;
        bool crco : 1;
        bool pwr_up : 1;
        bool prim_rx : 1;
    } packed;

    struct EN_AA {
        uint8_t : 2;
        bool enaa_p5 : 1;
        bool enaa_p4 : 1;
        bool enaa_p3 : 1;
        bool enaa_p2 : 1;
        bool enaa_p1 : 1;
        bool enaa_p0 : 1;
    } packed;

    struct EN_RXADDR {
        uint8_t : 2;
        bool erx_p5 : 1;
        bool erx_p4 : 1;
        bool erx_p3 : 1;
        bool erx_p2 : 1;
        bool erx_p1 : 1;
        bool erx_p0 : 1;
    } packed;

    struct SETUP_AW {
        uint8_t : 6;
        uint8_t aw : 2;
    } packed;

    struct SETUP_RETR {
        uint8_t ard : 4;
        uint8_t arc : 4;
    } packed;

    struct RF_CH {
        bool : 1;
        uint8_t rf_ch : 7;
    } packed;

    struct RF_SETUP {
        uint8_t : 2;
        bool pll_lock : 1;
        bool rd_dr : 1;
        uint8_t rd_pwr : 2;
        bool lna_hcurr : 1;
    } packed;

    struct STATUS {
        bool : 1;
        bool rx_dr : 1;
        bool tx_ds : 1;
        bool max_rt : 1;
        uint8_t rx_p_no : 3;
        bool tx_full : 1;
    } packed;

    struct OBSERVE_TX {
        uint8_t plos_cnt : 4;
        uint8_t arc_cnt : 4;
    } packed;

    struct CD {
        uint8_t : 7;
        bool cd : 1;
    } packed;

    struct RX_ADDR_P0 {
        uint8_t rx_addr_p0[5UL];
    } packed;

    struct RX_ADDR_P1 {
        uint8_t rx_addr_p1[5UL];
    } packed;

    struct RX_ADDR_P2 {
        uint8_t rx_addr_p2;
    } packed;

    struct RX_ADDR_P3 {
        uint8_t rx_addr_p3;
    } packed;

    struct RX_ADDR_P4 {
        uint8_t rx_addr_p4;
    } packed;

    struct RX_ADDR_P5 {
        uint8_t rx_addr_p5;
    } packed;

    struct TX_ADDR {
        uint8_t tx_addr[5UL];
    } packed;

    struct RX_PW_P0 {
        uint8_t : 2;
        uint8_t rx_pw_p0 : 6;
    } packed;

    struct RX_PW_P1 {
        uint8_t : 2;
        uint8_t rx_pw_p1 : 6;
    } packed;

    struct RX_PW_P2 {
        uint8_t : 2;
        uint8_t rx_pw_p2 : 6;
    } packed;

    struct RX_PW_P3 {
        uint8_t : 2;
        uint8_t rx_pw_p3 : 6;
    } packed;

    struct RX_PW_P4 {
        uint8_t : 2;
        uint8_t rx_pw_p4 : 6;
    } packed;

    struct RX_PW_P5 {
        uint8_t : 2;
        uint8_t rx_pw_p5 : 6;
    } packed;

    struct FIFO_STATUS {
        bool : 1;
        bool tx_reuse : 1;
        bool tx_full : 1;
        bool tx_empty : 1;
        uint8_t : 2;
        bool rx_full : 1;
        bool rx_empty : 1;
    } packed;

    struct ACK_PLD {
        uint8_t ack_pld[24UL];
    } packed;

    struct TX_PLD {
        uint8_t tx_pld[24UL];
    } packed;

    struct RX_PLD {
        uint8_t rx_pld[24UL];
    } packed;

    struct DYNPD {
        uint8_t dpl_p5 : 2;
        bool dpl_p5 : 1;
        bool dpl_p4 : 1;
        bool dpl_p3 : 1;
        bool dpl_p2 : 1;
        bool dpl_p1 : 1;
        bool dpl_p0 : 1;
    } packed;

    struct FEATURE {
        uint8_t : 4;
        bool en_dpl : 1;
        bool en_ack_pay : 1;
        bool en_dyn_ack : 1;
    } packed;

}; // namespace nRF24L01

#endif // NRF24L01_HPP