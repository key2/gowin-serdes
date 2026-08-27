// USB3 enumeration core shim.
//
// Wraps the Gowin USB3.2 device controller stack (pipe/ltssm/link netlists +
// plaintext protocol/endpoint RTL) and the reference-design user layer
// (ControlTransfer + descriptors) behind FLAT ports, so the Amaranth top can
// instantiate it with a plain Verilog-2001 instantiation (the vendor modules
// use SystemVerilog unpacked-array ports, which Amaranth's Instance cannot
// connect directly).
//
// The bulk endpoint (EP2 IN) data source is tied off: the device enumerates
// as the reference UVC camera but never streams video -- exactly what a
// basic "does it enumerate" bring-up needs.

`include "usb3_macro_define.v"

module usb31_enum_core (
     input  wire        pclk
    ,input  wire        reset_n

    // PIPE interface to the PHY
    ,input  wire [3:0]  phy_pipe_rx_data_k
    ,input  wire [63:0] phy_pipe_rx_data
    ,input  wire [3:0]  phy_pipe_rx_sync_head
    ,input  wire        phy_pipe_rx_start_block
    ,input  wire        phy_pipe_rx_valid
    ,output wire [3:0]  phy_pipe_tx_data_k
    ,output wire [63:0] phy_pipe_tx_data
    ,output wire [3:0]  phy_pipe_tx_sync_head
    ,output wire        phy_pipe_tx_start_block
    ,output wire        phy_pipe_tx_valid

    // PHY control/status
    ,output wire        phy_reset_n
    ,output wire        phy_tx_detrx_lpbk
    ,output wire        phy_tx_elecidle
    ,input  wire        phy_rx_elecidle
    ,input  wire [2:0]  phy_rx_status
    ,output wire [1:0]  phy_power_down
    ,input  wire        phy_phy_status
    ,input  wire        phy_pwrpresent
    ,output wire [1:0]  phy_tx_deemph
    ,output wire [2:0]  phy_tx_margin
    ,output wire        phy_tx_swing
    ,output wire        phy_rx_polarity
    ,output wire        phy_rx_termination
    ,output wire        phy_rate
    ,output wire        phy_elas_buf_mode
    ,input  wire [4:0]  phy_tx_fifo_wrnum
    ,input  wire        phy_serdes_pll_lock
    ,output wire        phy_ltssm_is_training
    ,output wire [5:0]  dbg_ltssm_state

    // debug / status
    ,output wire        attached
    ,output wire        itp_received
    ,output wire        warm_or_hot_reset
    ,output wire        request_active
    ,output wire [7:0]  bmRequestType
    ,output wire [7:0]  bRequest
    ,output wire [15:0] wValue
    ,output wire [15:0] wIndex
    ,output wire [15:0] wLength
);

// ---------------------------------------------------------------------
// endpoint transfer buses (SystemVerilog unpacked arrays)
// ---------------------------------------------------------------------
wire        host_requests_data_from_endpt [0:15];
wire        wait_user_commit_request;
wire        request_commit;

wire        transfer_in_mem_wr          [0:15];
wire [7:0]  transfer_in_mem_wr_mask     [0:15];
wire [13:0] transfer_in_mem_wr_addr     [0:15];
wire [63:0] transfer_in_mem_wr_data     [0:15];
wire        transfer_in_mem_commit      [0:15];
wire [16:0] transfer_in_mem_commit_len  [0:15];
wire        transfer_in_mem_ready       [0:15];
wire        transfer_in_done            [0:15];
wire        transfer_in_mem_empty       [0:15];

wire        transfer_out_mem_has_data   [0:15];
wire [16:0] transfer_out_mem_len        [0:15];
wire [13:0] transfer_out_mem_rd_addr    [0:15];
wire [63:0] transfer_out_mem_rd_data    [0:15];
wire        transfer_out_mem_clr        [0:15];

// ---------------------------------------------------------------------
// USB 3.2 device controller (pipe + ltssm + link netlists, protocol RTL)
// ---------------------------------------------------------------------
`getname(usb3_2_device_controller,`module_name)
usb3_2_device_controller_inst (
     .phy_clk                   (pclk)
    ,.reset_n                   (reset_n)
    ,.phy_pipe_rx_data_k        (phy_pipe_rx_data_k)
    ,.phy_pipe_rx_data          (phy_pipe_rx_data)
    ,.phy_pipe_rx_sync_head     (phy_pipe_rx_sync_head)
    ,.phy_pipe_rx_start_block   (phy_pipe_rx_start_block)
    ,.phy_pipe_rx_valid         (phy_pipe_rx_valid)
    ,.phy_pipe_tx_data_k        (phy_pipe_tx_data_k)
    ,.phy_pipe_tx_data          (phy_pipe_tx_data)
    ,.phy_pipe_tx_sync_head     (phy_pipe_tx_sync_head)
    ,.phy_pipe_tx_start_block   (phy_pipe_tx_start_block)
    ,.phy_pipe_tx_valid         (phy_pipe_tx_valid)
    ,.phy_reset_n               (phy_reset_n)
    ,.phy_tx_detrx_lpbk         (phy_tx_detrx_lpbk)
    ,.phy_tx_elecidle           (phy_tx_elecidle)
    ,.phy_rx_elecidle           (phy_rx_elecidle)
    ,.phy_rx_status             (phy_rx_status)
    ,.phy_power_down            (phy_power_down)
    ,.phy_phy_status            (phy_phy_status)
    ,.phy_pwrpresent            (phy_pwrpresent)
    ,.phy_tx_deemph             (phy_tx_deemph)
    ,.phy_tx_margin             (phy_tx_margin)
    ,.phy_tx_swing              (phy_tx_swing)
    ,.phy_rx_polarity           (phy_rx_polarity)
    ,.phy_rx_termination        (phy_rx_termination)
    ,.phy_rate                  (phy_rate)
    ,.phy_elas_buf_mode         (phy_elas_buf_mode)
    ,.phy_tx_fifo_wrnum         (phy_tx_fifo_wrnum)
    ,.phy_serdes_pll_lock       (phy_serdes_pll_lock)
    ,.phy_ltssm_is_training     (phy_ltssm_is_training)
    ,.dbg_ltssm_state           (dbg_ltssm_state)

    ,.warm_or_hot_reset         (warm_or_hot_reset)
    ,.host_requests_data_from_endpt (host_requests_data_from_endpt)
    ,.itp_received              (itp_received)
    ,.attached                  (attached)

    ,.request_active            (request_active)
    ,.bmRequestType             (bmRequestType)
    ,.bRequest                  (bRequest)
    ,.wValue                    (wValue)
    ,.wIndex                    (wIndex)
    ,.wLength                   (wLength)
    ,.wait_user_commit_request  (wait_user_commit_request)
    ,.request_commit            (request_commit)

    ,.transfer_in_mem_wr            (transfer_in_mem_wr)
    ,.transfer_in_mem_wr_mask       (transfer_in_mem_wr_mask)
    ,.transfer_in_mem_wr_addr       (transfer_in_mem_wr_addr)
    ,.transfer_in_mem_wr_data       (transfer_in_mem_wr_data)
    ,.transfer_in_mem_commit        (transfer_in_mem_commit)
    ,.transfer_in_mem_commit_len    (transfer_in_mem_commit_len)
    ,.transfer_in_mem_ready         (transfer_in_mem_ready)
    ,.transfer_in_done              (transfer_in_done)
    ,.transfer_in_mem_empty         (transfer_in_mem_empty)

    ,.transfer_out_mem_has_data     (transfer_out_mem_has_data)
    ,.transfer_out_mem_len          (transfer_out_mem_len)
    ,.transfer_out_mem_rd_addr      (transfer_out_mem_rd_addr)
    ,.transfer_out_mem_rd_data      (transfer_out_mem_rd_data)
    ,.transfer_out_mem_clr          (transfer_out_mem_clr)
);

// ---------------------------------------------------------------------
// User layer: EP0 control transfers + descriptors, EP2 IN tied off
// ---------------------------------------------------------------------
UserLayer_top UserLayer_top_inst (
     .pclk                      (pclk)
    ,.phy_resetn                (reset_n)
    ,.phy_rate                  (phy_rate)

    ,.warm_or_hot_reset         (warm_or_hot_reset)
    ,.host_requests_data_from_endpt (host_requests_data_from_endpt)
    ,.itp_received              (itp_received)
    ,.attached                  (attached)

    ,.request_active            (request_active)
    ,.bmRequestType             (bmRequestType)
    ,.bRequest                  (bRequest)
    ,.wValue                    (wValue)
    ,.wIndex                    (wIndex)
    ,.wLength                   (wLength)
    ,.wait_user_commit_request  (wait_user_commit_request)
    ,.request_commit            (request_commit)

    ,.transfer_in_mem_wr            (transfer_in_mem_wr)
    ,.transfer_in_mem_wr_mask       (transfer_in_mem_wr_mask)
    ,.transfer_in_mem_wr_addr       (transfer_in_mem_wr_addr)
    ,.transfer_in_mem_wr_data       (transfer_in_mem_wr_data)
    ,.transfer_in_mem_commit        (transfer_in_mem_commit)
    ,.transfer_in_mem_commit_len    (transfer_in_mem_commit_len)
    ,.transfer_in_mem_ready         (transfer_in_mem_ready)
    ,.transfer_in_done              (transfer_in_done)
    ,.transfer_in_mem_empty         (transfer_in_mem_empty)

    ,.transfer_out_mem_has_data     (transfer_out_mem_has_data)
    ,.transfer_out_mem_len          (transfer_out_mem_len)
    ,.transfer_out_mem_rd_addr      (transfer_out_mem_rd_addr)
    ,.transfer_out_mem_rd_data      (transfer_out_mem_rd_data)
    ,.transfer_out_mem_clr          (transfer_out_mem_clr)

    // video line-fifo interface: tied off (no video source)
    ,.intf_active               ()
    ,.yuv_rd_en                 ()
    ,.yuv_rd_data               (64'd0)
    ,.yuv_data_byte_num         (15'd0)
);

endmodule
