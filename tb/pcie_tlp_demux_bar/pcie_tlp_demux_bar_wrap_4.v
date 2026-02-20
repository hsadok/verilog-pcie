/*

Copyright (c) 2022 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * PCIe TLP 4 port demux (BAR ID) (wrapper)
 */
module pcie_tlp_demux_bar_wrap_4 #
(
    // TLP data width
    parameter TLP_DATA_WIDTH = 256,
    // TLP strobe width
    parameter TLP_STRB_WIDTH = TLP_DATA_WIDTH/32,
    // TLP header width
    parameter TLP_HDR_WIDTH = 128,
    // Sequence number width
    parameter SEQ_NUM_WIDTH = 6,
    // TLP segment count (input)
    parameter IN_TLP_SEG_COUNT = 1,
    // TLP segment count (output)
    parameter OUT_TLP_SEG_COUNT = IN_TLP_SEG_COUNT,
    // Include output FIFOs
    parameter FIFO_ENABLE = 1,
    // FIFO depth
    parameter FIFO_DEPTH = 2048,
    // FIFO watermark level
    parameter FIFO_WATERMARK = FIFO_DEPTH/2,
    // Base BAR
    parameter BAR_BASE = 0,
    // BAR stride
    parameter BAR_STRIDE = 1,
    // Explicit BAR numbers (set to 0 to use base/stride)
    parameter BAR_IDS = 0
)
(
    input  wire                                        clk,
    input  wire                                        rst,

    /*
     * TLP input
     */
    input  wire [TLP_DATA_WIDTH-1:0]                   in_tlp_data,
    input  wire [TLP_STRB_WIDTH-1:0]                   in_tlp_strb,
    input  wire [IN_TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0]   in_tlp_hdr,
    input  wire [IN_TLP_SEG_COUNT*SEQ_NUM_WIDTH-1:0]   in_tlp_seq,
    input  wire [IN_TLP_SEG_COUNT*3-1:0]               in_tlp_bar_id,
    input  wire [IN_TLP_SEG_COUNT*8-1:0]               in_tlp_func_num,
    input  wire [IN_TLP_SEG_COUNT*4-1:0]               in_tlp_error,
    input  wire [IN_TLP_SEG_COUNT-1:0]                 in_tlp_valid,
    input  wire [IN_TLP_SEG_COUNT-1:0]                 in_tlp_sop,
    input  wire [IN_TLP_SEG_COUNT-1:0]                 in_tlp_eop,
    output wire                                        in_tlp_ready,

    /*
     * TLP outputs
     */
    output wire [TLP_DATA_WIDTH-1:0]                   out00_tlp_data,
    output wire [TLP_STRB_WIDTH-1:0]                   out00_tlp_strb,
    output wire [OUT_TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0]  out00_tlp_hdr,
    output wire [OUT_TLP_SEG_COUNT*SEQ_NUM_WIDTH-1:0]  out00_tlp_seq,
    output wire [OUT_TLP_SEG_COUNT*3-1:0]              out00_tlp_bar_id,
    output wire [OUT_TLP_SEG_COUNT*8-1:0]              out00_tlp_func_num,
    output wire [OUT_TLP_SEG_COUNT*4-1:0]              out00_tlp_error,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out00_tlp_valid,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out00_tlp_sop,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out00_tlp_eop,
    input  wire                                        out00_tlp_ready,

    output wire [TLP_DATA_WIDTH-1:0]                   out01_tlp_data,
    output wire [TLP_STRB_WIDTH-1:0]                   out01_tlp_strb,
    output wire [OUT_TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0]  out01_tlp_hdr,
    output wire [OUT_TLP_SEG_COUNT*SEQ_NUM_WIDTH-1:0]  out01_tlp_seq,
    output wire [OUT_TLP_SEG_COUNT*3-1:0]              out01_tlp_bar_id,
    output wire [OUT_TLP_SEG_COUNT*8-1:0]              out01_tlp_func_num,
    output wire [OUT_TLP_SEG_COUNT*4-1:0]              out01_tlp_error,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out01_tlp_valid,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out01_tlp_sop,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out01_tlp_eop,
    input  wire                                        out01_tlp_ready,

    output wire [TLP_DATA_WIDTH-1:0]                   out02_tlp_data,
    output wire [TLP_STRB_WIDTH-1:0]                   out02_tlp_strb,
    output wire [OUT_TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0]  out02_tlp_hdr,
    output wire [OUT_TLP_SEG_COUNT*SEQ_NUM_WIDTH-1:0]  out02_tlp_seq,
    output wire [OUT_TLP_SEG_COUNT*3-1:0]              out02_tlp_bar_id,
    output wire [OUT_TLP_SEG_COUNT*8-1:0]              out02_tlp_func_num,
    output wire [OUT_TLP_SEG_COUNT*4-1:0]              out02_tlp_error,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out02_tlp_valid,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out02_tlp_sop,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out02_tlp_eop,
    input  wire                                        out02_tlp_ready,

    output wire [TLP_DATA_WIDTH-1:0]                   out03_tlp_data,
    output wire [TLP_STRB_WIDTH-1:0]                   out03_tlp_strb,
    output wire [OUT_TLP_SEG_COUNT*TLP_HDR_WIDTH-1:0]  out03_tlp_hdr,
    output wire [OUT_TLP_SEG_COUNT*SEQ_NUM_WIDTH-1:0]  out03_tlp_seq,
    output wire [OUT_TLP_SEG_COUNT*3-1:0]              out03_tlp_bar_id,
    output wire [OUT_TLP_SEG_COUNT*8-1:0]              out03_tlp_func_num,
    output wire [OUT_TLP_SEG_COUNT*4-1:0]              out03_tlp_error,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out03_tlp_valid,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out03_tlp_sop,
    output wire [OUT_TLP_SEG_COUNT-1:0]                out03_tlp_eop,
    input  wire                                        out03_tlp_ready,

    /*
     * Control
     */
    input  wire                                        enable,

    /*
     * Status
     */
    output wire                                        out00_fifo_half_full,
    output wire                                        out00_fifo_watermark,
    output wire                                        out01_fifo_half_full,
    output wire                                        out01_fifo_watermark,
    output wire                                        out02_fifo_half_full,
    output wire                                        out02_fifo_watermark,
    output wire                                        out03_fifo_half_full,
    output wire                                        out03_fifo_watermark
);

pcie_tlp_demux_bar #(
    .PORTS(4),
    .TLP_DATA_WIDTH(TLP_DATA_WIDTH),
    .TLP_STRB_WIDTH(TLP_STRB_WIDTH),
    .TLP_HDR_WIDTH(TLP_HDR_WIDTH),
    .SEQ_NUM_WIDTH(SEQ_NUM_WIDTH),
    .IN_TLP_SEG_COUNT(IN_TLP_SEG_COUNT),
    .OUT_TLP_SEG_COUNT(OUT_TLP_SEG_COUNT),
    .FIFO_ENABLE(FIFO_ENABLE),
    .FIFO_DEPTH(FIFO_DEPTH),
    .FIFO_WATERMARK(FIFO_WATERMARK),
    .BAR_BASE(BAR_BASE),
    .BAR_STRIDE(BAR_STRIDE),
    .BAR_IDS(BAR_IDS)
)
pcie_tlp_demux_bar_inst (
    .clk(clk),
    .rst(rst),

    /*
     * TLP input
     */
    .in_tlp_data(in_tlp_data),
    .in_tlp_strb(in_tlp_strb),
    .in_tlp_hdr(in_tlp_hdr),
    .in_tlp_seq(in_tlp_seq),
    .in_tlp_bar_id(in_tlp_bar_id),
    .in_tlp_func_num(in_tlp_func_num),
    .in_tlp_error(in_tlp_error),
    .in_tlp_valid(in_tlp_valid),
    .in_tlp_sop(in_tlp_sop),
    .in_tlp_eop(in_tlp_eop),
    .in_tlp_ready(in_tlp_ready),

    /*
     * TLP output
     */
    .out_tlp_data({ out03_tlp_data, out02_tlp_data, out01_tlp_data, out00_tlp_data }),
    .out_tlp_strb({ out03_tlp_strb, out02_tlp_strb, out01_tlp_strb, out00_tlp_strb }),
    .out_tlp_hdr({ out03_tlp_hdr, out02_tlp_hdr, out01_tlp_hdr, out00_tlp_hdr }),
    .out_tlp_seq({ out03_tlp_seq, out02_tlp_seq, out01_tlp_seq, out00_tlp_seq }),
    .out_tlp_bar_id({ out03_tlp_bar_id, out02_tlp_bar_id, out01_tlp_bar_id, out00_tlp_bar_id }),
    .out_tlp_func_num({ out03_tlp_func_num, out02_tlp_func_num, out01_tlp_func_num, out00_tlp_func_num }),
    .out_tlp_error({ out03_tlp_error, out02_tlp_error, out01_tlp_error, out00_tlp_error }),
    .out_tlp_valid({ out03_tlp_valid, out02_tlp_valid, out01_tlp_valid, out00_tlp_valid }),
    .out_tlp_sop({ out03_tlp_sop, out02_tlp_sop, out01_tlp_sop, out00_tlp_sop }),
    .out_tlp_eop({ out03_tlp_eop, out02_tlp_eop, out01_tlp_eop, out00_tlp_eop }),
    .out_tlp_ready({ out03_tlp_ready, out02_tlp_ready, out01_tlp_ready, out00_tlp_ready }),

    /*
     * Control
     */
    .enable(enable),

    /*
     * Status
     */
    .fifo_half_full({ out03_fifo_half_full, out02_fifo_half_full, out01_fifo_half_full, out00_fifo_half_full }),
    .fifo_watermark({ out03_fifo_watermark, out02_fifo_watermark, out01_fifo_watermark, out00_fifo_watermark })
);

endmodule

`resetall
