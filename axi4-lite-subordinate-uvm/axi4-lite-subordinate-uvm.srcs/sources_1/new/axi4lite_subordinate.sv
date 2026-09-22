`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name:   axi4lite_subordinate
// Project:       AXI4-Lite Subordinate + UVM Verification
// Author:        playersplat
// Create Date:   09/21/2026
// Target Device: Artix-7 100T (xc7a100t[package-speed])
// Tool Version:  Vivado [version]
//
// Description:
//   AXI4-Lite subordinate (slave) with a memory-mapped register file.
//   Independent AW/W channel acceptance, WSTRB byte-lane writes, and
//   SLVERR response for out-of-range addresses.
//
// Parameters:
//   ADDR_WIDTH - address bus width (default 32)
//   DATA_WIDTH - data bus width (default 32)
//   NUM_REGS   - number of 32-bit registers (default 16)
//
// Reference:  ARM IHI 0022H, AMBA AXI and ACE Protocol Specification
//
// SPDX-License-Identifier: MIT
//////////////////////////////////////////////////////////////////////////////////


module axi4lite_subordinate #(
    parameter int ADDR_W  = 8,  // byte address width for subordinate
    parameter int DATA_W  = 32, // going for 32 bit length for ease
    parameter int NUM_REG = 32,  // num of DATA_W - wide regs
    localparam int STRB_W = DATA_W / 8
) (
    // global
    input logic ACLK,
    input logic ARESETn, //active low
    
    // Write address channel
    input  logic                AWVALID,
    output logic                AWREADY,
    input  logic [ADDR_W-1:0]   AWADDR,
    input  logic [2:0]          AWPROT, //3 independent yes/no attribs
                                        //Bit 0 - unprivileged / privileged
                                        //Bit 1 - secure / non-secure
                                        //Bit 2 - instruction / data
    
    // Write data channel
    input  logic                WVALID,
    output logic                WREADY,
    input  logic [DATA_W-1:0]   WDATA,
    input  logic [STRB_W-1:0] WSTRB, // one bit per byte lane
                                       // scales with data bus
    
    // Write response channel
    output logic                BVALID,
    input  logic                BREADY,
    output logic [1:0]          BRESP, // OKAY  (2'b00)
                                       // EXOKAY(2'b01)
                                       // SLVERR(2'b10)
                                       // DECERR(2'b11)
    
    // Read address channel
    input  logic                ARVALID,
    output logic                ARREADY,
    input  logic [ADDR_W-1:0]   ARADDR,
    input  logic [2:0]          ARPROT, //3 independent yes/no attribs
                                        //Bit 0 - unprivileged / privileged
                                        //Bit 1 - secure / non-secure
                                        //Bit 2 - instruction / data
    
    // Read data channel
    output logic                RVALID,
    input  logic                RREADY,
    output logic [DATA_W-1:0]   RDATA,
    output logic [1:0]          RRESP  // OKAY  (2'b00)
                                       // EXOKAY(2'b01)
                                       // SLVERR(2'b10)
                                       // DECERR(2'b11)
    
);
// B/R RESP types
localparam logic [1:0] RESP_OKAY   = 2'b00;
localparam logic [1:0] RESP_EXOKAY = 2'b01; // not legal for AXI4-Lite
localparam logic [1:0] RESP_SLVERR = 2'b10;
localparam logic [1:0] RESP_DECERR = 2'b11;

// Derived address map params
localparam int ADDR_LSB = $clog2(STRB_W);  // word offset bits dropped (2 for 32-bit)
localparam int IDX_W    = $clog2(NUM_REG); // register index width (5 for 32 regs)
localparam int MAP_SIZE = NUM_REG * STRB_W; // mapped bytes (addr >= MAP_SIZE -> SLVERR


endmodule
