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
    parameter int ADDR_W = 8, // byte address width for subordinate
    parameter int DATA_W = 32 // going for 32 bit length for ease
) (
    // global
    input logic ACLK,
    input logic ARESETn, //active low
    
    // Write address channel
    input  logic                AWVALID,
    output logic                AWREADY,
    input  logic [ADDR_W-1:0]   AWADDR,
    output logic [2:0]          AWPROT, //3 independent yes/no attribs
                                        //Bit 0 - privileged
                                        //Bit 1 - secure / non-secure
                                        //Bit 2 - instruction / data
    
    // Write data channel
    input  logic                WVALID,
    output logic                WREADY,
    input  logic [DATA_W-1:0]   WDATA,
    input  logic [DATA_W/8-1:0] WSTRB, // one bit per byte lane
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
    input  logic [DATA_W-1:0]   ARADDR,
    input  logic [2:0]          ARPROT, //3 independent yes/no attribs
                                        //Bit 0 - privileged
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


endmodule
