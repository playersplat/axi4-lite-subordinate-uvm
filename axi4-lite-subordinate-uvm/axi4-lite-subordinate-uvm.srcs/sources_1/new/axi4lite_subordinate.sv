`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name:   axi4lite_subordinate
// Project:       AXI4-Lite Subordinate + UVM Verification
// Author:        playersplat
// Create Date:   09/21/2026
// Target Device: Artix-7 100T (xc7a100t[package-speed])
// Tool Version:  Vivado [2023.1]
//
// Description:
//   AXI4-Lite subordinate (slave) with a memory-mapped register file.
//   Independent AW/W channel acceptance, WSTRB byte-lane writes, and
//   SLVERR response for out-of-range addresses.
//
// Parameters:
//   ADDR_W - address bus width (default 12)
//   DATA_W - data bus width (default 32)
//   NUM_REG   - number of 32-bit registers (default 32)
//
// Reference:  ARM IHI 0022H, AMBA AXI and ACE Protocol Specification
//
// SPDX-License-Identifier: MIT
// SLVERR used to indicate unsuccessfull transaction
// returned for unmapped offsets iwthin this subordinate's address window
// DECERR is reserved for the interconnect, per the spec:
// "Generated, typically, by an interconnect component, to incidate that
// there is no subordinate at the transaction address." (IHI 0022H A3-60)
//////////////////////////////////////////////////////////////////////////////////


module axi4lite_subordinate #(
    parameter int ADDR_W  = 12,  // byte address width for subordinate
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
                                        //[0] - 0 unprivileged / 1 privileged
                                        //[1] - 0 secure /  1 non-secure
                                        //[2] - 0 data / 1 instruction
    
    // Write data channel
    input  logic                WVALID,
    output logic                WREADY,
    input  logic [DATA_W-1:0]   WDATA,
    input  logic [STRB_W-1:0]   WSTRB, // one bit per byte lane
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
                                        //[0] - 0 unprivileged / 1 privileged
                                        //[1] - 0 secure /  1 non-secure
                                        //[2] - 0 data / 1 instruction
    
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

typedef enum logic [1:0] {W_IDLE, W_WAIT_DATA, W_WAIT_ADDR, W_RESP} w_state_t;
w_state_t w_state;

logic [DATA_W-1:0] regs [NUM_REG];
logic [ADDR_W-1:0] addr_q, wr_addr;
logic [DATA_W-1:0] data_q, wr_data;
logic [STRB_W-1:0] strb_q, wr_strb;
logic [IDX_W-1:0]  wr_idx;
logic              aw_hs, w_hs, wr_fire, wr_in_range;

always_comb begin
    AWREADY = (w_state == W_IDLE) || (w_state == W_WAIT_ADDR);
    WREADY  = (w_state == W_IDLE) || (w_state == W_WAIT_DATA);
    aw_hs   = AWVALID && AWREADY; //valid ready signal state machine
    w_hs    = WVALID  && WREADY;  //
    
    // default to stop latches
    wr_fire = 1'b0;
    wr_addr = AWADDR;
    wr_data = WDATA;
    wr_strb = WSTRB;
    
    unique case (w_state)
        W_IDLE      : wr_fire = aw_hs && w_hs;
        W_WAIT_DATA : begin wr_fire = w_hs; wr_addr = addr_q; end
        W_WAIT_ADDR : begin wr_fire = aw_hs; wr_data = data_q; wr_strb = strb_q; end
        default     :   ; // W_RESP: nothing fires
    endcase
    
    wr_in_range = wr_addr < MAP_SIZE;
    wr_idx      = wr_addr[ADDR_LSB +: IDX_W]; // indexed part select  
end

always_ff @(posedge ACLK) begin
    if (!ARESETn) begin
        w_state <= W_IDLE;
        BVALID  <= 1'b0; //low during reset
        BRESP   <= RESP_OKAY;
        addr_q  <= '0;
        data_q  <= '0;
        strb_q  <= 0;
        for (int i = 0; i < NUM_REG; i++) begin 
            regs[i] <= '0; 
        end
    end else begin
    
        // register write
        if (wr_fire && wr_in_range) begin
            for (int b = 0; b < STRB_W; b++) begin
                if(wr_strb[b]) begin
                    regs[wr_idx][8*b +: 8] <= wr_data[8*b +: 8];
                end
            end 
        end
        
        // write response: set on write and hold until handshake
        if (wr_fire) begin
            BVALID <= 1'b1;
            BRESP  <= wr_in_range ? RESP_OKAY : RESP_SLVERR;
        end else if (BVALID && BREADY) begin
            BVALID <= 1'b0;
        end
        // state transitions and latching which one arrived first
        unique case (w_state)
            W_IDLE :
                if (wr_fire) begin
                    w_state <= W_RESP;
                end else if (aw_hs) begin
                    addr_q  <= AWADDR;
                    w_state <= W_WAIT_DATA;
                end else if (w_hs) begin
                    data_q  <= WDATA;
                    strb_q  <= WSTRB;
                    w_state <= W_WAIT_ADDR;
                end else begin
                    w_state <= W_IDLE;
                end
                
            W_WAIT_DATA, W_WAIT_ADDR:
                if (wr_fire) begin
                    w_state <= W_RESP;
                end else begin
                    w_state <= w_state;
                end
                
            W_RESP:
                if (BVALID && BREADY) begin
                    w_state <= W_IDLE;
                end else begin
                    w_state <= W_RESP;
                end
        endcase
    end
end

endmodule