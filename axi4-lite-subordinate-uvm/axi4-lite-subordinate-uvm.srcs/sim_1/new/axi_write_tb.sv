`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/22/2026
// Design Name: 
// Module Name: axi_write_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module axi_write_tb;
    // Pararmeters
    localparam int ADDR_W   = 12;
    localparam int DATA_W   = 32;
    localparam int NUM_REG  = 32;
    localparam int STRB_W   = DATA_W / 8;
    localparam int ADDR_LSB = $clog2(STRB_W);
    localparam int MAP_SIZE = NUM_REG * STRB_W;
 
    localparam logic [1:0] RESP_OKAY   = 2'b00;
    localparam logic [1:0] RESP_SLVERR = 2'b10;


    // Signals
    logic                ACLK    = 1'b0;
    logic                ARESETn = 1'b0;
 
    logic                AWVALID = 1'b0;
    logic                AWREADY;
    logic [ADDR_W-1:0]   AWADDR  = '0;
    logic [2:0]          AWPROT  = '0;
 
    logic                WVALID  = 1'b0;
    logic                WREADY;
    logic [DATA_W-1:0]   WDATA   = '0;
    logic [STRB_W-1:0]   WSTRB   = '0;
 
    logic                BVALID;
    logic                BREADY  = 1'b0;
    logic [1:0]          BRESP;
 
    // Read channels tied off (read path not tested here)
    logic                ARVALID = 1'b0;
    logic                ARREADY;
    logic [ADDR_W-1:0]   ARADDR  = '0;
    logic [2:0]          ARPROT  = '0;
    logic                RVALID;
    logic                RREADY  = 1'b0;
    logic [DATA_W-1:0]   RDATA;
    logic [1:0]          RRESP;
   
   
    // DUT
    
    axi4lite_subordinate #(
        .ADDR_W  (ADDR_W),
        .DATA_W  (DATA_W),
        .NUM_REG (NUM_REG)
   ) dut (.*);
   
   always #5 ACLK = ~ACLK; // 100MHz
   
   // ref model & quick error handling
   logic [DATA_W-1:0] model [NUM_REG];
   int errors     = 0;
   int num_writes = 0;
   
   function automatic void fail(input string msg);
       $error("[%0t] %s", $time, msg);
       errors++;
   endfunction
   
   function automatic void check_regs(input string tag);
       for (int i = 0; i < NUM_REG; i++) begin
           if (dut.regs[i] !== model[i]) begin
                $error("[%0t] %s: reg[%0d] = 0x%h, expected 0x%h",
                            $time, tag, i, dut.regs[i], model[i]);
                errors++;
           end
       end
   endfunction
   
   task automatic send_aw(input logic [ADDR_W-1:0] addr,
                          input int delay);
       repeat (delay) @(negedge ACLK);
       AWADDR = addr;
       AWVALID = 1'b1;
       while (!AWREADY) @(negedge ACLK); //READY seen goes into handshake at next posedge
       @(posedge ACLK);
       @(negedge ACLK);
       AWVALID = 1'b0;
   endtask
   
   task automatic send_w(input logic [DATA_W-1:0] data,
                         input logic [STRB_W-1:0] strb,
                         input int delay);
   
       repeat (delay) @(negedge ACLK);
       WDATA  = data;
       WSTRB  = strb;
       WVALID = 1'b1;
       while (!WREADY) @(negedge ACLK);
       @(posedge ACLK);
       @(negedge ACLK);
       WVALID = 1'b0;
   endtask
   
   // wait for BVALID, holds BREADY low for "stall" cycles while checking that
   // BVALID/BRESP stays stable, then completes handshake
   task automatic get_b(input int stall, output logic [1:0] resp);
       while (!BVALID) @(negedge ACLK);
       resp = BRESP;
       repeat (stall) begin
           @(negedge ACLK);
           if (!BVALID) begin        fail("BVALID dropped before BREADY"); end
           if (BRESP !== resp) begin fail("BRESP changed while waiting for BREADY"); end
       end
       BREADY = 1'b1;
       @(posedge ACLK);
       @(negedge ACLK);
       BREADY = 1'b0;
       if (BVALID) fail("BVALID still high after B handshake");
   endtask
   
   // Full write transaction && checking
   task automatic axi_write(input logic[ADDR_W-1:0] addr,
                            input logic[DATA_W-1:0] data,
                            input logic[STRB_W-1:0] strb,
                            input int aw_delay = 0,
                            input int w_delay  = 0,
                            input int b_stall  = 0);
       logic [1:0] resp, exp_resp;
       bit         in_range;
       int         idx;
       
       @(negedge ACLK);
       fork
           send_aw(addr, aw_delay);
           send_w(data, strb, w_delay);
       join
       get_b(b_stall, resp);
       
       in_range = (addr < MAP_SIZE);
       exp_resp = in_range ? RESP_OKAY : RESP_SLVERR;
       if (resp !== exp_resp) begin
           fail($sformatf("addr 0x%h: BRESP = %b, expected %b",
               addr, resp, exp_resp));
       end
       
       if (in_range) begin
           idx = addr >> ADDR_LSB;
           for (int b = 0; b < STRB_W; b++) begin
               if (strb[b]) begin
                  model[idx][8*b +: 8] = data[8*b +: 8];
               end
           end
       end
       
       check_regs($sformatf("after write #%0d to 0x%h", num_writes, addr));
       num_writes++;
   endtask
   
   // Test sequence
   initial begin
        for (int i = 0; i < NUM_REG; i++) model[i] = '0;
 
        // Reset
        ARESETn = 1'b0;
        repeat (5) @(posedge ACLK);
        @(negedge ACLK);
        if (BVALID !== 1'b0) fail("BVALID not low during reset");
        ARESETn = 1'b1;
        check_regs("after reset");
 
        //        addr                             data          strb     aw w  b
        axi_write(ADDR_W'('h000),                  32'hDEAD_BEEF, 4'hF);            // AW + W same cycle
        axi_write(ADDR_W'('h004),                  32'h1234_5678, 4'hF,   0, 3);    // AW first
        axi_write(ADDR_W'('h008),                  32'hCAFE_F00D, 4'hF,   3, 0);    // W first
        axi_write(ADDR_W'('h000),                  32'h1111_2222, 4'b0101);         // partial: bytes 0, 2 of reg 0
        axi_write(ADDR_W'('h00C),                  32'hFFFF_FFFF, 4'h0);            // WSTRB = 0: OKAY, nothing written
        axi_write(ADDR_W'(MAP_SIZE - STRB_W),      32'hA5A5_A5A5, 4'hF);            // last mapped reg (0x07C)
        axi_write(ADDR_W'(MAP_SIZE),               32'hFFFF_FFFF, 4'hF);            // first unmapped (0x080): SLVERR
        axi_write(ADDR_W'(2**ADDR_W - STRB_W),     32'hFFFF_FFFF, 4'hF);            // top of space (0xFFC): SLVERR
        axi_write(ADDR_W'('h010),                  32'h0BAD_CAFE, 4'hF,   0, 0, 5); // B backpressure, 5 cycles
 
        $display("====================================");
        $display(" writes: %0d   errors: %0d", num_writes, errors);
        $display(errors == 0 ? " PASS" : " FAIL");
        $display("====================================");
        $finish;
   end
     // Watchdog: catch a hung handshake
    initial begin
        #10us;
        $fatal(1, "TIMEOUT: a handshake never completed");
    end
 

endmodule
