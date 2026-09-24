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
    // Parameters
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
   int tc_num     = 0;
   
   function automatic void fail(input string msg);
        $error("[%0t] TC%0d: %s", $time, tc_num, msg);
        errors++;
   endfunction
   
   function automatic void testcase(input string name);
        tc_num++;
        $display("[%0t] ---- TC%0d: %s", $time, tc_num, name);
   endfunction
   
   function automatic void check_regs(input string tag);
       for (int i = 0; i < NUM_REG; i++) begin
           if (dut.regs[i] !== model[i]) begin
               fail($sformatf("%s: reg[%0d] = 0x%h, expected 0x%h",
                              tag, i, dut.regs[i], model[i]));
           end
       end
   endfunction
   
   // Protocol monitor: B must never appear before both AW and W were accepted
   int aw_pending = 0, w_pending = 0;

   always @(posedge ACLK) begin
       if (!ARESETn) begin
           aw_pending = 0;
           w_pending  = 0;
       end else begin
           if (BVALID && (aw_pending == 0 || w_pending == 0)) begin
               fail("BVALID asserted before both AW and W were accepted");
           end
           if (AWVALID && AWREADY) begin aw_pending++; end
           if (WVALID  && WREADY)  begin w_pending++;  end
           if (BVALID  && BREADY)  begin aw_pending--; w_pending--; end
       end
   end
   
   task automatic send_aw(input logic [ADDR_W-1:0] addr,
                          input int delay);
       repeat (delay) begin @(negedge ACLK); end
       AWADDR = addr;
       AWVALID = 1'b1;
       while (!AWREADY) begin @(negedge ACLK); end//READY seen goes into handshake at next posedge
       @(posedge ACLK);
       @(negedge ACLK);
       AWVALID = 1'b0;
   endtask
   
   task automatic send_w(input logic [DATA_W-1:0] data,
                         input logic [STRB_W-1:0] strb,
                         input int delay);
   
       repeat (delay) begin @(negedge ACLK); end
       WDATA  = data;
       WSTRB  = strb;
       WVALID = 1'b1;
       while (!WREADY) begin @(negedge ACLK); end
       @(posedge ACLK);
       @(negedge ACLK);
       WVALID = 1'b0;
   endtask
   
   
   // wait for BVALID, holds BREADY low for "stall" cycles while checking that
   // BVALID/BRESP stays stable, then completes handshake
   task automatic get_b(input int stall, output logic [1:0] resp);
       while (!BVALID) begin @(negedge ACLK); end
       resp = BRESP;
       repeat (stall) begin
           @(negedge ACLK);
           if (!BVALID) begin fail("BVALID dropped before BREADY"); end
           if (BRESP !== resp) begin fail("BRESP changed while waiting for BREADY"); end
       end
       BREADY = 1'b1;
       @(posedge ACLK);
       @(negedge ACLK);
       BREADY = 1'b0;
       if (BVALID) begin fail("BVALID still high after B handshake"); end
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
   
   // reset DUT alongside model and check both come out at zero
   // currently only calls between transactions, never mid handshake
   // going to implement that later
   task automatic do_reset(input int cycles = 5);
       if (ARESETn) begin @(negedge ACLK); end // to account for initial ARESETn assignment
       ARESETn = 1'b0;
       AWVALID = 1'b0; //spec wants manager to drive VALIDs low during reset
       WVALID  = 1'b0;
       BREADY  = 1'b0;
       foreach (model[i]) begin model[i] = '0; end
       repeat (cycles) begin @(posedge ACLK); end
       @(negedge ACLK);
       if (BVALID !== 1'b0) begin fail("BVALID not low during reset"); end
       ARESETn = 1'b1;
       check_regs("after reset");
   endtask
   // Test sequence
   initial begin
 
        // Reset
        do_reset();
 
        
        //        addr                             data          strb     aw w  b
        testcase("AW and W in the same cycle");
        axi_write(ADDR_W'('h000),                  32'hDEAD_BEEF, 4'hF);
        testcase("AW before W");
        axi_write(ADDR_W'('h004),                  32'h1234_5678, 4'hF,   0, 3);
        testcase("W before AW");
        axi_write(ADDR_W'('h008),                  32'hCAFE_F00D, 4'hF,   3, 0);
        testcase("AW and W both delayed");
        axi_write(ADDR_W'('h014),                  32'hFACA_DE00, 4'hF,   3, 3);
        testcase("WSTRB = 0 writes nothing");
        axi_write(ADDR_W'('h00C),                  32'hFFFF_FFFF, 4'h0);
        
        testcase("mid-test reset clears written registers");
        do_reset();

        testcase("unaligned partial write lands in containing word");
        axi_write(ADDR_W'('h016),                  32'h1357_9BDF, 4'b1100);         // reg 5 -> 0x13570000
        testcase("post-reset rewrite, then partial strobe merge");
        axi_write(ADDR_W'('h000),                  32'h7733_8844, 4'hF);
        axi_write(ADDR_W'('h000),                  32'h1111_2222, 4'b0101);         // reg 0 -> 0x77118822
        testcase("last mapped register");
        axi_write(ADDR_W'(MAP_SIZE - STRB_W),      32'hA5A5_A5A5, 4'hF);
        testcase("first unmapped address: SLVERR");
        axi_write(ADDR_W'(MAP_SIZE),               32'hFFFF_FFFF, 4'hF);
        testcase("top of address space: SLVERR");
        axi_write(ADDR_W'(2**ADDR_W - STRB_W),     32'hFFFF_FFFF, 4'hF);
        testcase("B channel backpressure, 5 cycles");
        axi_write(ADDR_W'('h010),                  32'h0BAD_CAFE, 4'hF,   0, 0, 5);

        $display("====================================");
        $display(" writes: %0d   errors: %0d", num_writes, errors);
        $display(" aw_pending: %0d    w_pending: %0d", aw_pending, w_pending);
        $display("%s", errors == 0 ? " PASS" : " FAIL");
        $display("====================================");
        $finish;
   end
     // Watchdog: catch a hung handshake
    initial begin
        #10us;
        $fatal(1, "TIMEOUT: a handshake never completed");
    end
 

endmodule
