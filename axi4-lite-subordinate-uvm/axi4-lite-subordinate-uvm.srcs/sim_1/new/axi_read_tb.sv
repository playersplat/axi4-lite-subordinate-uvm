`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/24/2026
// Design Name: 
// Module Name: axi_read_tb
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


module axi_read_tb;
    // Parameters
    localparam int ADDR_W   = 12;
    localparam int DATA_W   = 32;
    localparam int NUM_REG  = 32;
    localparam int STRB_W   = DATA_W / 8;
    localparam int ADDR_LSB = $clog2(STRB_W);
    localparam int MAP_SIZE = NUM_REG * STRB_W;

    localparam logic [1:0] RESP_OKAY   = 2'b00;
    localparam logic [1:0] RESP_SLVERR = 2'b10;

    localparam bit CHECK_ERR_RDATA_ZERO = 1'b1;


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
    int num_reads  = 0;
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

    // Protocol monitor: R needs an accepted AR B must never appear before both AW and W accepted.
    logic ar_pending = 0, aw_pending = 0, w_pending = 0;

    always @(posedge ACLK) begin
        if (!ARESETn) begin
            ar_pending = 0;
            aw_pending = 0; 
            w_pending = 0;
        end else begin
            if (RVALID && ar_pending == 0) begin
                fail("RVALID asserted with no outstanding AR");
            end
            if (BVALID && (aw_pending == 0 || w_pending == 0)) begin
                fail("BVALID asserted before both AW and W were accepted");
            end

            if (ARVALID && ARREADY) begin ar_pending++; end
            if (RVALID  && RREADY)  begin ar_pending--; end
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
    task automatic axi_write(input logic [ADDR_W-1:0] addr,
                             input logic [DATA_W-1:0] data,
                             input logic [STRB_W-1:0] strb,
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
        if (resp !== exp_resp)
            fail($sformatf("write 0x%h: BRESP = %b, expected %b",
                 addr, resp, exp_resp));

        if (in_range) begin
            idx = addr >> ADDR_LSB;
            for (int b = 0; b < STRB_W; b++)
                if (strb[b]) model[idx][8*b +: 8] = data[8*b +: 8];
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
 
    // Read channel tasks
    task automatic send_ar(input logic [ADDR_W-1:0] addr, input int delay);
        repeat (delay) begin @(negedge ACLK); end
        ARADDR  = addr;
        ARVALID = 1'b1;
        while (!ARREADY) begin @(negedge ACLK); end
        @(posedge ACLK);
        @(negedge ACLK);
        ARVALID = 1'b0;
    endtask

    // Wait for RVALID, hold RREADY low for "stall" cycles while checking that
    // RVALID/RDATA/RRESP stay stable, then complete the handshake.
    task automatic get_r(input  int                stall,
                         output logic [DATA_W-1:0] data,
                         output logic [1:0]        resp);
        while (!RVALID) begin @(negedge ACLK); end
        data = RDATA;
        resp = RRESP;
        repeat (stall) begin
            @(negedge ACLK);
            if (!RVALID)        fail("RVALID dropped before RREADY");
            if (RDATA !== data) fail("RDATA changed while waiting for RREADY");
            if (RRESP !== resp) fail("RRESP changed while waiting for RREADY");
        end
        RREADY = 1'b1;
        @(posedge ACLK);
        @(negedge ACLK);
        RREADY = 1'b0;
        if (RVALID) begin fail("RVALID still high after R handshake"); end
    endtask

    // Full read: checks RRESP and RDATA against the model.
    task automatic axi_read(input logic [ADDR_W-1:0] addr,
                            input int ar_delay = 0,
                            input int r_stall  = 0);
        logic [DATA_W-1:0] data, exp_data;
        logic [1:0]        resp, exp_resp;
        bit                in_range;

        @(negedge ACLK);
        send_ar(addr, ar_delay);
        get_r(r_stall, data, resp);

        in_range = (addr < MAP_SIZE);
        exp_resp = in_range ? RESP_OKAY : RESP_SLVERR;
        exp_data = in_range ? model[addr >> ADDR_LSB] : '0;

        if (resp !== exp_resp) begin
            fail($sformatf("read 0x%h: RRESP = %b, expected %b", addr, resp, exp_resp));
        end
        if ((in_range || CHECK_ERR_RDATA_ZERO) && data !== exp_data) begin
            fail($sformatf("read 0x%h: RDATA = 0x%h, expected 0x%h", addr, data, exp_data));
        end
        
        check_regs($sformatf("after read #%0d to 0x%h", num_reads, addr));
        num_reads++;
        
    endtask

    // ---------------- Test sequence ----------------
    initial begin

        // Reset
        do_reset();

        //        addr                             data          strb     aw w  b
        // TC1: every register reads back its reset value (0)
        testcase("reset values, all registers");
        for (int i = 0; i < NUM_REG; i++) axi_read(ADDR_W'(i * STRB_W));

        // TC2: basic write -> read, then read again (reads must not side-effect)
        testcase("write then read back, read twice");
        axi_write(ADDR_W'('h000),                 32'hDEAD_BEEF, 4'hF);
        axi_read (ADDR_W'('h000));
        axi_read (ADDR_W'('h000));

        // TC3: delayed ARVALID + R backpressure (RDATA/RRESP must hold)
        testcase("AR delay 2, R backpressure 5 cycles");
        axi_write(ADDR_W'('h004),                 32'h1234_5678, 4'hF);
        axi_read (ADDR_W'('h004),                                        2, 5);

        // TC4: partial-strobe write merges bytes, visible through the read port
        testcase("partial strobe merge (bytes 0 and 2)");
        axi_write(ADDR_W'('h008),                 32'hCAFE_F00D, 4'hF);
        axi_write(ADDR_W'('h008),                 32'h1111_2222, 4'b0101);
        axi_read (ADDR_W'('h008));                       // expect 0xCA11_F022

        // TC5: WSTRB = 0 writes nothing
        testcase("WSTRB = 0 leaves register unchanged");
        axi_write(ADDR_W'('h00C),                 32'h5555_AAAA, 4'hF);
        axi_write(ADDR_W'('h00C),                 32'hFFFF_FFFF, 4'h0);
        axi_read (ADDR_W'('h00C));

        // TC6: last mapped register
        testcase("last mapped register");
        axi_write(ADDR_W'(MAP_SIZE - STRB_W),     32'hA5A5_A5A5, 4'hF);
        axi_read (ADDR_W'(MAP_SIZE - STRB_W));

        // TC7: first unmapped address -> SLVERR, and must not alias onto reg 0
        testcase("first unmapped address: SLVERR, no aliasing");
        axi_write(ADDR_W'(MAP_SIZE),              32'hFFFF_FFFF, 4'hF);
        axi_read (ADDR_W'(MAP_SIZE));
        axi_read (ADDR_W'('h000));                       // reg 0 still 0xDEADBEEF

        // TC8: top of address space -> SLVERR
        testcase("top of address space: SLVERR");
        axi_read (ADDR_W'(2**ADDR_W - STRB_W));

        // TC9: unaligned read returns the containing word
        testcase("unaligned read address");
        axi_write(ADDR_W'('h014),                 32'h7777_8888, 4'hF);
        axi_read (ADDR_W'('h016));

        // TC10: unique pattern in every register, read back in reverse with
        // random delays/stalls. Catches address decode aliasing.
        testcase("full map unique pattern, random timing");
        for (int i = 0; i < NUM_REG; i++)
            axi_write(ADDR_W'(i * STRB_W), {16'hC0DE, 8'(i), 8'(~i)}, 4'hF,
                      $urandom_range(0, 3), $urandom_range(0, 3), $urandom_range(0, 3));
        for (int i = NUM_REG - 1; i >= 0; i--)
            axi_read(ADDR_W'(i * STRB_W), $urandom_range(0, 3), $urandom_range(0, 4));

        $display("====================================");
        $display(" testcases: %0d  writes: %0d  reads: %0d", tc_num, num_writes, num_reads);
        $display(" ar_pending: %0d, aw_pending: %0d, w_pending: %d;", ar_pending, aw_pending, w_pending);
        $display(" errors: %0d", errors);
        $display("%s", errors == 0 ? " PASS" : " FAIL");
        $display("====================================");
        $finish;
    end

    // Watchdog: catch a hung handshake
    initial begin
        #100us;
        $fatal(1, "TIMEOUT: a handshake never completed");
    end

endmodule