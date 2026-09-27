// fault_inject.svh
// Fault-injection helpers for AXI4-Lite subordinate testbenches.
//

// OOB reads: also require RDATA == 0 alongside SLVERR.
`ifndef FI_CHECK_ERR_RDATA_ZERO
`define FI_CHECK_ERR_RDATA_ZERO 1
`endif

// Force a signal to a value for N cycles, then release. Blocks the caller, so
// fork it next to the traffic it should disturb:
//     fork `FI_STUCK(RDATA, '0, 20)  fi_axi_read(ADDR_W'('h004)); join
// Do NOT stick a READY low: the handshake waits have no timeout, so the
// global watchdog will $fatal the whole run.
`ifndef FI_STUCK
`define FI_STUCK(sig, val, ncyc) \
  begin \
    $display("[%0t]      stuck: %s = %s for %0d cycles", $time, `"sig`", `"val`", ncyc); \
    force sig = val; \
    repeat (ncyc) begin @(negedge ACLK); end \
    release sig; \
  end
`endif

//     states
int    fi_faults, fi_detected, fi_masked;
int    fi_err_snap;
string fi_fault_name;
int    fi_num_reads;
int    fi_ar_pending = 0;

// R channel monitor
// RVALID must only appear with an accepted AR outstanding. (axi_read_tb has an
// equivalent check, so there a real R violation is simply reported twice.)
always @(posedge ACLK) begin
    if (!ARESETn) begin
        fi_ar_pending = 0;
    end else begin
        if (RVALID && fi_ar_pending == 0) begin fail("[FI mon] RVALID asserted with no outstanding AR"); end
        if (ARVALID && ARREADY) begin fi_ar_pending++; end
        if (RVALID  && RREADY)  begin fi_ar_pending--; end
    end
end

// SVA checks
// Concurrent assertions for the AXI4-Lite rules, each named after the matching
// Arm AXI assertion (as listed in AMD's AXI Protocol Checker guide, PG101).
// Errors go through fail(), so inside a fault window they count as detections
// like every other checker. 
// Define FI_NO_SVA before the include to turn the whole block off.
`ifndef FI_NO_SVA

`ifndef FI_MAXWAITS
`define FI_MAXWAITS 16
`endif

`define FI_SVA(name, prop, msg) \
  name: assert property (@(posedge ACLK) disable iff (!ARESETn) prop) \
    else begin fail($sformatf("[SVA %s] %s", `"name`", msg)); end

`define FI_REC(name, prop, msg) \
  name: assert property (@(posedge ACLK) disable iff (!ARESETn) prop) \
    else begin $warning("[SVA %s] %s", `"name`", msg); end

// X only matters on byte lanes whose strobe is set (AXI_ERRM_WDATA_X).
function automatic bit fi_wdata_lanes_known(logic [DATA_W-1:0] d, logic [STRB_W-1:0] s);
    for (int b = 0; b < STRB_W; b++) begin
        if (s[b] && $isunknown(d[8*b +: 8])) begin return 1'b0; end
    end
    return 1'b1;
endfunction

// Handshake: VALID held until READY, payload stable meanwhile 
`FI_SVA(AXI_ERRM_AWVALID_STABLE, AWVALID && !AWREADY |=> AWVALID,
        "AWVALID dropped before AWREADY")
`FI_SVA(AXI_ERRM_AWADDR_STABLE,  AWVALID && !AWREADY |=> {AWADDR, AWPROT} === $past({AWADDR, AWPROT}),
        "AWADDR/AWPROT changed while waiting for AWREADY")
`FI_SVA(AXI_ERRM_WVALID_STABLE,  WVALID && !WREADY |=> WVALID,
        "WVALID dropped before WREADY")
`FI_SVA(AXI_ERRM_WDATA_STABLE,   WVALID && !WREADY |=> {WDATA, WSTRB} === $past({WDATA, WSTRB}),
        "WDATA/WSTRB changed while waiting for WREADY")
`FI_SVA(AXI_ERRS_BVALID_STABLE,  BVALID && !BREADY |=> BVALID,
        "BVALID dropped before BREADY")
`FI_SVA(AXI_ERRS_BRESP_STABLE,   BVALID && !BREADY |=> BRESP === $past(BRESP),
        "BRESP changed while waiting for BREADY")
`FI_SVA(AXI_ERRM_ARVALID_STABLE, ARVALID && !ARREADY |=> ARVALID,
        "ARVALID dropped before ARREADY")
`FI_SVA(AXI_ERRM_ARADDR_STABLE,  ARVALID && !ARREADY |=> {ARADDR, ARPROT} === $past({ARADDR, ARPROT}),
        "ARADDR/ARPROT changed while waiting for ARREADY")
`FI_SVA(AXI_ERRS_RVALID_STABLE,  RVALID && !RREADY |=> RVALID,
        "RVALID dropped before RREADY")
`FI_SVA(AXI_ERRS_RDATA_STABLE,   RVALID && !RREADY |=> {RDATA, RRESP} === $past({RDATA, RRESP}),
        "RDATA/RRESP changed while waiting for RREADY")

// ---- No X on handshake signals out of reset, or on payload while VALID ----
`FI_SVA(AXI_ERR_VALID_READY_X,
        !$isunknown({AWVALID, AWREADY, WVALID, WREADY, BVALID, BREADY, ARVALID, ARREADY, RVALID, RREADY}),
        "X on a VALID/READY signal out of reset")
`FI_SVA(AXI_ERRM_AWADDR_X, AWVALID |-> !$isunknown({AWADDR, AWPROT}), "X on AWADDR/AWPROT while AWVALID")
`FI_SVA(AXI_ERRM_WSTRB_X,  WVALID  |-> !$isunknown(WSTRB),             "X on WSTRB while WVALID")
`FI_SVA(AXI_ERRM_WDATA_X,  WVALID  |-> fi_wdata_lanes_known(WDATA, WSTRB),
        "X on a strobed WDATA byte lane while WVALID")
`FI_SVA(AXI_ERRS_BRESP_X,  BVALID  |-> !$isunknown(BRESP),             "X on BRESP while BVALID")
`FI_SVA(AXI_ERRM_ARADDR_X, ARVALID |-> !$isunknown({ARADDR, ARPROT}), "X on ARADDR/ARPROT while ARVALID")
`FI_SVA(AXI_ERRS_RDATA_X,  RVALID  |-> !$isunknown({RDATA, RRESP}),   "X on RDATA/RRESP while RVALID")

//  AXI4-Lite: exclusive access is not supported
`FI_SVA(AXI4LITE_ERRS_BRESP_EXOKAY, BVALID |-> BRESP !== 2'b01, "EXOKAY write response on AXI4-Lite")
`FI_SVA(AXI4LITE_ERRS_RRESP_EXOKAY, RVALID |-> RRESP !== 2'b01, "EXOKAY read response on AXI4-Lite")

// Reset
// Manager VALIDs low throughout reset. Subordinate VALIDs low from the second
// reset cycle on (a synchronous-reset DUT needs one edge to clear them).
// Every VALID low in the first cycle after ARESETn goes high.
AXI_RST_MGR_VALID_LOW: assert property (@(posedge ACLK)
        !ARESETn |-> !AWVALID && !WVALID && !ARVALID)
    else begin fail("[SVA AXI_RST_MGR_VALID_LOW] AW/W/ARVALID high during reset"); end
AXI_RST_SUB_VALID_LOW: assert property (@(posedge ACLK)
        (!ARESETn ##1 !ARESETn) |-> BVALID === 1'b0 && RVALID === 1'b0)
    else begin fail("[SVA AXI_RST_SUB_VALID_LOW] BVALID/RVALID high during reset"); end
AXI_ERR_VALID_RESET: assert property (@(posedge ACLK)
        $rose(ARESETn) |-> !AWVALID && !WVALID && !ARVALID && !BVALID && !RVALID)
    else begin fail("[SVA AXI_ERR_VALID_RESET] a VALID was high in the first cycle after reset"); end

// READY within FI_MAXWAITS cycles (warnings only)
`FI_REC(AXI_RECS_AWREADY_MAX_WAIT, $rose(AWVALID) |-> ##[0:`FI_MAXWAITS] AWREADY, "AWREADY slow")
`FI_REC(AXI_RECS_WREADY_MAX_WAIT,  $rose(WVALID)  |-> ##[0:`FI_MAXWAITS] WREADY,  "WREADY slow")
`FI_REC(AXI_RECS_ARREADY_MAX_WAIT, $rose(ARVALID) |-> ##[0:`FI_MAXWAITS] ARREADY, "ARREADY slow")
`FI_REC(AXI_RECM_BREADY_MAX_WAIT,  $rose(BVALID)  |-> ##[0:`FI_MAXWAITS] BREADY,  "BREADY slow")
`FI_REC(AXI_RECM_RREADY_MAX_WAIT,  $rose(RVALID)  |-> ##[0:`FI_MAXWAITS] RREADY,  "RREADY slow")

`endif // FI_NO_SVA

// read path 
task automatic fi_send_ar(input logic [ADDR_W-1:0] addr, input int delay);
    repeat (delay) begin @(negedge ACLK); end
    ARADDR  = addr;
    ARVALID = 1'b1;
    while (!ARREADY) begin @(negedge ACLK); end
    @(posedge ACLK);
    @(negedge ACLK);
    ARVALID = 1'b0;
endtask

// Wait for RVALID, hold RREADY low for `stall` cycles checking RVALID/RDATA/RRESP
// stay put, then complete the handshake.
task automatic fi_get_r(input  int                stall,
                        output logic [DATA_W-1:0] data,
                        output logic [1:0]        resp);
    while (!RVALID) begin @(negedge ACLK); end
    data = RDATA;
    resp = RRESP;
    repeat (stall) begin
        @(negedge ACLK);
        if (!RVALID)        begin fail("RVALID dropped before RREADY"); end
        if (RDATA !== data) begin fail("RDATA changed while waiting for RREADY"); end
        if (RRESP !== resp) begin fail("RRESP changed while waiting for RREADY"); end
    end
    RREADY = 1'b1;
    @(posedge ACLK);
    @(negedge ACLK);
    RREADY = 1'b0;
    if (RVALID) begin fail("RVALID still high after R handshake"); end
endtask

// Full read checked against the model. do_check_regs=0 skips the backdoor
// compare, for reads that overlap an in-flight write.
task automatic fi_axi_read(input logic [ADDR_W-1:0] addr,
                           input int ar_delay      = 0,
                           input int r_stall       = 0,
                           input bit do_check_regs = 1);
    logic [DATA_W-1:0] data, exp_data;
    logic [1:0]        resp, exp_resp;
    bit                in_range;

    @(negedge ACLK);
    fi_send_ar(addr, ar_delay);
    fi_get_r(r_stall, data, resp);

    in_range = (addr < MAP_SIZE);
    exp_resp = in_range ? RESP_OKAY : RESP_SLVERR;
    exp_data = in_range ? model[addr >> ADDR_LSB] : '0;

    if (resp !== exp_resp) begin
        fail($sformatf("read 0x%h: RRESP = %b, expected %b", addr, resp, exp_resp));
    end
    if ((in_range || `FI_CHECK_ERR_RDATA_ZERO) && data !== exp_data) begin
        fail($sformatf("read 0x%h: RDATA = 0x%h, expected 0x%h", addr, data, exp_data));
    end
    if (do_check_regs) begin
        check_regs($sformatf("after FI read #%0d to 0x%h", fi_num_reads, addr));
    end
    fi_num_reads++;
endtask

// Reset that is safe mid-transaction: drops every manager VALID/READY on both
// channels and checks BVALID and RVALID are low while ARESETn is low.
task automatic fi_reset(input int cycles = 5);
    if (ARESETn) begin @(negedge ACLK); end
    ARESETn = 1'b0;
    AWVALID = 1'b0;  
    WVALID  = 1'b0;
    BREADY  = 1'b0;
    ARVALID = 1'b0;  
    RREADY  = 1'b0;
    foreach (model[i]) begin model[i] = '0; end
    repeat (cycles) begin
        @(negedge ACLK);
        if (BVALID !== 1'b0) begin fail("BVALID not low during reset"); end
        if (RVALID !== 1'b0) begin fail("RVALID not low during reset"); end
    end
    // AXI: deassertion must be synchronous with a rising edge of ACLK, so
    // release it just after a posedge, then realign to the negedge the
    // drivers use.
    @(posedge ACLK);
    ARESETn <= 1'b1;
    @(negedge ACLK);
    check_regs("after FI reset");
endtask

// protocol faults: write

// All 2^STRB_W strobe patterns on one register, read back through the bus.
// The strb==0 case carries X data, which must never reach the register.
task automatic fi_strobe_sweep(input logic [ADDR_W-1:0] addr);
    logic [DATA_W-1:0] base = {STRB_W{8'hA5}};
    logic [DATA_W-1:0] patt = {STRB_W{8'h3C}};
    testcase($sformatf("FI: WSTRB sweep, all %0d patterns @0x%h", 1 << STRB_W, addr));
    for (int s = 0; s < (1 << STRB_W); s++) begin
        axi_write(addr, base, '1);
        axi_write(addr, (s == 0) ? {DATA_W{1'bx}} : patt, STRB_W'(s));
        fi_axi_read(addr);
    end
endtask

// Each of AW / W leads by 1, 3 and 8 cycles, with B/R backpressure at the same depth.
task automatic fi_skew_sweep(input logic [ADDR_W-1:0] addr);
    int d[3] = '{1, 3, 8};
    testcase($sformatf("FI: AW/W skew sweep @0x%h", addr));
    foreach (d[i]) begin
        axi_write  (addr, DATA_W'($urandom), '1, d[i], 0,    d[i]);   // W leads AW
        fi_axi_read(addr, 0, d[i]);
        axi_write  (addr, DATA_W'($urandom), '1, 0,    d[i], d[i]);   // AW leads W
        fi_axi_read(addr, 0, d[i]);
    end
endtask

// protocol faults: read
// Every combination of AR delay and R backpressure (0, 1, 3, 8 cycles).
task automatic fi_read_timing_sweep(input logic [ADDR_W-1:0] addr);
    int d[4] = '{0, 1, 3, 8};
    testcase($sformatf("FI: AR delay x R backpressure sweep @0x%h", addr));
    axi_write(addr, DATA_W'(32'h5AC3_3CA5), '1);
    foreach (d[i]) begin
        foreach (d[j]) begin
            fi_axi_read(addr, d[i], d[j]);
        end
    end
endtask

// Unique value in every register, read back in a scrambled order (stride 7,
// coprime with 32) with varying timing. Catches read-decode aliasing and stale
// RDATA from the previous read.
task automatic fi_read_map();
    int idx;
    testcase("FI: full map unique pattern, scrambled read order");
    for (int i = 0; i < NUM_REG; i++) begin
        axi_write(ADDR_W'(i * STRB_W), {16'hBEEF, 8'(i), 8'(~i)}, '1);
    end
    for (int i = 0; i < NUM_REG; i++) begin
        idx = (i * 7) % NUM_REG;
        fi_axi_read(ADDR_W'(idx * STRB_W), i % 3, i % 4);
    end
endtask

// OOB reads: SLVERR, RDATA = 0, no register side effects.
task automatic fi_read_bad_addr();
    testcase("FI: out-of-range reads return SLVERR");
    fi_axi_read(ADDR_W'(MAP_SIZE));                        // first unmapped word
    fi_axi_read(ADDR_W'(MAP_SIZE + STRB_W + 1), 2, 3);     // unmapped, unaligned
    fi_axi_read(ADDR_W'(2**ADDR_W - STRB_W), 0, 5);        // top of address space
endtask

// protocol faults: both

// Write one register while reading another, with the two transactions
// overlapping at different offsets. Catches read/write arbitration bugs
// (a read stalling a write, or returning the write's data).
task automatic fi_concurrent_one(input int wi, input int ri,
                                 input int aw_d, input int w_d, input int b_s,
                                 input int ar_d, input int r_s);
    logic [DATA_W-1:0] wdata = DATA_W'($urandom);
    fork
        axi_write  (ADDR_W'(wi * STRB_W), wdata, '1, aw_d, w_d, b_s);
        fi_axi_read(ADDR_W'(ri * STRB_W), ar_d, r_s, 0);
    join
    check_regs("after concurrent write/read");
endtask

task automatic fi_concurrent_rw();
    testcase("FI: simultaneous write and read to different registers");
    for (int k = 0; k < 8; k++) begin
        fi_concurrent_one(k, NUM_REG - 1 - k, k % 3, (k + 1) % 3, k % 2, (k + 2) % 3, k % 4);
    end
endtask

// X on every address/data/strobe input while all VALIDs are low. The DUT must
// ignore it: no X on handshake outputs, no stray BVALID/RVALID, no reg changes.
task automatic fi_x_idle(input int cycles = 20);
    testcase($sformatf("FI: X on idle bus for %0d cycles", cycles));
    @(negedge ACLK);
    AWADDR = 'x;  WDATA = 'x;  WSTRB = 'x;  ARADDR = 'x;
    repeat (cycles) begin
        @(negedge ACLK);
        if ($isunknown({AWREADY, WREADY, ARREADY, BVALID, RVALID})) begin
            fail("X on a subordinate handshake output while bus idle");
        end else if (BVALID || RVALID) begin
            fail("BVALID/RVALID asserted on an idle bus");
        end
    end
    AWADDR = '0;  WDATA = '0;  WSTRB = '0;  ARADDR = '0;
    check_regs("after X on idle bus");
endtask

// Pull reset 1..max_cycles into a write, then into a read, so reset lands in
// every phase (AW, W, B wait, AR, R wait). The bus must recover each time.
// Leaves every register at its reset value, so run it last.
task automatic fi_reset_sweep(input logic [ADDR_W-1:0] addr, input int max_cycles = 6);
    for (int n = 1; n <= max_cycles; n++) begin
        testcase($sformatf("FI: reset %0d cycles into a write @0x%h", n, addr));
        fork begin                                    // isolate disable fork
            fork
                axi_write(addr, DATA_W'(32'h1234_5678), '1, 0, 0, 2);
                begin repeat (n) begin @(negedge ACLK); end end
            join_any
            disable fork;
        end join
        fi_reset();
        axi_write  (addr, DATA_W'(32'hCAFE_F00D), '1);
        fi_axi_read(addr);

        testcase($sformatf("FI: reset %0d cycles into a read @0x%h", n, addr));
        fork begin
            fork
                fi_axi_read(addr, 0, 2);
                begin repeat (n) begin @(negedge ACLK); end end
            join_any
            disable fork;
        end join
        fi_reset();
        fi_axi_read(addr);                            // model is 0 after reset
    end
endtask

// internal faults

// Errors raised by fail() inside a window are the fault's symptoms: they're
// counted as detections and removed from `errors`, so the final PASS/FAIL only
// reflects unexpected problems.
function automatic void fi_fault_begin(input string name);
    testcase($sformatf("FI fault: %s", name));
    fi_faults++;
    fi_fault_name = name;
    fi_err_snap   = errors;
    $display("[%0t] ---- FAULT %0d: %s  (errors until END FAULT are expected)",
             $time, fi_faults, name);
endfunction

// Recover and classify. The DUT is reset (a forced flop can keep its bad value
// after release and simulators often merge the TB net with the DUT port),
// then every register is restored from the model by backdoor. Anything the
// fault still disturbs during recovery counts as a symptom too.
task automatic fi_fault_end();
    logic [DATA_W-1:0] snap [NUM_REG];
    int hits;
    snap = model;
    fi_reset();
    model = snap;
    foreach (model[i]) begin dut.regs[i] = model[i]; end
    hits   = errors - fi_err_snap;
    errors = fi_err_snap;
    if (hits > 0) begin
        fi_detected++;
        $display("[%0t] ---- END FAULT %0d: DETECTED (%0d checker hits)", $time, fi_faults, hits);
    end else begin
        fi_masked++;
        $display("[%0t] ---- END FAULT %0d: MASKED", $time, fi_faults);
    end
endtask

// Single Event upset: flip one register bit by deposit, at negedge so it can't
// race the DUT's always_ff. The next legit write to that register clears it.
task automatic fi_seu(input int reg_idx, input int bit_idx);
    @(negedge ACLK);
    dut.regs[reg_idx][bit_idx] = ~dut.regs[reg_idx][bit_idx];
    $display("[%0t]      SEU: flipped regs[%0d][%0d]", $time, reg_idx, bit_idx);
endtask

task automatic fi_write_faults();
    // SEU overwritten by a full-strobe write before anyone looks: expect MASKED
    fi_fault_begin("SEU regs[6][0], overwritten before observed");
    fi_seu(6, 0);
    axi_write(ADDR_W'(6 * STRB_W), DATA_W'(32'h0BAD_F00D), '1);
    fi_fault_end();

    // Phantom BVALID on an idle bus: only the B-channel monitor can see this
    fi_fault_begin("BVALID stuck-at-1 on idle bus");
    repeat (2) begin @(negedge ACLK); end          // clear of the reset-release cycle
    `FI_STUCK(BVALID, 1'b1, 3)
    fi_fault_end();

    // Wrong response code on a legal write
    fi_fault_begin("BRESP stuck at SLVERR during a legal write");
    fork
        `FI_STUCK(BRESP, RESP_SLVERR, 20)
        axi_write(ADDR_W'('h008), DATA_W'(32'hFEED_FACE), '1);
    join
    fi_fault_end();

    // Write data corrupted on the wires: the register gets the wrong value
    fi_fault_begin("WDATA bus stuck-at-0 during a write");
    fork
        `FI_STUCK(WDATA, '0, 20)
        axi_write(ADDR_W'('h00C), DATA_W'(32'h1357_9BDF), '1);
    join
    fi_fault_end();

    // EXOKAY on a legal write: illegal on AXI4-Lite. Caught by the BRESP
    // compare AND by SVA AXI4LITE_ERRS_BRESP_EXOKAY (proves the SVA block runs).
    fi_fault_begin("BRESP = EXOKAY during a legal write");
    fork
        `FI_STUCK(BRESP, 2'b01, 20)
        axi_write(ADDR_W'('h008), DATA_W'(32'hFEED_FACE), '1);
    join
    fi_fault_end();

    // Write address stuck: the write lands on reg 0 instead of reg 4
    fi_fault_begin("AWADDR stuck at 0x000 during a write to 0x010");
    fork
        `FI_STUCK(AWADDR, '0, 20)
        axi_write(ADDR_W'('h010), DATA_W'(32'hA11A_5ED0), '1);
    join
    fi_fault_end();
endtask

task automatic fi_read_faults();
    // SEU, then read the register: bus data check + check_regs both fire
    fi_fault_begin("SEU regs[5][7], then read regs[5]");
    fi_seu(5, 7);
    fi_axi_read(ADDR_W'(5 * STRB_W));
    fi_fault_end();

    // Phantom RVALID on an idle bus: only the R-channel monitor can see this
    fi_fault_begin("RVALID stuck-at-1 on idle bus");
    repeat (2) begin @(negedge ACLK); end          // clear of the reset-release cycle
    `FI_STUCK(RVALID, 1'b1, 3)
    fi_fault_end();

    // X on RVALID for one cycle on an idle bus. The procedural monitors read
    // X as false and stay silent, so only SVA AXI_ERR_VALID_READY_X sees it.
    fi_fault_begin("RVALID = X on idle bus (SVA-only detection)");
    repeat (2) begin @(negedge ACLK); end
    `FI_STUCK(RVALID, 1'bx, 1)
    fi_fault_end();

    // Corrupted read data
    fi_fault_begin("RDATA stuck-at-0 during a read");
    axi_write(ADDR_W'('h004), DATA_W'(32'h1234_5678), '1);
    fork
        `FI_STUCK(RDATA, '0, 20)
        fi_axi_read(ADDR_W'('h004));
    join
    fi_fault_end();

    // Wrong response code on a legal read
    fi_fault_begin("RRESP stuck at SLVERR during a legal read");
    fork
        `FI_STUCK(RRESP, RESP_SLVERR, 20)
        fi_axi_read(ADDR_W'('h004));
    join
    fi_fault_end();

    // Read address stuck: reg 0 comes back instead of reg 3
    fi_fault_begin("ARADDR stuck at 0x000 during a read of 0x00C");
    axi_write(ADDR_W'('h000), DATA_W'(32'h0000_AAAA), '1);
    axi_write(ADDR_W'('h00C), DATA_W'(32'h0000_CCCC), '1);
    fork
        `FI_STUCK(ARADDR, '0, 20)
        fi_axi_read(ADDR_W'('h00C));
    join
    fi_fault_end();
endtask

// runners
task automatic fi_run_all();
    fi_x_idle();
    fi_strobe_sweep     (ADDR_W'('h010));
    fi_skew_sweep       (ADDR_W'('h018));
    fi_read_timing_sweep(ADDR_W'('h01C));
    fi_read_bad_addr();
    fi_read_map();
    fi_concurrent_rw();
    fi_write_faults();
    fi_read_faults();
    fi_reset_sweep      (ADDR_W'('h020));             // last: leaves regs at reset values
endtask

function automatic void fi_report();
    $display(" FI reads: %0d   faults injected: %0d  detected: %0d  masked: %0d",
             fi_num_reads, fi_faults, fi_detected, fi_masked);
endfunction
