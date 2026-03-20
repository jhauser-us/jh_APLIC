// SPDX-License-Identifier: Apache-2.0

/*============================================================================

Wrapper module for APLIC_AXI4LiteM with a concrete APLICConfig.

Configuration summary:
  - 100 interrupt sources
  - 9 harts (indices 0..8), little-endian only
  - 3 interrupt domains:
      0: M-level root domain    (harts 0..0)
      1: M-level child domain   (harts 1..8), MSI delivery
      2: S-level child domain   (harts 1..8), MSI delivery
  - MSI delivery mode only (no direct delivery)
  - EIID width: 8 bits

*============================================================================*/

module APLIC_AXI4Lite_wrapper (
    input  logic        nReset,
    input  logic        nLockReset,
    input  logic        clock,

    // Interrupt source inputs (bit 0 unused per spec)
    input  logic [100:0] intrsInV,

    // AXI4-Lite subordinate port (configuration/control access)
    // domainNum selects which interrupt domain is being accessed;
    // it is expected to be extracted from the high address bits by the
    // interconnect.
    output logic        s_AWREADY,
    input  logic        s_AWVALID,
    input  logic [1:0]  s_AW_domainNum,
    input  logic [14:0] s_AWADDR,
    output logic        s_WREADY,
    input  logic        s_WVALID,
    input  logic [3:0]  s_WSTRB,
    input  logic [31:0] s_WDATA,
    input  logic        s_BREADY,
    output logic        s_BVALID,
    output logic [1:0]  s_BRESP,
    output logic        s_ARREADY,
    input  logic        s_ARVALID,
    input  logic [1:0]  s_AR_domainNum,
    input  logic [14:0] s_ARADDR,
    input  logic        s_RREADY,
    output logic        s_RVALID,
    output logic [31:0] s_RDATA,

    // AXI4-Lite manager port (outgoing MSI writes)
    input  logic        m_AWREADY,
    output logic        m_AWVALID,
    output logic [31:0] m_AWADDR,
    input  logic        m_WREADY,
    output logic        m_WVALID,
    output logic [3:0]  m_WSTRB,
    output logic [31:0] m_WDATA
);

    // -------------------------------------------------------------------------
    // APLIC configuration
    // -------------------------------------------------------------------------
    localparam APLICPkg::configT APLICConfig = '{
        sources: '{
            num: shortint'(100),
            // Sources 97..101 are detached (unused high sources + source 0)
            implDetachedV:
                                                     5'b_11111<<96
                | 32'b_11111111_11111000_00000000_00000000<<64
                | 32'b_00000000_00000000_00000000_00000000<<32
                | 32'b_00000000_00000000_00000000_00000000,
            // Sources 1..96 support rising-edge (Edge1) triggering
            implEdge1V:
                                                     5'b_00000<<96
                | 32'b_00000111_11111111_11111111_11111111<<64
                | 32'b_11111111_11111111_11111111_11111111<<32
                | 32'b_11111111_11111111_11100000_11111110,
            implEdge0V:  '0,
            implLevel1V: '1,   // all sources support high-level triggering
            implLevel0V: '0
        },
        harts: '{
            maxHartIndex:            shortint'(8),
            numValidHartIndexRanges: byte'(1),
            validHartIndexRanges:    {{shortint'(0), shortint'(8)}},
            implLittleEndian:        '1,
            implBigEndian:           '0,
            maxGEILEN:               byte'(6)
        },
        // No direct delivery mode; zero out the hart index range
        deliverDirectHartIndexRange: '0,
        implOutTopMIntrPrios:        '0,
        implOutTopSIntrPrios:        '0,
        outTopIntrPriosHartIndexRange: '0,
        // All MSI address config fields are writable
        mmsiaddrcfg_writable: '{
            L:       '1,
            HHXS:    'b11111,
            LHXS:    'b111,
            HHXW:    'b011,
            LHXW:    'b1111,
            basePPN: 44'h00000FFFFF0
        },
        mmsiaddrcfg_const: '0,
        smsiaddrcfg_writable: '{
            LHXS:    'b111,
            basePPN: 44'h00000FFFFF0
        },
        smsiaddrcfg_const: '0,
        numDomains: byte'(3),
        // Domains are listed from highest number to lowest (packed array order)
        domains: {
            APLICPkg::domainConfigT'{ // domain 2: S-level, child of domain 1
                parentNum:                   byte'(1),
                privLevel:                   APLICPkg::privLevel_S,
                firstHartIndex:              shortint'(1),
                numHartIndices:              shortint'(8),
                S_perceivedFirstHartIndex:   shortint'(0),
                implDeliverDirect:           '0,
                implDeliverMSI:              '1,
                M_msiaddrcfg_visible:        '0
            },
            APLICPkg::domainConfigT'{ // domain 1: M-level, child of domain 0
                parentNum:                   byte'(0),
                privLevel:                   APLICPkg::privLevel_M,
                firstHartIndex:              shortint'(1),
                numHartIndices:              shortint'(8),
                S_perceivedFirstHartIndex:   '0,
                implDeliverDirect:           '0,
                implDeliverMSI:              '1,
                M_msiaddrcfg_visible:        '1
            },
            APLICPkg::domainConfigT'{ // domain 0: M-level root
                parentNum:                   byte'(-1),
                privLevel:                   APLICPkg::privLevel_M,
                firstHartIndex:              shortint'(0),
                numHartIndices:              shortint'(1),
                S_perceivedFirstHartIndex:   '0,
                implDeliverDirect:           '0,
                implDeliverMSI:              '1,
                M_msiaddrcfg_visible:        '1
            }
        },
        intrPrioW: byte'(0),  // unused (no direct delivery)
        EIIDW:     byte'(8)
    };

    // -------------------------------------------------------------------------
    // APLIC_AXI4LiteM instantiation
    // -------------------------------------------------------------------------
    // harts_intrs/harts_topMIntrPrio/harts_topSIntrPrio are sized by
    // deliverDirectHartIndexRange and outTopIntrPriosHartIndexRange, both '0,
    // so they resolve to [0:0].  Since no domain uses direct delivery these
    // outputs carry no useful data and are left unconnected.
    //
    // managMemPortAddrW = 32: width of MSI write addresses on the manager port.
    // -------------------------------------------------------------------------
    APLIC_AXI4LiteM #(
        .options           (0),
        .APLICConfig       (APLICConfig),
        .managMemPortAddrW (32)
    ) u_aplic (
        .nReset                       (nReset),
        .nLockReset                   (nLockReset),
        .clock                        (clock),

        .intrsInV                     (intrsInV),

        // Subordinate port
        .subordMemPort_AWREADY        (s_AWREADY),
        .subordMemPort_AWVALID        (s_AWVALID),
        .subordMemPort_AW_domainNum   (s_AW_domainNum),
        .subordMemPort_AWADDR         (s_AWADDR),
        .subordMemPort_WREADY         (s_WREADY),
        .subordMemPort_WVALID         (s_WVALID),
        .subordMemPort_WSTRB          (s_WSTRB),
        .subordMemPort_WDATA          (s_WDATA),
        .subordMemPort_BREADY         (s_BREADY),
        .subordMemPort_BVALID         (s_BVALID),
        .subordMemPort_BRESP          (s_BRESP),
        .subordMemPort_ARREADY        (s_ARREADY),
        .subordMemPort_ARVALID        (s_ARVALID),
        .subordMemPort_AR_domainNum   (s_AR_domainNum),
        .subordMemPort_ARADDR         (s_ARADDR),
        .subordMemPort_RREADY         (s_RREADY),
        .subordMemPort_RVALID         (s_RVALID),
        .subordMemPort_RDATA          (s_RDATA),

        // Direct-delivery outputs — unused (no direct delivery configured)
        .harts_intrs                  (),
        .harts_topMIntrPrio           (),
        .harts_topSIntrPrio           (),

        // Manager port (MSI writes)
        .managMemPort_AWREADY         (m_AWREADY),
        .managMemPort_AWVALID         (m_AWVALID),
        .managMemPort_AWADDR          (m_AWADDR),
        .managMemPort_WREADY          (m_WREADY),
        .managMemPort_WVALID          (m_WVALID),
        .managMemPort_WSTRB           (m_WSTRB),
        .managMemPort_WDATA           (m_WDATA)
    );

endmodule
