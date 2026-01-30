// SPDX-License-Identifier: Apache-2.0

/*============================================================================

This file is part of an implementation of a RISC-V Advanced Platform-Level
Interrupt Controller (APLIC) by John R. Hauser.

Copyright 2022-2025 John R. Hauser.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*============================================================================*/

/*----------------------------------------------------------------------------
| Supports the following maximums:
|     interrupt domains:     127
|     harts:              16,384
| When direct delivery mode is implemented, the maximum number of harts per
| interrupt domain is 512.
*----------------------------------------------------------------------------*/

module APLIC_AXI4LiteM#(
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        int options = 0,
        APLICPkg::configT APLICConfig,
        int managMemPortAddrW = 0,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam int numISources = APLICConfig.sources.num,
        localparam int domainNumXW =
            (APLICConfig.numDomains < 2) ? 1 : $clog2(APLICConfig.numDomains),
        localparam int deliverDirect_firstHartIndex =
            APLICConfig.deliverDirectHartIndexRange.first,
        localparam int deliverDirect_lastHartIndex =
            APLICConfig.deliverDirectHartIndexRange.last,
        localparam int outTopIntrPrios_firstHartIndex =
            APLICConfig.outTopIntrPriosHartIndexRange.first,
        localparam int outTopIntrPrios_lastHartIndex =
            APLICConfig.outTopIntrPriosHartIndexRange.last,
        localparam int intrPrioW =
            (APLICConfig.intrPrioW < 1) ? 1 : APLICConfig.intrPrioW,
        localparam int managMemPortAddrXW =
            (managMemPortAddrW < 1) ? 1 : managMemPortAddrW
    ) (
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire nReset,
        input uwire nLockReset,
        input uwire clock,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire [numISources:0] intrsInV,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        output uwire subordMemPort_AWREADY,
        input uwire subordMemPort_AWVALID,
        input uwire [(domainNumXW - 1):0] subordMemPort_AW_domainNum,
        input uwire [14:0] subordMemPort_AWADDR,
        output uwire subordMemPort_WREADY,
        input uwire subordMemPort_WVALID,
        input uwire [3:0] subordMemPort_WSTRB,
        input uwire [31:0] subordMemPort_WDATA,
        input uwire subordMemPort_BREADY,
        output uwire subordMemPort_BVALID,
        output uwire [1:0] subordMemPort_BRESP,
        output uwire subordMemPort_ARREADY,
        input uwire subordMemPort_ARVALID,
        input uwire [(domainNumXW - 1):0] subordMemPort_AR_domainNum,
        input uwire [14:0] subordMemPort_ARADDR,
        input uwire subordMemPort_RREADY,
        output uwire subordMemPort_RVALID,
        output reg [31:0] subordMemPort_RDATA,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        output uwire [1:0]
            harts_intrs[
                deliverDirect_firstHartIndex:deliverDirect_lastHartIndex],
        output uwire [(intrPrioW - 1):0]
            harts_topMIntrPrio[
                outTopIntrPrios_firstHartIndex:outTopIntrPrios_lastHartIndex],
        output uwire [(intrPrioW - 1):0]
            harts_topSIntrPrio[
                outTopIntrPrios_firstHartIndex:outTopIntrPrios_lastHartIndex],
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire managMemPort_AWREADY,
        output uwire managMemPort_AWVALID,
        output uwire [(managMemPortAddrXW - 1):0] managMemPort_AWADDR,
        input uwire managMemPort_WREADY,
        output uwire managMemPort_WVALID,
        output uwire [3:0] managMemPort_WSTRB,
        output uwire [31:0] managMemPort_WDATA
    );

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire subordMemPort_doAcceptAW =
        subordMemPort_AWREADY && subordMemPort_AWVALID;
    uwire subordMemPort_doAcceptW =
        subordMemPort_WREADY  && subordMemPort_WVALID;
    uwire subordMemPort_doAcceptAR =
        subordMemPort_ARREADY && subordMemPort_ARVALID;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire subordMemPort_doAdvanceWrite;
    uwire subordMemPort_doAdvanceRead;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    reg subordMemPort_rAWVALID;
    reg subordMemPort_rWVALID;
    reg subordMemPort_rARVALID;
    always_ff @(negedge nReset, posedge clock) begin
        if (!nReset) begin
            subordMemPort_rAWVALID <= '0;
            subordMemPort_rWVALID  <= '0;
            subordMemPort_rARVALID <= '0;
        end else begin
            subordMemPort_rAWVALID <=
                subordMemPort_doAcceptAW
                    || (subordMemPort_rAWVALID
                            && !subordMemPort_doAdvanceWrite);
            subordMemPort_rWVALID <=
                subordMemPort_doAcceptW
                    || (subordMemPort_rWVALID
                            && !subordMemPort_doAdvanceWrite);
            subordMemPort_rARVALID <=
                subordMemPort_doAcceptAR
                    || (subordMemPort_rARVALID
                            && !subordMemPort_doAdvanceRead);
        end
    end
    reg [(domainNumXW - 1):0] subordMemPort_rAW_domainNum;
    reg [(domainNumXW - 1):0] subordMemPort_rAR_domainNum;
    if (APLICConfig.numDomains >= 2) begin
        always_ff @(posedge clock) begin
            if (subordMemPort_doAcceptAW) begin
                subordMemPort_rAW_domainNum <= subordMemPort_AW_domainNum;
            end
            if (subordMemPort_doAcceptAR) begin
                subordMemPort_rAR_domainNum <= subordMemPort_AR_domainNum;
            end
        end
    end
    reg [14:0] subordMemPort_rAWADDR;
    reg [3:0] subordMemPort_rW_allBytes;
    reg [31:0] subordMemPort_rWDATA;
    reg [14:0] subordMemPort_rARADDR;
    always_ff @(posedge clock) begin
        if (subordMemPort_doAcceptAW) begin
            subordMemPort_rAWADDR <= subordMemPort_AWADDR;
        end
        if (subordMemPort_doAcceptW) begin
            subordMemPort_rW_allBytes <= &subordMemPort_WSTRB;
            subordMemPort_rWDATA      <= subordMemPort_WDATA;
        end
        if (subordMemPort_doAcceptAR) begin
            subordMemPort_rARADDR <= subordMemPort_ARADDR;
        end
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    assign subordMemPort_AWREADY = !subordMemPort_rAWVALID;
    assign subordMemPort_WREADY  = !subordMemPort_rWVALID;
    assign subordMemPort_ARREADY = !subordMemPort_rARVALID;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire main_subordMemPort_nextBusy;
    reg main_subordMemPort_enWrite;
    reg subordMemPort_validResp;
    uwire subordMemPort_canAdvanceResp;
    /*------------------------------------------------------------------------
    | If both a read and write are pending simultaneously, give priority to
    | the opposite of the last advanced access (determined from the value of
    | 'main_subordMemPort_enWrite').
    *------------------------------------------------------------------------*/
    uwire subordMemPort_canAdvanceOp =
        !main_subordMemPort_nextBusy
            && (!subordMemPort_validResp || subordMemPort_canAdvanceResp);
    uwire subordMemPort_validWrite =
        subordMemPort_rAWVALID && subordMemPort_rWVALID;
    uwire subordMemPort_selValidWrite =
        subordMemPort_validWrite
            && (!subordMemPort_rARVALID || !main_subordMemPort_enWrite);
    assign subordMemPort_doAdvanceWrite =
        subordMemPort_canAdvanceOp && subordMemPort_selValidWrite;
    uwire subordMemPort_doAdvanceAllowedWrite =
        subordMemPort_doAdvanceWrite && subordMemPort_rW_allBytes;
    uwire subordMemPort_selValidRead =
        subordMemPort_rARVALID
            && (!subordMemPort_validWrite || main_subordMemPort_enWrite);
    assign subordMemPort_doAdvanceRead =
        subordMemPort_canAdvanceOp && subordMemPort_selValidRead;
    uwire subordMemPort_doAdvanceOp =
        subordMemPort_doAdvanceWrite || subordMemPort_doAdvanceRead;
    uwire subordMemPort_doAdvanceAllowedOp =
        subordMemPort_doAdvanceAllowedWrite || subordMemPort_doAdvanceRead;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire main_subordMemPort_validReadResp;
    uwire [31:0] main_subordMemPort_readData;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    reg main_subordMemPort_validOp;
    always_ff @(negedge nReset, posedge clock) begin
        if (!nReset) begin
            main_subordMemPort_validOp <= '0;
            main_subordMemPort_enWrite <= '0;
            subordMemPort_validResp    <= '0;
        end else begin
            main_subordMemPort_validOp <= subordMemPort_doAdvanceAllowedOp;
            if (subordMemPort_doAdvanceOp) begin
                main_subordMemPort_enWrite <= subordMemPort_doAdvanceWrite;
            end
            subordMemPort_validResp <=
                subordMemPort_doAdvanceWrite
                    || main_subordMemPort_validReadResp
                    || (subordMemPort_validResp
                            && !subordMemPort_canAdvanceResp);
        end
    end
    reg [(domainNumXW - 1):0] main_subordMemPort_domainNum;
    if (APLICConfig.numDomains < 2) begin
        assign main_subordMemPort_domainNum = '0;
    end else begin
        always_ff @(posedge clock) begin
            if (subordMemPort_doAdvanceAllowedOp) begin
                main_subordMemPort_domainNum <=
                      (subordMemPort_selValidRead
                           ? subordMemPort_rAR_domainNum : '0)
                    | (subordMemPort_selValidWrite
                           ? subordMemPort_rAW_domainNum : '0);
            end
        end
    end
    reg subordMemPort_resp_deniedWrite;
    reg [12:0] main_subordMemPort_addr;
    reg [31:0] main_subordMemPort_writeData;
    always_ff @(posedge clock) begin
        if (subordMemPort_doAdvanceOp) begin
            subordMemPort_resp_deniedWrite <=
                subordMemPort_selValidWrite && !subordMemPort_rW_allBytes;
        end
        if (subordMemPort_doAdvanceAllowedOp) begin
            main_subordMemPort_addr <=
                  (subordMemPort_selValidRead  ? subordMemPort_rARADDR>>2 : '0)
                | (subordMemPort_selValidWrite ? subordMemPort_rAWADDR>>2
                       : '0);
        end
        if (subordMemPort_doAdvanceAllowedWrite) begin
            main_subordMemPort_writeData <= subordMemPort_rWDATA;
        end
        if (main_subordMemPort_validReadResp) begin
            subordMemPort_RDATA <= main_subordMemPort_readData;
        end
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    assign subordMemPort_RVALID =
        subordMemPort_validResp && !main_subordMemPort_enWrite;
    assign subordMemPort_BVALID =
        subordMemPort_validResp &&  main_subordMemPort_enWrite;
    assign subordMemPort_BRESP = subordMemPort_resp_deniedWrite ? 2 : '0;
    assign subordMemPort_canAdvanceResp =
        main_subordMemPort_enWrite ? subordMemPort_BREADY
            : subordMemPort_RREADY;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    reg managMemPort_validAddrOnly;
    reg managMemPort_validWriteDataOnly;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire main_masterMemPort_ready =
        (managMemPort_AWREADY && managMemPort_WREADY)
            || (managMemPort_validAddrOnly      && managMemPort_AWREADY)
            || (managMemPort_validWriteDataOnly && managMemPort_WREADY);
    uwire main_masterMemPort_validWrite;
    APLIC_mainM#(options, APLICConfig, managMemPortAddrXW)
        main(
            .nReset                      (nReset                          ),
            .nLockReset                  (nLockReset                      ),
            .clock                       (clock                           ),
            .intrsInV                    ({intrsInV[numISources:1], 1'b0} ),
            .subordMemPort_nextBusy      (main_subordMemPort_nextBusy     ),
            .subordMemPort_validOp       (main_subordMemPort_validOp      ),
            .subordMemPort_domainNum     (main_subordMemPort_domainNum    ),
            .subordMemPort_enWrite       (main_subordMemPort_enWrite      ),
            .subordMemPort_addr          (main_subordMemPort_addr         ),
            .subordMemPort_writeData     (main_subordMemPort_writeData    ),
            .subordMemPort_validReadResp (main_subordMemPort_validReadResp),
            .subordMemPort_readData      (main_subordMemPort_readData     ),
            .harts_intrs                 (harts_intrs                     ),
            .harts_topMIntrPrio          (harts_topMIntrPrio              ),
            .harts_topSIntrPrio          (harts_topSIntrPrio              ),
            .masterMemPort_ready         (main_masterMemPort_ready        ),
            .masterMemPort_validWrite    (main_masterMemPort_validWrite   ),
            .masterMemPort_addr          (managMemPort_AWADDR             ),
            .masterMemPort_writeData     (managMemPort_WDATA              )
        );
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    always_ff @(negedge nReset, posedge clock) begin
        if (!nReset) begin
            managMemPort_validAddrOnly      <= '0;
            managMemPort_validWriteDataOnly <= '0;
        end else begin
            managMemPort_validAddrOnly <=
                !managMemPort_AWREADY
                    && ((managMemPort_AWVALID && managMemPort_WREADY)
                            || managMemPort_validAddrOnly);
            managMemPort_validWriteDataOnly <=
                !managMemPort_WREADY
                    && ((managMemPort_WVALID && managMemPort_AWREADY)
                            || managMemPort_validWriteDataOnly);
        end
    end
    assign managMemPort_AWVALID =
        main_masterMemPort_validWrite && !managMemPort_validWriteDataOnly;
    assign managMemPort_WVALID =
        main_masterMemPort_validWrite && !managMemPort_validAddrOnly;
    assign managMemPort_WSTRB = '1;

endmodule

