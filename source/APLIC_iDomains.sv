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
*----------------------------------------------------------------------------*/

module
    APLIC_iDomainsM#(
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        int options,
        APLICPkg::configT APLICConfig,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam int numISources = APLICConfig.sources.num,
        localparam int numDomains = APLICConfig.numDomains,
        localparam int domainNumXW =
            (numDomains < 2) ? 1 : $clog2(APLICConfig.numDomains)
    ) (
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire nReset,
        input uwire clock,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire regs_validDomain,
        input uwire [(domainNumXW - 1):0] regs_domainNum,
        input uwire regs_selGroup_sourcecfg,
        input uwire regs_doWrite,
        input uwire [1023:0] regs_selV_1024,
        input uwire [10:0] regs_writeData,
        input uwire [31:0] regs_setclr_selISourceVRegV,
        output uwire regs_domain_BE,
        output uwire regs_domain_DM,
        output uwire [31:0] regs_zReadData,
        output uwire regs_single_validInDomain,
        output uwire regs_doWriteISourceMode,
        output uwire [31:0] regs_setclr_validInDomainV,
        output uwire regs_doInitISourceTarget,
        output uwire [(domainNumXW - 1):0] regs_initISourceTarget_domainNum,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        output uwire domains_DM[numDomains],
        output reg domains_IE[numDomains],
        output uwire [(domainNumXW - 1):0] iSources_domainNum[numISources + 1]
    );

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam int numISourceVRegs = numISources/32 + 1;
    localparam bit implLittleEndian = APLICConfig.harts.implLittleEndian;
    localparam bit implBigEndian    = APLICConfig.harts.implBigEndian;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/

//*** SANITY CHECKS ON DOMAIN CONFIGS.

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        bit orderedLtDomain(
            APLICPkg::configT APLICConfig, int domainNumA, int domainNumB);
        int d = domainNumB;
        if (d <= domainNumA) return 0;
//*** NEED TO CHECK EARLIER FOR DOMAIN LOOPS!
        forever begin
            d = APLICConfig.domains[d].parentNum;
            if (d == domainNumA) return 1;
            if (d < domainNumA) return 0;
        end
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        int numChildrenOfDomain(APLICPkg::configT APLICConfig, int domainNum);
        int n = 0;
        for (int d = domainNum + 1; d < numDomains; d += 1) begin
            if (APLICConfig.domains[d].parentNum == domainNum) n = n + 1;
        end
        return n;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic int maxNumDomainChildren(APLICPkg::configT APLICConfig);
        int n = 0;
        for (int d = 0; d < numDomains; d += 1) begin
            int numChildren = numChildrenOfDomain(APLICConfig, d);
            if (n < numChildren) n = numChildren;
        end
        return n;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam int maxChildIndex = maxNumDomainChildren(APLICConfig) - 1;
    localparam int childIndexXW =
        (maxChildIndex < 1) ? 1 : $clog2(maxChildIndex + 1);
    localparam int extNumDomainChildren =
        (maxChildIndex < 1) ? 1 : 1<<childIndexXW;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        int childDomainFromIndex(
            APLICPkg::configT APLICConfig, int domainNum, int childIndex);
        int c = childIndex;
        for (int d = domainNum + 1; d < numDomains; d += 1) begin
            if (APLICConfig.domains[d].parentNum == domainNum) begin
                if (c == 0) return d;
                c -= 1;
            end
        end
        return 0;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        int childIndexAtAscendantDomain(
            APLICPkg::configT APLICConfig, int domainNumA, int domainNumB);
        int c = 0;
        for (int d = domainNumA + 1; d <= domainNumB; d += 1) begin
            if (APLICConfig.domains[d].parentNum == domainNumA) begin
                if (
                    (d == domainNumB)
                        || orderedLtDomain(APLICConfig, d, domainNumB)
                ) begin
                    return c;
                end
                c = c + 1;
            end
        end
        return 0;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire domains2_orderedLt[numDomains][numDomains];
    uwire domains_isLeaf[numDomains];
    uwire [(domainNumXW - 1):0]
        domains_childDomains[numDomains][extNumDomainChildren];
    uwire [(childIndexXW - 1):0] domains2_childIndex[numDomains][numDomains];
    for (genvar d = 0; d < numDomains; d += 1) begin
        for (genvar dB = 0; dB < numDomains; dB += 1) begin
            localparam bit orderedLt = orderedLtDomain(APLICConfig, d, dB);
            assign domains2_orderedLt[d][dB] = orderedLt;
            assign domains2_childIndex[d][dB] =
                orderedLt ? childIndexAtAscendantDomain(APLICConfig, d, dB)
                    : 'x;
        end
        localparam int numChildren = numChildrenOfDomain(APLICConfig, d);
        assign domains_isLeaf[d] = (numChildren == 0);
        if (numChildren == 0) begin
            for (genvar c = 0; c < extNumDomainChildren; c += 1) begin
                assign domains_childDomains[d][c] = 'x;
            end
        end else begin
            for (genvar c = 0; c < numChildren; c += 1) begin
                assign domains_childDomains[d][c] =
                    childDomainFromIndex(APLICConfig, d, c);
            end
            localparam int excessChildIndexMask =
                (1<<($clog2(numChildren + 1) - 1)) - 1;
            for (
                genvar c = numChildren; c < extNumDomainChildren; c += 1
            ) begin
                assign domains_childDomains[d][c] =
                    childDomainFromIndex(
                        APLICConfig, d, c & excessChildIndexMask);
            end
        end
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire regs_doWrite_domaincfg =
        regs_doWrite && regs_selGroup_sourcecfg && regs_selV_1024[0];
    uwire [(domainNumXW - 1):0] regs_single_zDomainNum;
    APLIC_selectOneM#(numISources, domainNumXW)
        selISourceDomainNum(
            regs_selV_1024[numISources:1],
            iSources_domainNum[1:numISources],
            regs_single_zDomainNum
        );
    assign regs_single_validInDomain =
        regs_validDomain && (regs_single_zDomainNum == regs_domainNum);
    uwire regs_single_isDelegated =
        regs_validDomain
            && domains2_orderedLt[regs_domainNum][regs_single_zDomainNum];
    uwire [(childIndexXW - 1):0] regs_single_delegated_childIndex =
        (maxChildIndex < 1) ? '0
            : domains2_childIndex[regs_domainNum][regs_single_zDomainNum];
    uwire [(childIndexXW - 1):0] regs_write_sourcecfg_newChildIndex =
        (maxChildIndex < 1) ? '0 : regs_writeData[9:0];
    uwire regs_write_sourcecfg_inAscendantDomain =
        regs_single_isDelegated
            && (!regs_writeData[10]
                    || (regs_single_delegated_childIndex
                            != regs_write_sourcecfg_newChildIndex));
    uwire regs_doWriteISourceDomainNum =
        regs_doWrite && regs_selGroup_sourcecfg
            && ((regs_single_validInDomain && !domains_isLeaf[regs_domainNum])
                    || regs_write_sourcecfg_inAscendantDomain);
    uwire [(domainNumXW - 1):0] regs_write_sourcecfg_newDomainNum =
        regs_writeData[10]
            ? domains_childDomains[regs_domainNum][
                  regs_write_sourcecfg_newChildIndex]
            : regs_domainNum;
    assign regs_doWriteISourceMode =
        regs_doWrite && regs_selGroup_sourcecfg
            && (regs_single_validInDomain
                    || regs_write_sourcecfg_inAscendantDomain);
    assign regs_doInitISourceTarget =
        regs_doWriteISourceDomainNum
            && (regs_single_zDomainNum != regs_write_sourcecfg_newDomainNum);
    assign regs_initISourceTarget_domainNum =
        regs_write_sourcecfg_newDomainNum;
    /*------------------------------------------------------------------------
    | The 'domaincfg' registers.
    *------------------------------------------------------------------------*/
    reg domains_BE[numDomains];
    for (genvar d = 0; d < numDomains; d += 1) begin :Domain
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam bit implDeliverDirect =
            APLICConfig.domains[d].implDeliverDirect;
        localparam bit implDeliverMSI = APLICConfig.domains[d].implDeliverMSI;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire doWrite_domaincfg =
            regs_doWrite_domaincfg && (regs_domainNum == d);
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        if (implLittleEndian && implBigEndian) begin
            always_ff @(negedge nReset, posedge clock) begin
                if (!nReset) begin
                    domains_BE[d] <= '0;
                end else begin
                    if (doWrite_domaincfg) domains_BE[d] <= regs_writeData[0];
                end
            end
        end else begin
            assign domains_BE[d] = implBigEndian;
        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        reg rDM;
        if (implDeliverDirect && implDeliverMSI) begin
            always_ff @(negedge nReset, posedge clock) begin
                if (!nReset) begin
                    rDM <= '0;
                end else begin
                    if (doWrite_domaincfg) rDM <= regs_writeData[2];
                end
            end
            assign domains_DM[d] = rDM;
        end else begin
            assign domains_DM[d] = implDeliverMSI;
        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        always_ff @(negedge nReset, posedge clock) begin
            if (!nReset) begin
                domains_IE[d] <= '0;
            end else begin
                if (doWrite_domaincfg) domains_IE[d] <= regs_writeData[8];
            end
        end
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    assign regs_domain_BE = domains_BE[regs_domainNum];
    assign regs_domain_DM = domains_DM[regs_domainNum];
    /*------------------------------------------------------------------------
    | The assignments of interrupt sources to domains are encoded in the
    | 'iSources_domainNum' array.  Ensure that the entries of this array never
    | exceed the value numDomains - 1.
    *------------------------------------------------------------------------*/
    assign iSources_domainNum[0] = 'x;
    if (numDomains < 2) begin
        for (genvar s = 1; s <= numISources; s += 1) begin
            assign iSources_domainNum[s] = '0;
        end
    end else begin :ISources
        reg [(domainNumXW - 1):0] rDomainNums[numISources + 1];
        for (genvar s = 1; s <= numISources; s += 1) begin
            always_ff @(negedge nReset, posedge clock) begin
                if (!nReset) begin
                    rDomainNums[s] <= '0;
                end else begin
                    if (
                        regs_doWriteISourceDomainNum && regs_selV_1024[s]
                    ) begin
                        rDomainNums[s] <= regs_write_sourcecfg_newDomainNum;
                    end
                end
            end
            if (domainNumXW < 2) begin
                assign iSources_domainNum[s] = rDomainNums[s];
            end else begin
                assign iSources_domainNum[s][domainNumXW - 1] =
                    (rDomainNums[s] < numDomains)
                        ? rDomainNums[s][domainNumXW - 1] : '0;
                assign iSources_domainNum[s][(domainNumXW - 2):0] =
                    rDomainNums[s][(domainNumXW - 2):0];
            end
        end
    end
    /*------------------------------------------------------------------------
    | Value for a read of a 'domaincfg' register, or of a 'sourcecfg' register
    | for a delegated source.  (For a non-delegated source, a 'sourcecfg'
    | value comes from module 'APLIC_iSourcesM'.)
    *------------------------------------------------------------------------*/
    assign regs_zReadData =
          (regs_validDomain && regs_selGroup_sourcecfg && regs_selV_1024[0]
               ? 1'b1<<31 | domains_IE[regs_domainNum]<<8 | regs_domain_DM<<2
                     | regs_domain_BE
               : '0)
        | (regs_selGroup_sourcecfg && regs_single_isDelegated
               ? 1'b1<<10 | regs_single_delegated_childIndex : '0);
    /*------------------------------------------------------------------------
    | Determine 'regs_setclr_validInDomainV'.
    *------------------------------------------------------------------------*/
    uwire [domainNumXW:0] iSourcesBy32s_xDomainNum[32][numISourceVRegs];
    for (genvar s = 0; s <= (numISources | 31); s += 1) begin
        if ((s > 0) && (s <= numISources)) begin
            assign iSourcesBy32s_xDomainNum[s & 31][s/32] =
                iSources_domainNum[s];
        end else begin
            assign iSourcesBy32s_xDomainNum[s & 31][s/32] = 1'b1<<domainNumXW;
        end
    end
    for (genvar i = 0; i < 32; i += 1) begin :Bit_ISourceVReg
        uwire [domainNumXW:0] xDomainNum;
        APLIC_selectOneM#(numISourceVRegs, domainNumXW + 1)
            selXDomain(
                regs_setclr_selISourceVRegV[(numISourceVRegs - 1):0],
                iSourcesBy32s_xDomainNum[i],
                xDomainNum
            );
        assign regs_setclr_validInDomainV[i] = (xDomainNum == regs_domainNum);
    end

endmodule

