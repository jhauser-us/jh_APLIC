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
| Supports up to the following maximums:
|     interrupt domains:     127
|     harts:              16,384
| When direct delivery mode is implemented, the maximum number of harts per
| interrupt domain is 512.
|   An access through the subordinate memory port ('subordMemPort') always
| causes 'subordMemPort_nextBusy' to be asserted for one clock cycle, and
| sometimes two cycles.  Consequently, register accesses cannot occur back-to-
| back but at the maximum rate of one access every two or three clock cycles.
*----------------------------------------------------------------------------*/

module
    APLIC_mainM#(
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        int options,
        APLICPkg::configT APLICConfig,
        int masterMemPortAddrXW,
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
            (APLICConfig.intrPrioW < 1) ? 1 : APLICConfig.intrPrioW
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
        output uwire subordMemPort_nextBusy,
        input uwire subordMemPort_validOp,
        input uwire [(domainNumXW - 1):0] subordMemPort_domainNum,
        input uwire subordMemPort_enWrite,
        input uwire [12:0] subordMemPort_addr,
        input uwire [31:0] subordMemPort_writeData,
        output uwire subordMemPort_validReadResp,
        output uwire [31:0] subordMemPort_readData,
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
        input uwire masterMemPort_ready,
        output uwire masterMemPort_validWrite,
        output uwire [(masterMemPortAddrXW - 1):0] masterMemPort_addr,
        output uwire [31:0] masterMemPort_writeData
    );

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam int maxHartIndex = APLICConfig.harts.maxHartIndex;
    localparam int numDomains   = APLICConfig.numDomains;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    if (numISources > 1023) begin
        $fatal(0, "APLIC sources config has number of sources > 1023.");
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        bit anyImproperValidHartIndexRanges(APLICPkg::configT APLICConfig);
        for (
            int n = 0; n < APLICConfig.harts.numValidHartIndexRanges; n += 1
        ) begin
            int firstHartIndex =
                APLICConfig.harts.validHartIndexRanges[n].first;
            int lastHartIndex = APLICConfig.harts.validHartIndexRanges[n].last;
            if (
                (firstHartIndex < 0) || (firstHartIndex > lastHartIndex)
                    || (lastHartIndex > maxHartIndex)
            ) begin
                return 1;
            end
        end
        return 0;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        bit anyOverlappingValidHartIndexRanges(APLICPkg::configT APLICConfig);
        for (
            int n = 1; n < APLICConfig.harts.numValidHartIndexRanges; n += 1
        ) begin
            if (
                APLICConfig.harts.validHartIndexRanges[n].first
                    <= APLICConfig.harts.validHartIndexRanges[n - 1].last
            ) begin
                return 1;
            end
        end
        return 0;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam bit implLittleEndian = APLICConfig.harts.implLittleEndian;
    localparam bit implBigEndian    = APLICConfig.harts.implBigEndian;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        bit anyDomainsImplDeliverDirect(APLICPkg::configT APLICConfig);
        for (int d = 0; d < numDomains; d += 1) begin
            if (APLICConfig.domains[d].implDeliverDirect) return 1;
        end
        return 0;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        bit anyDomainsImplDeliverMSI(APLICPkg::configT APLICConfig);
        for (int d = 0; d < numDomains; d += 1) begin
            if (APLICConfig.domains[d].implDeliverMSI) return 1;
        end
        return 0;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam bit implDeliverDirect =
        anyDomainsImplDeliverDirect(APLICConfig);
    localparam bit implDeliverMSI = anyDomainsImplDeliverMSI(APLICConfig);
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    if (APLICConfig.harts.numValidHartIndexRanges < 1) begin
        $fatal(0, "APLIC harts config has no valid hart indices.");
    end else if (anyImproperValidHartIndexRanges(APLICConfig)) begin
        $fatal(
            0, "APLIC harts config has an improper range for valid indices."
        );
    end else if (anyOverlappingValidHartIndexRanges(APLICConfig)) begin
        $fatal(
            0, "APLIC harts config has overlapping ranges for valid indices."
        );
    end
    if (!implLittleEndian && !implBigEndian) begin
        $fatal(
            0,
           "APLIC harts config specifies neither little-endian nor big-endian."
        );
    end
    if (implDeliverMSI) begin
        if (APLICConfig.harts.maxGEILEN < 0) begin
            $fatal(0, "APLIC harts config has maximum GEILEN < 0.");
        end
        if (APLICConfig.harts.maxGEILEN > 63) begin
            $fatal(0, "APLIC harts config has maximum GEILEN > 63.");
        end
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/

//*** CHECK DOMAIN CONFIGS.

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    if (implDeliverDirect) begin
        if (intrPrioW > 8) begin
            $fatal(0, "APLIC config has interrupt priority width > 8.");
        end
    end
    if (implDeliverMSI) begin
        if (APLICConfig.EIIDW < 6) begin
            $fatal(0, "APLIC config has EIID width < 6.");
        end
        if (APLICConfig.EIIDW > 11) begin
            $fatal(0, "APLIC config has EIID width > 11.");
        end
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire regs_readWait;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    assign subordMemPort_nextBusy = subordMemPort_validOp || regs_readWait;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire regs_domain_BE;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire regs_doAccess = subordMemPort_validOp;
    uwire regs_doRead   = subordMemPort_validOp && !subordMemPort_enWrite;
    uwire regs_doWrite  = subordMemPort_validOp &&  subordMemPort_enWrite;
    uwire [(domainNumXW - 1):0] regs_domainNum =
        (numDomains < 2) ? '0 : subordMemPort_domainNum;
    uwire regs_validDomain = (regs_domainNum < numDomains);
    uwire [12:0] regs_addr = subordMemPort_addr;
    uwire regs_selGroup_sourcecfg = (regs_addr>>10 == 'h0000>>12);
    uwire [31:0] regs_selV_32 = 1'b1<<regs_addr[4:0];
    uwire [31:0] regs_sel32V_1024 = 1'b1<<regs_addr[9:5];
    uwire [1023:0] regs_selV_1024;
    for (genvar n = 0; n < 32; n += 1) begin
        assign regs_selV_1024[(n*32 + 31):n*32] =
            regs_sel32V_1024[n] ? regs_selV_32 : '0;
    end
    uwire [31:0] regs_writeData_LE = subordMemPort_writeData;
    uwire [31:0] regs_writeData =
          (implLittleEndian && (!implBigEndian || !regs_domain_BE)
               ? regs_writeData_LE : '0)
        | (implBigEndian && (!implLittleEndian || regs_domain_BE)
               ? {regs_writeData_LE[7:0], regs_writeData_LE[15:8],
                  regs_writeData_LE[23:16], regs_writeData_LE[31:24]}
               : '0);
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire doClearIP;
    uwire [9:0] clearIP_iSourceNum;
    /*------------------------------------------------------------------------
    | An "iVReg" (interrupt bit-vector register) is 32 bits of rectified
    | inputs, or 32 bits of interrupt-pending or interrupt-enable bits,
    | corresponding to one of the 32-bit registers 'setip', 'in_clrip',
    | 'setie', or 'clrie'.  Besides direct writes to these registers and
    | their "by-source-number" variants ('setipnum', etc.), interrupt-pending
    | and interrupt-enable bits may also need to be cleared on a write to a
    | 'sourcecfg' register, or when an interrupt is claimed or is forwarded by
    | MSI.
    *------------------------------------------------------------------------*/
    uwire iVReg_doAccessByNum_not_xe =
        regs_doAccess && !regs_addr[12] && regs_addr[10] && regs_addr[5];
    uwire iVReg_doAccessByNum_xe = regs_doAccess && (regs_addr[12:11] == 'b01);
    uwire [9:0] iVReg_zISourceNum =
          (regs_doAccess && (regs_addr[12:10] == 'b000)  // sourcecfg
               ? regs_addr[9:0] : '0)
        | (   (iVReg_doAccessByNum_not_xe        // set/clripnum, set/clrienum
                   && implLittleEndian && (!implBigEndian || !regs_domain_BE))
           || (iVReg_doAccessByNum_xe && (!implBigEndian || !regs_addr[0]))
                                                 // setipnum_le
               ? regs_writeData_LE[9:0] : '0)
        | (   (iVReg_doAccessByNum_not_xe        // set/clripnum, set/clrienum
                   && implBigEndian && (!implLittleEndian || regs_domain_BE))
           || (iVReg_doAccessByNum_xe && implBigEndian && regs_addr[0])
                                                 // setipnum_be
               ? {regs_writeData_LE[17:16], regs_writeData_LE[31:24]} : '0)
        | (doClearIP ? clearIP_iSourceNum : '0);
    uwire [31:0] selIVRegV =
        1'b1<<((regs_doAccess && (regs_addr[12:10] == 'b001) && !regs_addr[5]
                                              // setip, in_clrip, setie, clrie
                    ? regs_addr[4:0] : '0)
                   | iVReg_zISourceNum[9:5]);
    /*------------------------------------------------------------------------
    | For register accesses, module 'APLIC_iDomainsM' supplies
    | 'regs_domain_BE' (big-endian) and 'regs_domain_DM' (delivery mode) from
    | the 'domaincfg' of the accessed domain.  For accesses to a 'sourcecfg'
    | or 'target' register specifically, the module also supplies
    | 'regs_single_validInDomain', indicating whether the 'sourcecfg' or
    | 'target' register is valid in this domain; and for accesses to 'setip',
    | 'in_clrip', 'setie', or 'clrie', it similarly supplies the 32-bit vector
    | 'regs_setclr_validInDomainV'.
    *------------------------------------------------------------------------*/
    uwire regs_domain_DM;
    uwire [31:0] regs_zReadData_iDomains;
    uwire regs_single_validInDomain;
    uwire regs_doWriteISourceMode;
    uwire [31:0] regs_setclr_validInDomainV;
    uwire regs_doInitISourceTarget;
    uwire [(domainNumXW - 1):0] regs_initISourceTarget_domainNum;
    uwire domains_DM[numDomains];
    uwire domains_IE[numDomains];
    uwire [(domainNumXW - 1):0] iSources_domainNum[numISources + 1];
    APLIC_iDomainsM#(options, APLICConfig)
        iDomains(
            nReset,
            clock,
            regs_validDomain,
            regs_domainNum,
            regs_selGroup_sourcecfg,
            regs_doWrite,
            regs_selV_1024,
            regs_writeData[10:0],
            selIVRegV,
            regs_domain_BE,
            regs_domain_DM,
            regs_zReadData_iDomains,
            regs_single_validInDomain,
            regs_doWriteISourceMode,
            regs_setclr_validInDomainV,
            regs_doInitISourceTarget,
            regs_initISourceTarget_domainNum,
            domains_DM,
            domains_IE,
            iSources_domainNum
        );
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [31:0] regs_zReadData_iSources;
    uwire holdIPs;
    uwire [numISources:0] iSourcesV_active;
    uwire [numISources:0] iSourcesV_signal;
    APLIC_iSourcesM#(options, APLICConfig, implDeliverDirect, implDeliverMSI)
        iSources(
            nReset,
            clock,
            intrsInV,
            domains_DM,
            iSources_domainNum,
            regs_selGroup_sourcecfg,
            regs_doWrite,
            regs_addr,
            regs_selV_32,
            regs_selV_1024,
            regs_domain_BE,
            regs_writeData_LE,
            regs_writeData,
            regs_single_validInDomain,
            regs_doWriteISourceMode,
            regs_setclr_validInDomainV,
            holdIPs,
            doClearIP,
            selIVRegV,
            iVReg_zISourceNum[4:0],
            regs_zReadData_iSources,
            iSourcesV_active,
            iSourcesV_signal
        );
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [31:0] regs_zReadData_deliverIntrs;
    APLIC_deliverIntrsM#(
        APLICConfig, implDeliverDirect, implDeliverMSI, masterMemPortAddrXW
    ) deliverIntrs(
            nReset,
            nLockReset,
            clock,
            domains_DM,
            domains_IE,
            iSources_domainNum,
            iSourcesV_active,
            iSourcesV_signal,
            regs_validDomain,
            regs_domainNum,
            regs_doAccess,
            regs_doRead,
            regs_doWrite,
            regs_addr,
            regs_sel32V_1024,
            regs_selV_1024,
            regs_domain_DM,
            regs_writeData,
            regs_single_validInDomain,
            regs_doInitISourceTarget,
            regs_initISourceTarget_domainNum,
            regs_readWait,
            regs_zReadData_deliverIntrs,
            harts_intrs,
            harts_topMIntrPrio,
            harts_topSIntrPrio,
            holdIPs,
            doClearIP,
            clearIP_iSourceNum,
            masterMemPort_ready,
            masterMemPort_validWrite,
            masterMemPort_addr,
            masterMemPort_writeData
        );
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    reg regs_delayedReadResp;
    uwire regs_activeRead =
        (subordMemPort_validOp && !subordMemPort_enWrite)
            || regs_delayedReadResp;
    always_ff @(negedge nReset, posedge clock) begin
        if (!nReset) begin
            regs_delayedReadResp <= '0;
        end else begin
            regs_delayedReadResp <= regs_activeRead && regs_readWait;
        end
    end
    assign subordMemPort_validReadResp = regs_activeRead && !regs_readWait;
    uwire [31:0] regs_readData =
        regs_zReadData_iDomains | regs_zReadData_iSources
            | regs_zReadData_deliverIntrs;
    assign subordMemPort_readData =
          (implLittleEndian && (!implBigEndian || !regs_domain_BE)
               ? regs_readData : '0)
        | (implBigEndian && (!implLittleEndian || regs_domain_BE)
               ? {regs_readData[7:0], regs_readData[15:8],
                  regs_readData[23:16], regs_readData[31:24]}
               : '0);

endmodule

