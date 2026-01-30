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
    APLIC_deliverIntrsM#(
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        APLICPkg::configT APLICConfig,
        bit implDeliverDirect,
        bit implDeliverMSI,
        int masterMemPortAddrXW,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam int numISources = APLICConfig.sources.num,
        localparam int numDomains = APLICConfig.numDomains,
        localparam int domainNumXW = (numDomains < 2) ? 1 : $clog2(numDomains),
        localparam int deliverDirect_firstHartIndex =
            APLICConfig.deliverDirectHartIndexRange.first,
        localparam int deliverDirect_lastHartIndex =
            APLICConfig.deliverDirectHartIndexRange.last,
        localparam int outTopIntrPrios_firstHartIndex =
            APLICConfig.outTopIntrPriosHartIndexRange.first,
        localparam int outTopIntrPrios_lastHartIndex =
            APLICConfig.outTopIntrPriosHartIndexRange.last,
        localparam int intrPrioW = APLICConfig.intrPrioW,
        localparam int intrPrioXW = (intrPrioW < 1) ? 1 : intrPrioW
    ) (
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire nReset,
        input uwire nLockReset,
        input uwire clock,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire domains_DM[numDomains],
        input uwire domains_IE[numDomains],
        input uwire [(domainNumXW - 1):0] iSources_domainNum[numISources + 1],
        input uwire [numISources:0] iSourcesV_active,
        input uwire [numISources:0] iSourcesV_signal,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire regs_validDomain,
        input uwire [(domainNumXW - 1):0] regs_domainNum,
        input uwire regs_doAccess,
        input uwire regs_doRead,
        input uwire regs_doWrite,
        input uwire [12:0] regs_addr,
        input uwire [31:0] regs_sel32V_1024,
        input uwire [1023:0] regs_selV_1024,
        input uwire regs_domain_DM,
        input uwire [31:0] regs_writeData,
        input uwire regs_single_validInDomain,
        input uwire regs_doInitISourceTarget,
        input uwire [(domainNumXW - 1):0] regs_initISourceTarget_domainNum,
        output uwire regs_readWait,
        output uwire [31:0] regs_zReadData,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        output reg [1:0]
            harts_intrs[
                deliverDirect_firstHartIndex:deliverDirect_lastHartIndex],
        output reg [(intrPrioXW - 1):0]
            harts_topMIntrPrio[
                outTopIntrPrios_firstHartIndex:outTopIntrPrios_lastHartIndex],
        output reg [(intrPrioXW - 1):0]
            harts_topSIntrPrio[
                outTopIntrPrios_firstHartIndex:outTopIntrPrios_lastHartIndex],
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        output uwire holdIPs,
        output uwire doClearIP,
        output uwire [9:0] clearIP_iSourceNum,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire masterMemPort_ready,
        output reg masterMemPort_validWrite,
        output reg [(masterMemPortAddrXW - 1):0] masterMemPort_addr,
        output reg [31:0] masterMemPort_writeData
    );

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        int maxNumHartIndicesInDomains(APLICPkg::configT APLICConfig);
        int n = deliverDirect_lastHartIndex - deliverDirect_firstHartIndex + 1;
        for (int d = 0; d < numDomains; d += 1) begin
            int numHartIndices = APLICConfig.domains[d].numHartIndices;
            if (n < numHartIndices) n = numHartIndices;
        end
        return n;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam int intrIdentW = $clog2(numISources + 1);
    localparam int maxHartIndex = APLICConfig.harts.maxHartIndex;
    localparam int hartIndexXW =
        (maxHartIndex <= 0) ? 1 : $clog2(maxHartIndex + 1);
    localparam int encHartIndexW =
        $clog2(maxNumHartIndicesInDomains(APLICConfig));
    localparam int encHartIndexXW = (encHartIndexW < 1) ? 1 : encHartIndexW;
    localparam int upperDomHartIndexXW =
        (hartIndexXW <= encHartIndexW) ? 1 : hartIndexXW - encHartIndexW;
    localparam int numEncHartIndices = 1<<encHartIndexW;
    localparam int encHartIndexMask = numEncHartIndices - 1;
    localparam bit implOutTopMIntrPrios = APLICConfig.implOutTopMIntrPrios;
    localparam bit implOutTopSIntrPrios = APLICConfig.implOutTopSIntrPrios;
    localparam int maxGEILEN = APLICConfig.harts.maxGEILEN;
    localparam bit implGuestIndex = implDeliverMSI && (maxGEILEN > 0);
    localparam int guestIndexXW = implGuestIndex ? $clog2(maxGEILEN + 1) : 1;
    localparam bit implTargetArg = implDeliverMSI || (intrPrioW > 1);
    localparam int EIIDW = APLICConfig.EIIDW;
    localparam int EIIDXW = (EIIDW < 1) ? 1 : EIIDW;
    localparam int targetArgW =
        implDeliverMSI && (EIIDW > intrPrioW) ? EIIDW : intrPrioW;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        int firstDomHartIndexInDomain(
            APLICPkg::configT APLICConfig, int domainNum);
        return
            (APLICConfig.domains[domainNum].privLevel == APLICPkg::privLevel_M)
                ? APLICConfig.domains[domainNum].firstHartIndex
                : APLICConfig.domains[domainNum].S_perceivedFirstHartIndex;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        bit isValidHartIndex(APLICPkg::configT APLICConfig, int hartIndex);
        for (
            int n = 0; n < APLICConfig.harts.numValidHartIndexRanges; n += 1
        ) begin
            if (
                   (APLICConfig.harts.validHartIndexRanges[n].first
                        <= hartIndex)
                && (hartIndex
                        <= APLICConfig.harts.validHartIndexRanges[n].last)
            ) begin
                return 1;
            end
        end
        return 0;
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        int breakEncHartIndexForDomain(
            APLICPkg::configT APLICConfig, int domainNum);
        int encH =
            firstDomHartIndexInDomain(APLICConfig, domainNum)
                & encHartIndexMask;
        if (encH == 0) begin
            return 0;
        end else begin
            int b = encH;
            int h = APLICConfig.domains[domainNum].firstHartIndex;
            int stopHartIndex =
                h + APLICConfig.domains[domainNum].numHartIndices;
            h += numEncHartIndices;
            forever begin
                h -= 1;
                if (
                    (h < stopHartIndex) && isValidHartIndex(APLICConfig, h)
                ) break;
                encH -= 1;
                if (encH == 0) return 0;
                if ((b & -b) < (encH & -encH)) b = encH;
            end
            return b;
        end
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    function automatic
        int defaultDomHartIndexForDomain(
            APLICPkg::configT APLICConfig, int domainNum);

//*** TEMPORARY:
        return firstDomHartIndexInDomain(APLICConfig, domainNum);

    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [(numDomains - 1):0]
        encHartIndices_validInDomainsV[numEncHartIndices];
    uwire [(encHartIndexXW - 1):0] domains_defaultEncHartIndex[numDomains];
    typedef struct packed {
        logic level_S;
        logic [(upperDomHartIndexXW - 1):0] upperDomHartIndex;
        logic [(hartIndexXW - 1):0] hartIndexOffset;
    } domainTargetInfoT;
    uwire domainTargetInfoT domains_targetInfo[numDomains];
    for (genvar d = 0; d < numDomains; d += 1) begin :Domain
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam int firstDomHartIndex =
            firstDomHartIndexInDomain(APLICConfig, d);
        localparam int numHartIndices = APLICConfig.domains[d].numHartIndices;
        localparam int hartIndexOffset =
            APLICConfig.domains[d].firstHartIndex - firstDomHartIndex;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        for (genvar x = 0; x < numEncHartIndices; x += 1) begin
            localparam int domHartIndex = firstDomHartIndex + x;
            localparam int encHartIndex = domHartIndex & encHartIndexMask;
            assign encHartIndices_validInDomainsV[encHartIndex][d] =
                (x < numHartIndices)
                    && isValidHartIndex(
                           APLICConfig, domHartIndex + hartIndexOffset);
        end
        assign domains_defaultEncHartIndex[d] =
            defaultDomHartIndexForDomain(APLICConfig, d) & encHartIndexMask;
        assign domains_targetInfo[d] =
            '{ level_S:
                   (APLICConfig.domains[d].privLevel == APLICPkg::privLevel_S),
               upperDomHartIndex: firstDomHartIndex>>encHartIndexW,
               hartIndexOffset:   hartIndexOffset
             };
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [(numDomains - 1):0] regs_domainV = 1'b1<<regs_domainNum;
    uwire regs_selGroup_target = (regs_addr>>10 == 'h3000>>12);
    uwire [numISources:0] regs_selV_active =
        regs_selV_1024[numISources:0] & iSourcesV_active;
    uwire regs_sel_genmsi_activeInDomain =
        regs_validDomain && regs_selGroup_target && regs_selV_1024[0]
            && (regs_domain_DM == 1);
    uwire regs_sel_target_activeInDomain =
        regs_selGroup_target
            && regs_single_validInDomain && (|regs_selV_active);
    uwire regs_doReadGroup_target = regs_doRead && regs_selGroup_target;
    uwire regs_doWrite_target =
        regs_doWrite && regs_selGroup_target && regs_single_validInDomain;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
//*** ADDITIONALLY OPTIMIZE CASE THAT ALL ENCODED HART INDICES ARE VALID.
    uwire [(encHartIndexXW - 1):0] regs_defaultEncHartIndexInDomain;
    APLIC_selectOneM#(numDomains, encHartIndexXW)
        selDefaultEncHartIndexInDomain(
            regs_domainV,
            domains_defaultEncHartIndex,
            regs_defaultEncHartIndexInDomain
        );
    uwire [(encHartIndexXW - 1):0] regs_writeGroup_target_attemptEncHartIndex =
        regs_writeData[31:18];
    uwire [(encHartIndexXW - 1):0] regs_writeGroup_target_encHartIndex =
        |(regs_domainV
              & encHartIndices_validInDomainsV[
                    regs_writeGroup_target_attemptEncHartIndex])
            ? regs_writeGroup_target_attemptEncHartIndex
            : regs_defaultEncHartIndexInDomain;
    /*------------------------------------------------------------------------
    | The 'target' registers.
    *------------------------------------------------------------------------*/
    uwire [(encHartIndexXW - 1):0] iSource_target_newEncHartIndex =
        regs_selGroup_target ? regs_writeGroup_target_encHartIndex
            : domains_defaultEncHartIndex[regs_initISourceTarget_domainNum];
    reg [(encHartIndexXW - 1):0] iSources_target_encHartIndex[numISources + 1];
    reg [(guestIndexXW - 1):0] iSources_target_xGuestIndex[numISources + 1];
    reg [(targetArgW - 1):0] iSources_target_arg[numISources + 1];
    for (genvar s = 1; s <= numISources; s += 1) begin
        uwire doWriteTargetReg = regs_doWrite_target && regs_selV_active[s];
        if (numEncHartIndices > 1) begin
            always_ff @(negedge nReset, posedge clock) begin
                if (!nReset) begin
                    iSources_target_encHartIndex[s] <=
                        domains_defaultEncHartIndex[0];
                end else begin
                    if (
                        (regs_doInitISourceTarget && regs_selV_1024[s])
                            || doWriteTargetReg
                    ) begin
                        iSources_target_encHartIndex[s] <=
                            iSource_target_newEncHartIndex;
                    end
                end
            end
        end else begin
            assign iSources_target_encHartIndex[s] = '0;
        end
        if (implGuestIndex) begin
            always_ff @(posedge clock) begin
                if (doWriteTargetReg) begin
                    iSources_target_xGuestIndex[s] <= regs_writeData[17:12];
                end
            end
        end else begin
            assign iSources_target_xGuestIndex[s] = '0;
        end
        if (implTargetArg) begin
            always_ff @(posedge clock) begin
                if (doWriteTargetReg) begin
                    iSources_target_arg[s] <= regs_writeData[10:0];
                end
            end
        end else begin
            assign iSources_target_arg[s] = '0;
        end
    end
    /*------------------------------------------------------------------------
    | Components exclusive to direct delivery:
    |   * for each hart, the delivery control (IDC) structures for all
    |      relevant domains;
    |   * the interrupt signals to harts;
    |   * optionally, reports of the priorities of signaled interrupts.
    *------------------------------------------------------------------------*/
    uwire [31:0] regs_zReadData_IDC;
    reg deliverDirect_doClearIP;
    reg [(intrIdentW - 1):0] deliverDirect_intrIdent;
    if (implDeliverDirect) begin :DeliverDirect
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam int numDestHarts =
            deliverDirect_lastHartIndex - deliverDirect_firstHartIndex + 1;
        localparam int numXDestHarts = 1<<$clog2(numDestHarts);
        localparam int xDestHartMask = numXDestHarts - 1;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire [8:0] regs_IDCNum = regs_addr[11:3];
        uwire [(numDomains - 1):0] domainsV_regs_IDCNumInRange;
        for (genvar d = 0; d < numDomains; d += 1) begin
            localparam int hartIndexOffset =
                APLICConfig.domains[d].firstHartIndex
                    - firstDomHartIndexInDomain(APLICConfig, d);
            localparam int firstIDCNum =
                (deliverDirect_firstHartIndex >= hartIndexOffset)
                    ? deliverDirect_firstHartIndex - hartIndexOffset : 0;
            localparam int lastIDCNum =
                (deliverDirect_lastHartIndex >= hartIndexOffset)
                    ? deliverDirect_lastHartIndex - hartIndexOffset : 0;
            if (firstIDCNum == 0) begin
                assign domainsV_regs_IDCNumInRange[d] =
                    APLICConfig.domains[d].implDeliverDirect
                        && (regs_IDCNum <= lastIDCNum);
            end else begin
                assign domainsV_regs_IDCNumInRange[d] =
                    APLICConfig.domains[d].implDeliverDirect
                        && (firstIDCNum <= regs_IDCNum)
                        && (regs_IDCNum <= lastIDCNum);
            end
        end
        uwire regs_sel_IDC =
            regs_validDomain && (regs_domain_DM == 0) && regs_addr[12]
                && (|(regs_domainV & domainsV_regs_IDCNumInRange));
        uwire regs_doWrite_IDC = regs_doWrite && regs_sel_IDC;
        uwire [15:0] regs_IDCV_X = 1'b1<<{regs_addr[11:10], regs_addr[4:3]};
        uwire [511:0] regs_IDCV;
        for (genvar n = 0; n < 32; n += 1) begin
            assign
                {regs_IDCV[(384 + n*4 + 3):(384 + n*4)],
                 regs_IDCV[(256 + n*4 + 3):(256 + n*4)],
                 regs_IDCV[(128 + n*4 + 3):(128 + n*4)],
                 regs_IDCV[(      n*4 + 3):(      n*4)]} =
                    regs_sel32V_1024[n] ? regs_IDCV_X : '0;
        end
        uwire regs_IDC_sel_idelivery   = (regs_addr[2:0] == 0);
        uwire regs_IDC_sel_iforce      = (regs_addr[2:0] == 1);
        uwire regs_IDC_sel_ithreshold  = (regs_addr[2:0] == 2);
        uwire regs_IDC_sel_topi_claimi = (regs_addr[2:1] == 6>>1);
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire domains_enIntrDirect[numDomains];
        for (genvar d = 0; d < numDomains; d += 1) begin
            assign domains_enIntrDirect[d] =
                (domains_DM[d] == 0) && domains_IE[d];
        end
        uwire [(numDomains - 1):0] iSources_domainV[numISources + 1];
        uwire [(numEncHartIndices - 1):0]
            iSources_targetEncHartV[numISources + 1];
        assign iSources_domainV[0]        = 'x;
        assign iSources_targetEncHartV[0] = 'x;
        for (genvar s = 1; s <= numISources; s += 1) begin
            assign iSources_domainV[s] = 1'b1<<iSources_domainNum[s];
            assign iSources_targetEncHartV[s] =
                1'b1<<iSources_target_encHartIndex[s];
        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire domains_xIDCs_idelivery[numDomains][numXDestHarts];
        uwire domains_xIDCs_iforce[numDomains][numXDestHarts];
        uwire [(intrPrioW - 1):0]
            domains_xIDCs_ithreshold[numDomains][numXDestHarts];
        /*--------------------------------------------------------------------
        | The interrupt signals to harts ('harts_intrs').
        *--------------------------------------------------------------------*/
        for (genvar x = 0; x < numDestHarts; x += 1) begin
            localparam int h = deliverDirect_firstHartIndex + x;
            uwire [(numDomains - 1):0] MIntrFromDomainsV;
            uwire [(numDomains - 1):0] SIntrFromDomainsV;
            for (genvar d = 0; d < numDomains; d += 1) begin
                /*------------------------------------------------------------
                *------------------------------------------------------------*/
                localparam APLICPkg::privLevelT privLevel =
                    APLICConfig.domains[d].privLevel;
                localparam int hartIndexOffset =
                    APLICConfig.domains[d].firstHartIndex
                        - firstDomHartIndexInDomain(APLICConfig, d);
                localparam int encH = (h - hartIndexOffset) & encHartIndexMask;
                localparam int xIDCNum = encH & xDestHartMask;
                /*------------------------------------------------------------
                *------------------------------------------------------------*/
                uwire [(intrPrioW - 1):0] ithreshold =
                    domains_xIDCs_ithreshold[d][xIDCNum];
                uwire ithreshold_01 = (intrPrioW <= 1) || (ithreshold <= 1);
                uwire ithreshold_0 = ithreshold_01 && !ithreshold[0];
                uwire ithreshold_1 = ithreshold_01 &&  ithreshold[0];
                uwire [numISources:0] possSignalIntrsV;
                assign possSignalIntrsV[0] = '0;
                for (genvar s = 1; s <= numISources; s += 1) begin
                    uwire prioPassesThreshold;
                    if (intrPrioW > 1) begin
                        assign prioPassesThreshold =
                            ithreshold_0
                                || (iSources_target_arg[s][(intrPrioW - 1):0]
                                        < ithreshold);
                    end else begin
                        assign prioPassesThreshold = '1;
                    end
                    assign possSignalIntrsV[s] =
                        iSources_domainV[s][d]
                            && iSources_targetEncHartV[s][encH]
                            && prioPassesThreshold && iSourcesV_signal[s];
                end
                uwire intrOut =
                    APLICConfig.domains[d].implDeliverDirect
                        && domains_enIntrDirect[d]
                        && encHartIndices_validInDomainsV[encH][d]
                        && domains_xIDCs_idelivery[d][xIDCNum]
                        && (domains_xIDCs_iforce[d][xIDCNum]
                                || (!ithreshold_1 && (|possSignalIntrsV)));
                assign MIntrFromDomainsV[d] =
                    (privLevel == APLICPkg::privLevel_M) && intrOut;
                assign SIntrFromDomainsV[d] =
                    (privLevel == APLICPkg::privLevel_S) && intrOut;
            end
            always_ff @(posedge clock) begin
                harts_intrs[h] <= {(|SIntrFromDomainsV), (|MIntrFromDomainsV)};
            end
        end
        /*--------------------------------------------------------------------
        | The priorities of signaled interrupts to harts ('harts_topMIntrPrio'
        | and 'harts_topSIntrPrio').
        *--------------------------------------------------------------------*/
        for (
            genvar h = outTopIntrPrios_firstHartIndex;
            h <= outTopIntrPrios_lastHartIndex;
            h += 1
        ) begin

//*** TEMPORARY:
            assign harts_topMIntrPrio[h] = '0;
            assign harts_topSIntrPrio[h] = '0;

        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire regs_doClear_IDC_iforce;
        /*--------------------------------------------------------------------
        | The interrupt delivery control (IDC) structures:
        |   The writable registers, 'idelivery', 'iforce', and 'ithreshold'.
        *--------------------------------------------------------------------*/
        uwire [(numXDestHarts - 1):0] domains_regs_xIDCV[numDomains];
        for (genvar d = 0; d < numDomains; d += 1) begin :Domain_1
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            localparam int domain_firstHartIndex =
                APLICConfig.domains[d].firstHartIndex;
            localparam int domain_lastHartIndex =
                domain_firstHartIndex + APLICConfig.domains[d].numHartIndices
                    - 1;
            localparam int domain_hartIndexOffset =
                domain_firstHartIndex
                    - firstDomHartIndexInDomain(APLICConfig, d);
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            for (genvar x = 0; x < numXDestHarts; x += 1) begin :XDestHart
                localparam int h = deliverDirect_firstHartIndex + x;
                localparam int IDCNum = h - domain_hartIndexOffset;
                localparam int xIDCNum = IDCNum & xDestHartMask;
                if (
                    APLICConfig.domains[d].implDeliverDirect
                        && isValidHartIndex(APLICConfig, h)
                        && (domain_firstHartIndex <= h)
                        && (h <= domain_lastHartIndex)
                ) begin
                    reg r_idelivery;
                    reg r_iforce;
                    reg [(intrPrioW - 1):0] r_ithreshold;
                    always_ff @(posedge clock) begin
                        if (
                            regs_doWrite_IDC && regs_domainV[d]
                                && regs_IDCV[IDCNum] && regs_IDC_sel_idelivery
                        ) begin
                            r_idelivery <= regs_writeData[0];
                        end
                        if (
                            ((regs_doWrite_IDC && regs_IDC_sel_iforce)
                                 || regs_doClear_IDC_iforce)
                                && regs_domainV[d] && regs_IDCV[IDCNum]
                        ) begin
                            r_iforce <=
                                regs_doWrite_IDC ? regs_writeData[0] : '0;
                        end
                        if (
                            regs_doWrite_IDC && regs_domainV[d]
                                && regs_IDCV[IDCNum] && regs_IDC_sel_ithreshold
                        ) begin
                            r_ithreshold <= regs_writeData;
                        end
                    end
                    assign domains_regs_xIDCV[d][xIDCNum] = regs_IDCV[IDCNum];
                    assign domains_xIDCs_idelivery[d][xIDCNum]  = r_idelivery;
                    assign domains_xIDCs_iforce[d][xIDCNum]     = r_iforce;
                    assign domains_xIDCs_ithreshold[d][xIDCNum] = r_ithreshold;
                end else begin
                    assign domains_regs_xIDCV[d][xIDCNum]       = '0;
                    assign domains_xIDCs_idelivery[d][xIDCNum]  = '0;
                    assign domains_xIDCs_iforce[d][xIDCNum]     = '0;
                    assign domains_xIDCs_ithreshold[d][xIDCNum] = '0;
                end
            end
        end
        /*--------------------------------------------------------------------
        | The interrupt delivery control (IDC) structures:
        |   For any read of an IDC, select the values of the IDC's
        | 'idelivery', 'iforce', and 'ithreshold' registers.
        *--------------------------------------------------------------------*/
        uwire domains_IDC_idelivery[numDomains];
        uwire domains_IDC_iforce[numDomains];
        uwire [(intrPrioW - 1):0] domains_IDC_ithreshold[numDomains];
        for (genvar d = 0; d < numDomains; d += 1) begin :Domain_2
            APLIC_selectOneM#(numXDestHarts, 1)
                selIDC_idelivery(
                    domains_regs_xIDCV[d],
                    domains_xIDCs_idelivery[d],
                    domains_IDC_idelivery[d]
                );
            APLIC_selectOneM#(numXDestHarts, 1)
                selIDC_iforce(
                    domains_regs_xIDCV[d],
                    domains_xIDCs_iforce[d],
                    domains_IDC_iforce[d]
                );
            APLIC_selectOneM#(numXDestHarts, intrPrioW)
                selIDC_ithreshold(
                    domains_regs_xIDCV[d],
                    domains_xIDCs_ithreshold[d],
                    domains_IDC_ithreshold[d]
                );
        end
        uwire [(numDomains - 1):0] regs_sel_IDC_zDomainV =
            regs_sel_IDC ? regs_domainV : '0;
        uwire IDC_z_idelivery;
        APLIC_selectOneM#(numDomains, 1)
            selIDC_idelivery(
                regs_sel_IDC_zDomainV, domains_IDC_idelivery, IDC_z_idelivery);
        uwire IDC_z_iforce;
        APLIC_selectOneM#(numDomains, 1)
            selIDC_iforce(
                regs_sel_IDC_zDomainV, domains_IDC_iforce, IDC_z_iforce);
        uwire [(intrPrioW - 1):0] IDC_z_ithreshold;
        APLIC_selectOneM#(numDomains, intrPrioW)
            selIDC_ithreshold(
                regs_sel_IDC_zDomainV, domains_IDC_ithreshold, IDC_z_ithreshold
            );
        /*--------------------------------------------------------------------
        | The interrupt delivery control (IDC) structures:
        |   Determine the value of a read of a 'topi' or 'claimi' register.
        *--------------------------------------------------------------------*/
        uwire [numISources:0] IDCRead_signalIntrsV;
        assign IDCRead_signalIntrsV[0] = '0;
        for (genvar s = 1; s <= numISources; s += 1) begin
            assign IDCRead_signalIntrsV[s] =
                ((numDomains <= 1)
                     || (iSources_domainNum[s] == regs_domainNum))
                    && ((encHartIndexW < 1)
                            || (iSources_target_encHartIndex[s]
                                    == regs_IDCNum[(encHartIndexXW - 1):0]))
                    && iSourcesV_signal[s];
        end
        uwire IDCRead_validIntr;
        uwire [(intrIdentW - 1):0] IDCRead_topIntrIdent;
        uwire [(intrPrioW - 1):0] IDCRead_topIntrPrio;
        uwire regs_doRead_IDC_claimi_X;
        if ((numISources > 1) && (intrPrioW > 1)) begin
            /*----------------------------------------------------------------
            | Find the top (least) priority number among the pending-and-
            | enabled interrupts directed to the IDC's hart within the
            | interrupt domain.
            *----------------------------------------------------------------*/
            uwire [(intrPrioW - 1):0] intrTargetPrios_1[numISources + 1];
            assign intrTargetPrios_1[0] = 'x;
            for (genvar s = 1; s <= numISources; s += 1) begin
                assign intrTargetPrios_1[s] =
                    iSources_target_arg[s][(intrPrioW - 1):0];
            end
            uwire [(intrPrioW - 1):0] IDCRead_topIntrPrio_1;
            APLIC_topIntrPrioM#(numISources, intrPrioW)
                selTopIntrPrio(
                    IDCRead_signalIntrsV[numISources:1],
                    intrTargetPrios_1[1:numISources],
                    IDCRead_topIntrPrio_1
                );
            uwire regs_doRead_IDC_topi_claimi_1 =
                regs_doRead && regs_sel_IDC && regs_IDC_sel_topi_claimi;
            assign holdIPs       = regs_doRead_IDC_topi_claimi_1;
            assign regs_readWait = regs_doRead_IDC_topi_claimi_1;
            reg regs_doRead_IDC_claimi_2;
            reg [(intrPrioW - 1):0] IDCRead_topIntrPrio_2;
            always_ff @(posedge clock) begin
                regs_doRead_IDC_claimi_2 <=
                    regs_doRead_IDC_topi_claimi_1 && regs_addr[0];
                if (regs_doRead_IDC_topi_claimi_1) begin
                    IDCRead_topIntrPrio_2 <= IDCRead_topIntrPrio_1;
                end
            end
            /*----------------------------------------------------------------
            | For the pending-and-enabled interrupts directed to the IDC's
            | hart within the interrupt domain, among the subset with topmost
            | (least) priority number, find the lowest interrupt identity
            | number.
            *----------------------------------------------------------------*/
            uwire IDCRead_topIntrPrio_1_2 = (IDCRead_topIntrPrio_2>>1 == 0);
            uwire [numISources:0] IDCRead_signalIntrsV_topPrio_2;
            assign IDCRead_signalIntrsV_topPrio_2[0] = '0;
            for (genvar s = 1; s <= numISources; s += 1) begin
                assign IDCRead_signalIntrsV_topPrio_2[s] =
                    IDCRead_signalIntrsV[s]
                        && (intrTargetPrios_1[s][(intrPrioW - 1):1]
                                == IDCRead_topIntrPrio_2[(intrPrioW - 1):1])
                        && (IDCRead_topIntrPrio_1_2
                                || (intrTargetPrios_1[s][0]
                                        == IDCRead_topIntrPrio_2[0]));
            end
            APLIC_topIntrIdentM#(numISources)
                selTopIntrIdent(
                    IDCRead_signalIntrsV_topPrio_2, '0, IDCRead_topIntrIdent);
            assign IDCRead_topIntrPrio =
                IDCRead_topIntrPrio_2 | IDCRead_topIntrPrio_1_2;
            assign IDCRead_validIntr =
                ((IDC_z_ithreshold == 0) ||
                     (IDCRead_topIntrPrio < IDC_z_ithreshold))
                    && (IDCRead_topIntrIdent != 0);
            assign regs_doRead_IDC_claimi_X = regs_doRead_IDC_claimi_2;
        end else begin
            /*----------------------------------------------------------------
            | Simpler cases, either with only one interrupt source or without
            | configurable interrupt priorities (IPRIOLEN = 1).
            *----------------------------------------------------------------*/
            assign holdIPs = '0;
            assign regs_readWait = '0;
            if (numISources == 1) begin
                /*------------------------------------------------------------
                | With only one interrupt source, there's only one possible
                | top interrupt identity.
                *------------------------------------------------------------*/
                wire [(intrPrioW - 1):0] intrPrioFromTargetArg =
                    iSources_target_arg[1][(intrPrioW - 1):0];
                assign IDCRead_topIntrPrio =
                    intrPrioFromTargetArg | (intrPrioFromTargetArg>>1 == 0);
                assign IDCRead_validIntr =
                    ((IDC_z_ithreshold == 0) ||
                         (IDCRead_topIntrPrio < IDC_z_ithreshold))
                        && IDCRead_signalIntrsV[1];
                assign IDCRead_topIntrIdent = 1'b1;
            end else begin
                /*------------------------------------------------------------
                | Without configurable interrupt priorities (IPRIOLEN = 1),
                | find the lowest interrupt identity number among the pending-
                | and-enabled interrupts directed to the IDC's hart within the
                | interrupt domain.
                *------------------------------------------------------------*/
                APLIC_topIntrIdentM#(numISources)
                    selTopIntrIdent(
                        IDCRead_signalIntrsV, '0, IDCRead_topIntrIdent);
                assign IDCRead_validIntr =
                    (IDC_z_ithreshold == 0) && (IDCRead_topIntrIdent != 0);
                assign IDCRead_topIntrPrio = 1'b1;
            end
            assign regs_doRead_IDC_claimi_X =
                regs_doRead && regs_sel_IDC && regs_IDC_sel_topi_claimi
                    && regs_addr[0];
        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        assign regs_zReadData_IDC =
              (regs_IDC_sel_idelivery  ? IDC_z_idelivery  : '0)
            | (regs_IDC_sel_iforce     ? IDC_z_iforce     : '0)
            | (regs_IDC_sel_ithreshold ? IDC_z_ithreshold : '0)
            | (regs_sel_IDC && regs_IDC_sel_topi_claimi && IDCRead_validIntr
                   ? IDCRead_topIntrIdent<<16 | IDCRead_topIntrPrio : '0);
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        assign regs_doClear_IDC_iforce =
            regs_doRead_IDC_claimi_X && !IDCRead_validIntr;
        uwire deliverDirect_doClearIP_X =
            regs_doRead_IDC_claimi_X &&  IDCRead_validIntr;
        always_ff @(posedge clock) begin
            deliverDirect_doClearIP <= deliverDirect_doClearIP_X;
            if (deliverDirect_doClearIP_X) begin
                deliverDirect_intrIdent <= IDCRead_topIntrIdent;
            end
        end
    end else begin
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        assign holdIPs = '0;
        assign regs_readWait = '0;
        assign regs_zReadData_IDC = '0;
        for (
            genvar h = deliverDirect_firstHartIndex;
            h <= deliverDirect_lastHartIndex;
            h += 1
        ) begin
            assign harts_intrs[h] = '0;
        end
        for (
            genvar h = outTopIntrPrios_firstHartIndex;
            h <= outTopIntrPrios_lastHartIndex;
            h += 1
        ) begin
            assign harts_topMIntrPrio[h] = '0;
            assign harts_topSIntrPrio[h] = '0;
        end
        assign deliverDirect_doClearIP = '0;
        assign deliverDirect_intrIdent = '0;
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [(numDomains - 1):0] domainsV_doAdvanceExtemporeMSI;
    uwire deliverMSI_canAdvanceIntr_2;
    uwire deliverMSI_doAdvanceIntr_2;
    /*------------------------------------------------------------------------
    | Components exclusive to MSI delivery, part 1:
    |   * the 'genmsi' register for each relevant domain;
    |   * selection of an extempore MSI or pending-and-enabled interrupt to be
    |      delivered.
    *------------------------------------------------------------------------*/
    uwire regs_read_genmsi_zBusy;
    uwire deliverMSI_validExtemporeMSI;
    uwire [(numDomains - 1):0] extemporeMSI_zDomainV;
    uwire [(encHartIndexXW - 1):0] extemporeMSI_zEncHartIndex;
    uwire [(EIIDXW - 1):0] extemporeMSI_zEIID;
    reg deliverMSI_validIntr_2;
    reg [(intrIdentW - 1):0] deliverMSI_intrIdent_2;
    if (implDeliverMSI) begin :DeliverMSI_1
        /*--------------------------------------------------------------------
        | The 'genmsi' registers, for extempore MSIs.
        *--------------------------------------------------------------------*/
        uwire [(numDomains - 1):0] regs_zDomainV_genmsi =
            regs_sel_genmsi_activeInDomain ? regs_domainV : '0;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire [(numDomains - 1):0] domainsV_validExtemporeMSI;
        uwire [(encHartIndexXW - 1):0]
            domains_extemporeMSI_encHartIndex[numDomains];
        uwire [(EIIDW - 1):0] domains_extemporeMSI_EIID[numDomains];
        for (genvar d = 0; d < numDomains; d += 1) begin :IDomain
            reg rValidExtemporeMSI;
            uwire doWrite_genmsi =
                regs_doWrite && regs_zDomainV_genmsi[d] && !rValidExtemporeMSI;
            if (APLICConfig.domains[d].implDeliverMSI) begin
                always_ff @(negedge nReset, posedge clock) begin
                    if (!nReset) begin
                        rValidExtemporeMSI <= '0;
                    end else begin
                        rValidExtemporeMSI <=
                            (rValidExtemporeMSI
                                 && !domainsV_doAdvanceExtemporeMSI[d])
                                || doWrite_genmsi;
                    end
                end
            end else begin
                assign rValidExtemporeMSI = '0;
            end
            assign domainsV_validExtemporeMSI[d] = rValidExtemporeMSI;
            reg [(encHartIndexXW - 1):0] extemporeMSI_rEncHartIndex;
            if (
                APLICConfig.domains[d].implDeliverMSI
                    && (APLICConfig.domains[d].numHartIndices > 1)
            ) begin
                always_ff @(negedge nReset, posedge clock) begin
                    if (!nReset) begin
                        extemporeMSI_rEncHartIndex <=
                            domains_defaultEncHartIndex[d];
                    end else begin
                        if (doWrite_genmsi) begin
                            extemporeMSI_rEncHartIndex <=
                                regs_writeGroup_target_encHartIndex;
                        end
                    end
                end
            end else begin
                assign extemporeMSI_rEncHartIndex = '0;
            end
            assign domains_extemporeMSI_encHartIndex[d] =
                extemporeMSI_rEncHartIndex;
            reg [(EIIDW - 1):0] extemporeMSI_rEIID;
            if (APLICConfig.domains[d].implDeliverMSI) begin
                always_ff @(posedge clock) begin
                    if (doWrite_genmsi) begin
                        extemporeMSI_rEIID <= regs_writeData[10:0];
                    end
                end
            end else begin
                assign extemporeMSI_rEIID = '0;
            end
            assign domains_extemporeMSI_EIID[d] = extemporeMSI_rEIID;
        end
        /*--------------------------------------------------------------------
        | Select the value of a single 'genmsi' register, either for a read
        | of the register or to send an extempore MSI.  A read of a 'genmsi'
        | or 'target' register has priority, so will temporarily block the
        | sending of an extempore MSI.
        *--------------------------------------------------------------------*/
        assign regs_read_genmsi_zBusy =
            |(regs_zDomainV_genmsi & domainsV_validExtemporeMSI);
        assign deliverMSI_validExtemporeMSI = |domainsV_validExtemporeMSI;
        uwire [(numDomains - 1):0] extemporeMSIV_firstValid;
        APLIC_maskFirstM#(numDomains)
            findFirstValidExtemporeMSI(
                domainsV_validExtemporeMSI, extemporeMSIV_firstValid);
        assign extemporeMSI_zDomainV =
            (regs_doRead ? regs_zDomainV_genmsi : '0)
                | (!regs_doReadGroup_target ? extemporeMSIV_firstValid : '0);
        APLIC_selectOneM#(numDomains, encHartIndexXW)
            selExtemporeMSI_encHartIndex(
                extemporeMSI_zDomainV,
                domains_extemporeMSI_encHartIndex,
                extemporeMSI_zEncHartIndex
            );
        APLIC_selectOneM#(numDomains, EIIDW)
            selExtemporeMSI_EIID(
                extemporeMSI_zDomainV,
                domains_extemporeMSI_EIID,
                extemporeMSI_zEIID
            );
        /*--------------------------------------------------------------------
        | Delivery of interrupts by MSI, stage 1.
        +---------------------------------------------------------------------
        | As a simplification, after an interrupt is advanced for delivery
        | by MSI, its pending bit must be cleared before another interrupt
        | can be advanced.  This limits the maximum rate of outgoing MSIs to
        | one every 2 clock cycles.  (With extra effort, the maximum rate of
        | outgoing MSIs could be increased to one per clock cycle, but the
        | expectation is that most systems wouldn't benefit enough.)
        *--------------------------------------------------------------------*/
        uwire domains_enIntrMSIs[numDomains];
        for (genvar d = 0; d < numDomains; d += 1) begin
            assign domains_enIntrMSIs[d] =
                (domains_DM[d] == 1) && domains_IE[d];
        end
        uwire [numISources:0] iSourcesV_enMSI;
        assign iSourcesV_enMSI[0] = '0;
        for (genvar s = 1; s <= numISources; s += 1) begin
            assign iSourcesV_enMSI[s] =
                domains_enIntrMSIs[iSources_domainNum[s]];
        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire validIntr_1;
        reg [(intrIdentW - 1):0] roundRobin_lastIntrIdent;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        reg enRoundRobinIntrs;
        uwire newEnRoundRobinIntrs = validIntr_1 && !enRoundRobinIntrs;
        reg [(intrIdentW - 1):0] maxExcludedIntrIdent;
        always_ff @(negedge nReset, posedge clock) begin
            if (!nReset) begin
                enRoundRobinIntrs    <= '0;
                maxExcludedIntrIdent <= '0;
            end else begin
                if (
                    (enRoundRobinIntrs && !validIntr_1)
                        || deliverMSI_doAdvanceIntr_2
                ) begin
                    enRoundRobinIntrs <= newEnRoundRobinIntrs;
                    maxExcludedIntrIdent <=
                        newEnRoundRobinIntrs ? roundRobin_lastIntrIdent : '0;
                end
            end
        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire [(intrIdentW - 1):0] intrIdent_1;
        APLIC_topIntrIdentM#(numISources)
            selIntrIdent(
                iSourcesV_signal & iSourcesV_enMSI,
                maxExcludedIntrIdent,
                intrIdent_1
            );
        assign validIntr_1 = (intrIdent_1 != 0);
        uwire canAdvanceIntr_1 = !deliverMSI_validIntr_2;
        uwire doAdvanceIntr_1 = validIntr_1 && canAdvanceIntr_1;
        /*--------------------------------------------------------------------
        | After each delivered interrupt, 'deliverMSI_validIntr_2' must return
        | to false (0) before being set for the next interrupt.  This is done
        | to give time for the IP bit of the previous interrupt to be cleared
        | before another interrupt is queued for delivery.
        |   If an interrupt that is queued in 'deliverMSI_validIntr_2' and
        | 'deliverMSI_intrIdent_2' cannot be forwarded immediately as an MSI,
        | the set of signaling interrupts might change while it waits.  We
        | must adapt to the top interrupt changing during that time, or to
        | there no longer being any signaling interrupt at all.
        *--------------------------------------------------------------------*/
        always_ff @(negedge nReset, posedge clock) begin
            if (!nReset) begin
                deliverMSI_validIntr_2 <= '0;
            end else begin
                deliverMSI_validIntr_2 <=
                    !validIntr_1 || regs_doWrite ? '0
                        : ((deliverMSI_validIntr_2
                                && !deliverMSI_canAdvanceIntr_2)
                               || canAdvanceIntr_1);
            end
        end
        reg isRoundRobinIntr_2;
        always_ff @(posedge clock) begin
            if (validIntr_1) deliverMSI_intrIdent_2 <= intrIdent_1;
            if (doAdvanceIntr_1) isRoundRobinIntr_2 <= enRoundRobinIntrs;
        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        always_ff @(negedge nReset, posedge clock) begin
            if (!nReset) begin
                roundRobin_lastIntrIdent <= '0;
            end else begin
                if (
                    ((roundRobin_lastIntrIdent != 0) && !validIntr_1)
                        || (deliverMSI_doAdvanceIntr_2 && isRoundRobinIntr_2)
                ) begin
                    roundRobin_lastIntrIdent <=
                        !validIntr_1 ? '0 : deliverMSI_intrIdent_2;
                end
            end
        end
    end else begin
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        assign regs_read_genmsi_zBusy       = '0;
        assign deliverMSI_validExtemporeMSI = '0;
        assign extemporeMSI_zDomainV        = '0;
        assign extemporeMSI_zEncHartIndex   = '0;
        assign extemporeMSI_zEIID           = '0;
        assign deliverMSI_validIntr_2       = '0;
        assign deliverMSI_intrIdent_2       = '0;
    end
    /*------------------------------------------------------------------------
    | Select the value of a single 'target' register, either for a read of the
    | register or to forward an interrupt by MSI.  Highest priority goes to a
    | read of a 'genmsi' or 'target' register; next to an extempore MSI; and
    | lastly to the forwarding of an interrupt by MSI.
    *------------------------------------------------------------------------*/
    uwire [numISources:0] selISourceTargetV;
    uwire [(domainNumXW - 1):0] iSourceTarget_zDomainNum;
    if (implDeliverMSI) begin
        assign selISourceTargetV =
            1'b1<<(  (regs_doReadGroup_target ? regs_addr[9:0] : '0)
                   | (!regs_doReadGroup_target && !deliverMSI_validExtemporeMSI
                          ? deliverMSI_intrIdent_2 : '0));
        APLIC_selectOneM#(numISources, domainNumXW)
            selISourceTarget_domainNum(
                selISourceTargetV[numISources:1],
                iSources_domainNum[1:numISources],
                iSourceTarget_zDomainNum
            );
    end else begin
        assign selISourceTargetV = regs_selV_1024[numISources:0];
        assign iSourceTarget_zDomainNum = regs_domainNum;
    end
    uwire [(encHartIndexXW - 1):0] iSourceTarget_zEncHartIndex;
    if (numEncHartIndices > 1) begin
        APLIC_selectOneM#(numISources, encHartIndexXW)
            selISourceTarget_encHartIndex(
                selISourceTargetV[numISources:1],
                iSources_target_encHartIndex[1:numISources],
                iSourceTarget_zEncHartIndex
            );
    end else begin
        assign iSourceTarget_zEncHartIndex = '0;
    end
    uwire [(guestIndexXW - 1):0] iSourceTarget_zXGuestIndex;
    if (implGuestIndex) begin
        APLIC_selectOneM#(numISources, guestIndexXW)
            selISourceTarget_xGuestIndex(
                selISourceTargetV[numISources:1],
                iSources_target_xGuestIndex[1:numISources],
                iSourceTarget_zXGuestIndex
            );
    end else begin
        assign iSourceTarget_zXGuestIndex = '0;
    end
    uwire [(targetArgW - 1):0] iSourceTarget_zArg;
    if (implTargetArg) begin
        APLIC_selectOneM#(numISources, targetArgW)
            selISourceTarget_arg(
                selISourceTargetV[numISources:1],
                iSources_target_arg[1:numISources],
                iSourceTarget_zArg
            );
    end else begin
        assign iSourceTarget_zArg = '0;
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [(numDomains - 1):0] iTarget_domainV =
        implDeliverMSI ? extemporeMSI_zDomainV | 1'b1<<iSourceTarget_zDomainNum
            : regs_domainV;
    uwire domainTargetInfoT iTarget_domainInfo;
    APLIC_selectOneM#(numDomains, $bits(domainTargetInfoT))
        selDomainTargetInfo(
            iTarget_domainV, domains_targetInfo, iTarget_domainInfo);
    uwire iTarget_domain_S = iTarget_domainInfo.level_S;
    uwire [(encHartIndexXW - 1):0] iTarget_encHartIndex =
        extemporeMSI_zEncHartIndex | iSourceTarget_zEncHartIndex;
    uwire [(guestIndexXW - 1):0] iSourceTarget_zGuestIndex =
        iTarget_domain_S ? iSourceTarget_zXGuestIndex : '0;
    uwire [(guestIndexXW - 1):0] iTarget_guestIndex =
        iSourceTarget_zGuestIndex;
    uwire [(targetArgW - 1):0] iTarget_arg =
        extemporeMSI_zEIID | iSourceTarget_zArg;
    /*------------------------------------------------------------------------
    | Convert the encoded hart index of a 'genmsi' or 'target' register into a
    | proper hart index for the interrupt domain.
    *------------------------------------------------------------------------*/
    uwire [(numDomains - 1):0] iTarget_domainsV_domHartIndexIsWraparound;
    for (genvar d = 0; d < numDomains; d += 1) begin
        assign iTarget_domainsV_domHartIndexIsWraparound[d] =
            (iTarget_encHartIndex
                 < breakEncHartIndexForDomain(APLICConfig, d));
    end
    uwire iTarget_domHartIndexIsWraparound =
        |(iTarget_domainV & iTarget_domainsV_domHartIndexIsWraparound);
    uwire [(hartIndexXW - 1):0] iTarget_domHartIndex =
        (iTarget_domainInfo.upperDomHartIndex<<encHartIndexW)
            + {iTarget_domHartIndexIsWraparound, iTarget_encHartIndex};
    /*------------------------------------------------------------------------
    | Value for a read of a 'genmsi' or 'target' register.
    *------------------------------------------------------------------------*/
    uwire [(targetArgW - 1):0] iTarget_visibleArg;
    if (implDeliverDirect && implDeliverMSI && (intrPrioW < EIIDW)) begin
        assign iTarget_visibleArg[(EIIDW - 1):intrPrioW] =
            (regs_domain_DM == 1) ? iTarget_arg[(EIIDW - 1):intrPrioW] : '0;
        assign iTarget_visibleArg[(intrPrioW - 1):0] =
            iTarget_arg[(intrPrioW - 1):0]
                | ((regs_domain_DM == 0)
                       && (iTarget_arg[(intrPrioW - 1):0]>>1 == 0));
    end else if (
        implDeliverDirect && implDeliverMSI && (EIIDW < intrPrioW)
    ) begin
        assign iTarget_visibleArg[(intrPrioW - 1):EIIDW] =
            (regs_domain_DM == 0) ? iTarget_arg[(intrPrioW - 1):EIIDW] : '0;
        assign iTarget_visibleArg[(EIIDW - 1):0] =
            iTarget_arg[(EIIDW - 1):0]
                | ((regs_domain_DM == 0) && (iTarget_arg>>1 == 0));
    end else if (implDeliverDirect) begin
        assign iTarget_visibleArg =
            iTarget_arg
                | ((!implDeliverMSI || (regs_domain_DM == 0))
                       && (iTarget_arg>>1 == 0));
    end else begin
        assign iTarget_visibleArg = iTarget_arg;
    end
    uwire [31:0] regs_zReadData_group_target =
        (regs_sel_genmsi_activeInDomain || regs_sel_target_activeInDomain
             ? iTarget_domHartIndex<<18 | iTarget_visibleArg : '0)
            | regs_read_genmsi_zBusy<<12
            | (regs_sel_target_activeInDomain ? iSourceTarget_zGuestIndex<<12
                   : '0);
    /*------------------------------------------------------------------------
    | Components exclusive to MSI delivery, part 2:
    |   * the MSI address configuration registers;
    |   * the master memory port for MSI writes.
    *------------------------------------------------------------------------*/
    uwire [31:0] regs_zReadData_msiaddrcfg;
    uwire deliverMSI_doClearIP;
    if (implDeliverMSI) begin :DeliverMSI_2
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam APLICPkg::mmsiaddrcfgT mmsiaddrcfg_writable =
            APLICConfig.mmsiaddrcfg_writable;
        localparam APLICPkg::mmsiaddrcfgT mmsiaddrcfg_const =
            APLICConfig.mmsiaddrcfg_const;
        localparam APLICPkg::smsiaddrcfgT smsiaddrcfg_writable =
            APLICConfig.smsiaddrcfg_writable;
        localparam APLICPkg::smsiaddrcfgT smsiaddrcfg_const =
            APLICConfig.smsiaddrcfg_const;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire regs_domain_root = (regs_domainNum == 0);
        uwire regs_validDomain_M =
            regs_validDomain
//*** GET DOMAIN PRIV LEVEL ANOTHER WAY:
                && (APLICConfig.domains[regs_domainNum].privLevel
                        == APLICPkg::privLevel_M);
        uwire regs_selGroup_msiaddrcfg = (regs_addr>>10 == 'h1000>>12);
        uwire regs_sel_mmsiaddrcfg =
            regs_selGroup_msiaddrcfg && regs_selV_1024['hBC0/4];
        uwire regs_sel_mmsiaddrcfgh =
            regs_selGroup_msiaddrcfg && regs_selV_1024['hBC4/4];
        uwire regs_sel_smsiaddrcfg =
            regs_selGroup_msiaddrcfg && regs_selV_1024['hBC8/4];
        uwire regs_sel_smsiaddrcfgh =
            regs_selGroup_msiaddrcfg && regs_selV_1024['hBCC/4];
        /*--------------------------------------------------------------------
        | The MSI address configuration registers, 'mmsiaddrcfg' and
        | 'smsiaddrcfg'.
        *--------------------------------------------------------------------*/
        reg msiaddrcfg_L;
        uwire [4:0] msiaddrcfg_HHXS;
        uwire [2:0] mmsiaddrcfg_LHXS;
        uwire [2:0] msiaddrcfg_HHXW;
        uwire [3:0] msiaddrcfg_LHXW;
        uwire [43:0] mmsiaddrcfg_basePPN;
        uwire [2:0] smsiaddrcfg_LHXS;
        uwire [43:0] smsiaddrcfg_basePPN;
        if (
            (mmsiaddrcfg_writable != 0) || (smsiaddrcfg_writable != 0)
        ) begin :Regs_mmsiaddrcfg
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            uwire doWrite_en_msiaddrcfg =
                regs_doWrite && regs_domain_root && !msiaddrcfg_L;
            uwire doWrite_mmsiaddrcfgh =
                doWrite_en_msiaddrcfg && regs_sel_mmsiaddrcfgh;
            always_ff @(negedge nLockReset, posedge clock) begin
                if (!nLockReset) begin
                    msiaddrcfg_L <= '0;
                end else begin
                    if (doWrite_mmsiaddrcfgh) begin
                        msiaddrcfg_L <= regs_writeData[31];
                    end
                end
            end
            reg [4:0] msiaddrcfg_rHHXS;
            reg [2:0] mmsiaddrcfg_rLHXS;
            reg [2:0] msiaddrcfg_rHHXW;
            reg [3:0] msiaddrcfg_rLHXW;
            reg [43:0] mmsiaddrcfg_rBasePPN;
            reg [2:0] smsiaddrcfg_rLHXS;
            reg [43:0] smsiaddrcfg_rBasePPN;
            always_ff @(posedge clock) begin
                if (doWrite_mmsiaddrcfgh) begin
                    msiaddrcfg_rHHXS <=
                        mmsiaddrcfg_writable.HHXS & regs_writeData[28:24];
                    mmsiaddrcfg_rLHXS <=
                        mmsiaddrcfg_writable.LHXS & regs_writeData[22:20];
                    msiaddrcfg_rHHXW <=
                        mmsiaddrcfg_writable.HHXW & regs_writeData[18:16];
                    msiaddrcfg_rLHXW <=
                        mmsiaddrcfg_writable.LHXW & regs_writeData[15:12];
                    mmsiaddrcfg_rBasePPN[43:32] <=
                        mmsiaddrcfg_writable.basePPN[43:32]
                            & regs_writeData[11:0];
                end
                if (doWrite_en_msiaddrcfg && regs_sel_mmsiaddrcfg) begin
                    mmsiaddrcfg_rBasePPN[31:0] <=
                        mmsiaddrcfg_writable.basePPN[31:0] & regs_writeData;
                end
                if (doWrite_en_msiaddrcfg && regs_sel_smsiaddrcfgh) begin
                    smsiaddrcfg_rLHXS <=
                        smsiaddrcfg_writable.LHXS & regs_writeData[22:20];
                    smsiaddrcfg_rBasePPN[43:32] <=
                        smsiaddrcfg_writable.basePPN[43:32]
                            & regs_writeData[11:0];
                end
                if (doWrite_en_msiaddrcfg && regs_sel_smsiaddrcfg) begin
                    smsiaddrcfg_rBasePPN[31:0] <=
                        smsiaddrcfg_writable.basePPN[31:0] & regs_writeData;
                end
            end
            assign msiaddrcfg_HHXS =
                  ( mmsiaddrcfg_writable.HHXS    & msiaddrcfg_rHHXS)
                | (~mmsiaddrcfg_writable.HHXS    & mmsiaddrcfg_const.HHXS);
            assign mmsiaddrcfg_LHXS =
                  ( mmsiaddrcfg_writable.LHXS    & mmsiaddrcfg_rLHXS)
                | (~mmsiaddrcfg_writable.LHXS    & mmsiaddrcfg_const.LHXS);
            assign msiaddrcfg_HHXW =
                  ( mmsiaddrcfg_writable.HHXW    & msiaddrcfg_rHHXW)
                | (~mmsiaddrcfg_writable.HHXW    & mmsiaddrcfg_const.HHXW);
            assign msiaddrcfg_LHXW =
                  ( mmsiaddrcfg_writable.LHXW    & msiaddrcfg_rLHXW)
                | (~mmsiaddrcfg_writable.LHXW    & mmsiaddrcfg_const.LHXW);
            assign mmsiaddrcfg_basePPN =
                  ( mmsiaddrcfg_writable.basePPN & mmsiaddrcfg_rBasePPN)
                | (~mmsiaddrcfg_writable.basePPN & mmsiaddrcfg_const.basePPN);
            assign smsiaddrcfg_LHXS =
                  ( smsiaddrcfg_writable.LHXS    & smsiaddrcfg_rLHXS)
                | (~smsiaddrcfg_writable.LHXS    & smsiaddrcfg_const.LHXS);
            assign smsiaddrcfg_basePPN =
                  ( smsiaddrcfg_writable.basePPN & smsiaddrcfg_rBasePPN)
                | (~smsiaddrcfg_writable.basePPN & smsiaddrcfg_const.basePPN);
        end else begin
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            assign msiaddrcfg_L        = '1;
            assign msiaddrcfg_HHXS     = mmsiaddrcfg_const.HHXS;
            assign mmsiaddrcfg_LHXS    = mmsiaddrcfg_const.LHXS;
            assign msiaddrcfg_HHXW     = mmsiaddrcfg_const.HHXW;
            assign msiaddrcfg_LHXW     = mmsiaddrcfg_const.LHXW;
            assign mmsiaddrcfg_basePPN = mmsiaddrcfg_const.basePPN;
            assign smsiaddrcfg_LHXS    = smsiaddrcfg_const.LHXS;
            assign smsiaddrcfg_basePPN = smsiaddrcfg_const.basePPN;
        end
        /*--------------------------------------------------------------------
        | Value for a read of an 'msiaddrcfg' register.
        *--------------------------------------------------------------------*/
        uwire regs_msiaddrcfg_visible =
            (regs_validDomain_M
//*** GET 'M_msiaddrcfg_visible' PARAM ANOTHER WAY:
                 && APLICConfig.domains[regs_domainNum].M_msiaddrcfg_visible)
                || (regs_domain_root && !msiaddrcfg_L);
        assign regs_zReadData_msiaddrcfg =
              (regs_sel_mmsiaddrcfg && regs_msiaddrcfg_visible
                   ? mmsiaddrcfg_basePPN[31:0] : '0)
            | (regs_sel_mmsiaddrcfgh && regs_validDomain_M
                   ? (!regs_domain_root || msiaddrcfg_L)<<31 : '0)
            | (regs_sel_mmsiaddrcfgh && regs_msiaddrcfg_visible
                   ? msiaddrcfg_HHXS<<24 | mmsiaddrcfg_LHXS<<20
                         | msiaddrcfg_HHXW<<16 | msiaddrcfg_LHXW<<12
                         | mmsiaddrcfg_basePPN>>32
                   : '0)
            | (regs_sel_smsiaddrcfg && regs_msiaddrcfg_visible
                   ? smsiaddrcfg_basePPN[31:0] : '0)
            | (regs_sel_smsiaddrcfgh && regs_msiaddrcfg_visible
                   ? smsiaddrcfg_LHXS<<20 | smsiaddrcfg_basePPN>>32 : '0);
        /*--------------------------------------------------------------------
        | Delivery of interrupts by MSI, stage 2.
        +---------------------------------------------------------------------
        | A read of a 'genmsi' or 'target' register blocks the ability to
        | initiate an MSI in this stage, because the 'iTarget_*' values will
        | be for the register being read, not for the MSI to send.
        *--------------------------------------------------------------------*/
        uwire canSendExtemporeMSI =
            !masterMemPort_validWrite && !regs_doReadGroup_target;
        uwire doSendExtemporeMSI =
            deliverMSI_validExtemporeMSI && canSendExtemporeMSI;
        assign domainsV_doAdvanceExtemporeMSI =
            canSendExtemporeMSI ? extemporeMSI_zDomainV : '0;
        assign deliverMSI_canAdvanceIntr_2 =
            !deliverMSI_validExtemporeMSI && !masterMemPort_validWrite
//*** BE MORE SELECTIVE ABOUT REG ACCESSES THAT BLOCK ADVANCING AN INTERRUPT?
                && !deliverDirect_doClearIP && !regs_doAccess;
        assign deliverMSI_doAdvanceIntr_2 =
            deliverMSI_validIntr_2 && deliverMSI_canAdvanceIntr_2;
        uwire doSendMSI = doSendExtemporeMSI || deliverMSI_doAdvanceIntr_2;
        assign deliverMSI_doClearIP = deliverMSI_doAdvanceIntr_2;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire [(hartIndexXW - 1):0] iTarget_hartIndex =
            iTarget_domHartIndex + iTarget_domainInfo.hartIndexOffset;
        uwire [(hartIndexXW - 1):0] iTarget_hartIndex_g =
            iTarget_hartIndex>>msiaddrcfg_LHXW
                & ~({(hartIndexXW){1'b1}}<<msiaddrcfg_HHXW);
        uwire [(hartIndexXW - 1):0] iTarget_hartIndex_h =
            iTarget_hartIndex & ~({(hartIndexXW){1'b1}}<<msiaddrcfg_LHXW);
        always_ff @(negedge nReset, posedge clock) begin
            if (!nReset) begin
                masterMemPort_validWrite <= '0;
            end else begin
                masterMemPort_validWrite <=
                    (masterMemPort_validWrite && !masterMemPort_ready)
                        || doSendMSI;
            end
        end
        always_ff @(posedge clock) begin
            if (doSendMSI) begin
                masterMemPort_addr <=
                    ((iTarget_domain_S ? smsiaddrcfg_basePPN
                          : mmsiaddrcfg_basePPN)
                         | (iTarget_hartIndex_g<<msiaddrcfg_HHXS)<<12
                         | iTarget_hartIndex_h
                               <<(iTarget_domain_S ? smsiaddrcfg_LHXS
                                      : mmsiaddrcfg_LHXS)
                         | iTarget_guestIndex)
                        <<12;
                masterMemPort_writeData <= iTarget_arg[(EIIDW - 1):0];
            end
        end
    end else begin
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        assign regs_zReadData_msiaddrcfg = '0;
        assign deliverMSI_doClearIP      = '0;
        assign masterMemPort_validWrite  = '0;
        assign masterMemPort_addr        = '0;
        assign masterMemPort_writeData   = '0;
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    assign regs_zReadData =
        regs_zReadData_msiaddrcfg | regs_zReadData_group_target
            | regs_zReadData_IDC;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    assign doClearIP = deliverDirect_doClearIP || deliverMSI_doClearIP;
    if (implDeliverMSI) begin
//*** OPTIMIZE THIS BETTER?:
        assign clearIP_iSourceNum =
            deliverDirect_doClearIP ? deliverDirect_intrIdent
                : deliverMSI_intrIdent_2;
    end else begin
        assign clearIP_iSourceNum = deliverDirect_intrIdent;
    end

endmodule

