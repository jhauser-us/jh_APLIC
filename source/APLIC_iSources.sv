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
    APLIC_iSourcesM#(
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        int options,
        APLICPkg::configT APLICConfig,
        bit implDeliverDirect,
        bit implDeliverMSI,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam int numISources = APLICConfig.sources.num,
        localparam int numDomains = APLICConfig.numDomains,
        localparam int domainNumXW = (numDomains < 2) ? 1 : $clog2(numDomains)
    ) (
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire nReset,
        input uwire clock,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire [numISources:0] intrsInV,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire domains_DM[numDomains],
        input uwire [(domainNumXW - 1):0] iSources_domainNum[numISources + 1],
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        input uwire regs_selGroup_sourcecfg,
        input uwire regs_doWrite,
        input uwire [12:0] regs_addr,
        input uwire [31:0] regs_selV_32,
        input uwire [1023:0] regs_selV_1024,
        input uwire regs_domain_BE,
        input uwire [31:0] regs_writeData_LE,
        input uwire [31:0] regs_writeData,
        input uwire regs_single_validInDomain,
        input uwire regs_doWriteISourceMode,
        input uwire [31:0] regs_setclr_validInDomainV,
        input uwire holdIPs,
        input uwire doClearIP,
        input uwire [31:0] selIVRegV,
        input uwire [4:0] iVReg_iSourceNum5,
        output uwire [31:0] regs_zReadData,
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        output uwire [numISources:0] iSourcesV_active,
        output uwire [numISources:0] iSourcesV_signal
    );

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam int numIVRegs = numISources/32 + 1;
    localparam bit implLittleEndian = APLICConfig.harts.implLittleEndian;
    localparam bit implBigEndian    = APLICConfig.harts.implBigEndian;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire regs_selGroup_setclr = (regs_addr>>8 == 'h1C00>>10);
    uwire regs_setclr_ip    = !regs_addr[7];
    uwire regs_setclr_ie    =  regs_addr[7];
    uwire regs_setclr_set   = !regs_addr[6];
    uwire regs_setclr_clr   =  regs_addr[6];
    uwire regs_setclr_byNum =  regs_addr[5];
    uwire regs_setclr_setip    = regs_setclr_set && regs_setclr_ip;
    uwire regs_setclr_in_clrip = regs_setclr_clr && regs_setclr_ip;
    uwire regs_selGroup_setipnum_xe = (regs_addr>>10 == 'h2000>>12);
    uwire regs_setclr_setipnum_xe = regs_addr[11];
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire regs_sourcecfg_doInitISource =
        regs_doWriteISourceMode
            && (!regs_single_validInDomain || regs_writeData[10]);
    /*------------------------------------------------------------------------
    | An interrupt domain's memory region offsets 0x1C80-0x1CFF,
    | 0x1D80-0x1DFF, 0x1E80-0x1EFF, 0x1F80-0x1FFF, and 0x2000-0x2FFF contain
    | all the registers that set or clear IP/IE bits by interrupt number.
    | All other locations in these ranges are made inactive for this purpose
    | simply by ensuring that 'regs_setclr_isValidISourceNum' is false.
    *------------------------------------------------------------------------*/
    uwire regs_setclr_isValidISourceNum =
        ((regs_selGroup_setipnum_xe
              && (regs_selV_1024[0] || (implBigEndian && regs_selV_1024[1])))
             || (regs_selGroup_setclr
                     && regs_setclr_byNum && regs_selV_32['h5C>>2]))
            && (regs_writeData_LE[23:18] == 0)
            && (regs_writeData_LE[15:10] == 0)
            && (implBigEndian
                    && (regs_setclr_setipnum_xe ? regs_addr[0]
                            : (!implLittleEndian || regs_domain_BE))
                    ? (regs_writeData_LE[9:0] == 0)
                    : (regs_writeData_LE[31:24] == 0)
                          && (regs_writeData_LE[17:16] == 0));
    /*------------------------------------------------------------------------
    | If an interrupt source is inactive, its IP bit is zero (after a
    | negligible delay of one clock cycle after setting the source mode).  An
    | inactive source's IE bit should also be zero, but the stored "internal"
    | IE bit may actually be spurious.  Therefore, when IE bits are read
    | through the subordinate bus interface, the internal IEs must be masked
    | by whether the sources are active.  Whenever a 'sourcecfg' register
    | is written, the corresponding internal IE bit gets belatedly updated
    | with the previous correct IE bit, so that changing from an inactive to
    | an active mode leaves the visible IE bit as zero, as required.  (This
    | roundabout way of managing the IE bits is expected to save a logic
    | gate or two per interrupt source.)
    *------------------------------------------------------------------------*/
    uwire [numISources:0] iSourcesV_in;
    uwire [numISources:0] iSourcesV_IP;
    reg [numISources:0] iSourcesV_internalIE;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [31:0] iVRegs_in[numIVRegs];
    uwire [31:0] iVRegs_ip[numIVRegs];
    uwire [31:0] iVRegs_ie[numIVRegs];
    for (genvar r = 0; r < numIVRegs; r += 1) begin
        localparam int nomCapISourceNum = r*32 + 31;
        localparam int capISourceNum =
            (nomCapISourceNum <= numISources) ? nomCapISourceNum : numISources;
        assign iVRegs_in[r] = iSourcesV_in[capISourceNum:r*32];
        assign iVRegs_ip[r] = iSourcesV_IP[capISourceNum:r*32];
        assign iVRegs_ie[r] =
            iSourcesV_active[capISourceNum:r*32]
                & iSourcesV_internalIE[capISourceNum:r*32];
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [31:0] regs_zIVReg_ip;
    APLIC_selectOneM#(numIVRegs, 32)
        selIVReg_ip(
            regs_selGroup_setclr && regs_setclr_setip && !regs_setclr_byNum
                ? selIVRegV[(numIVRegs - 1):0] : '0,
            iVRegs_ip,
            regs_zIVReg_ip
        );
    uwire [31:0] regs_zIVReg_in;
    APLIC_selectOneM#(numIVRegs, 32)
        selIVReg_in(
            regs_selGroup_setclr && regs_setclr_in_clrip && !regs_setclr_byNum
                ? selIVRegV[(numIVRegs - 1):0] : '0,
            iVRegs_in,
            regs_zIVReg_in
        );
    uwire [31:0] regs_zIVReg_ie;
    APLIC_selectOneM#(numIVRegs, 32)
        selIVReg_ie(
            regs_doWrite
                || (regs_selGroup_setclr && regs_setclr_set && regs_setclr_ie
                        && !regs_setclr_byNum)
                ? selIVRegV[(numIVRegs - 1):0] : '0,
            iVRegs_ie,
            regs_zIVReg_ie
        );
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [2:0] iSources_xMode[numISources + 1];
    /*------------------------------------------------------------------------
    | Value for a read of a 'sourcecfg' register for a non-delegated source
    | (field 'SM' only), or of a 'setip', 'in_clrip', or 'setie' register.
    |   Source "xMode" values 2 and 3 are aliases for mode 0 (inactive).  (It
    | is cheaper to translate values 2 and 3 here, in one place, rather than
    | replicate the same logic for every interrupt source.)
    *------------------------------------------------------------------------*/
    uwire [2:0] regs_iSource_xMode;
    APLIC_selectOneM#(numISources, 3)
        selISourceMode(
            regs_selV_1024[numISources:1],
            iSources_xMode[1:numISources],
            regs_iSource_xMode
        );
    uwire [2:0] regs_iSource_mode =
        {regs_iSource_xMode[2], (regs_iSource_xMode[2:1] == 'b11),
             (regs_iSource_xMode[2:1] != 'b01) && regs_iSource_xMode[0]};
    assign regs_zReadData =
          (regs_selGroup_sourcecfg && regs_single_validInDomain
               ? regs_iSource_mode : '0)
        | ((regs_zIVReg_ip | regs_zIVReg_in | regs_zIVReg_ie)
               & regs_setclr_validInDomainV);
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [31:0] iVRegV_doModify_ip =
           regs_sourcecfg_doInitISource
        || (regs_doWrite
                && ((regs_selGroup_setclr && regs_setclr_ip)
                        || regs_selGroup_setipnum_xe))
        || doClearIP
            ? selIVRegV : '0;
    uwire [31:0] iVRegV_doModify_ie =
        regs_doWriteISourceMode
            || (regs_doWrite && regs_selGroup_setclr && regs_setclr_ie)
            ? selIVRegV : '0;
    uwire [31:0] iVReg_iSource32V = 1'b1<<iVReg_iSourceNum5;
    uwire [31:0] regs_setclr_zWriteData =
        regs_setclr_validInDomainV
            & (  (regs_setclr_isValidISourceNum ? iVReg_iSource32V : '0)
               | (regs_selGroup_setclr && !regs_setclr_byNum ? regs_writeData
                      : '0));
    uwire [31:0] iVReg_setV =
        regs_doWrite && (regs_setclr_setipnum_xe || regs_setclr_set)
            ? regs_setclr_zWriteData : '0;
    uwire [31:0] iVReg_clearV =
          (regs_sourcecfg_doInitISource || doClearIP ? iVReg_iSource32V  : '0)
        | (regs_doWrite && regs_setclr_clr      ? regs_setclr_zWriteData : '0);
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [2:0] iSource_newMode =
        regs_writeData[10] ? '0 : regs_writeData[2:0];
    uwire [7:0] iSource_newModeV = 1'b1<<iSource_newMode;
    uwire iSource_newMode_1_5 = (iSource_newMode[1:0] == 'b01);
    uwire iSource_newMode_1_6 = iSource_newModeV[1] || iSource_newModeV[6];
    uwire iSource_newMode_1_7 = iSource_newModeV[1] || iSource_newModeV[7];
    uwire iSource_newMode_4_5 = (iSource_newMode[2:1] == 'b10);
    uwire iSource_newMode_4_6 = iSource_newMode[2] && !iSource_newMode[0];
    uwire iSource_newMode_4_7 = iSource_newModeV[4] || iSource_newModeV[7];
    uwire iSource_newMode_5_6 = iSource_newModeV[5] || iSource_newModeV[6];
    uwire iSource_newMode_5_7 = iSource_newMode[2] &&  iSource_newMode[0];
    uwire iSource_newMode_6_7 = (iSource_newMode[2:1] == 'b11);
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    assign iSourcesV_active[0] = '0;
    assign iSources_xMode[0]   = 'x;
    assign iSourcesV_in[0]     = '0;
    assign iSourcesV_IP[0]     = '0;
    for (genvar s = 1; s <= numISources; s += 1) begin :ISource
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam bit implModeDetached = APLICConfig.sources.implDetachedV[s];
        localparam bit implModeEdge1    = APLICConfig.sources.implEdge1V[s];
        localparam bit implModeEdge0    = APLICConfig.sources.implEdge0V[s];
        localparam bit implModeEdge     = implModeEdge1 || implModeEdge0;
        localparam bit implModeLevel1   = APLICConfig.sources.implLevel1V[s];
        localparam bit implModeLevel0   = APLICConfig.sources.implLevel0V[s];
        localparam bit implModeLevel    = implModeLevel1 || implModeLevel0;
        localparam int encModeW =
            $clog2(
                1 + implModeDetached + implModeEdge1 + implModeEdge0
                    + implModeLevel1 + implModeLevel0
            );
        localparam int encModeXW = (encModeW < 1) ? 1 : encModeW;
        localparam bit implEdgeDetect =
            implModeEdge || (implModeLevel && implDeliverMSI);
        /*--------------------------------------------------------------------
        | For 'xMode', values 2 and 3 are allowed as aliases for value 0
        | (inactive).  (Allowing these aliases can reduce the logic for each
        | interrupt source.)
        *--------------------------------------------------------------------*/
        reg [(encModeXW - 1):0] rEncMode;
        uwire mode_active;
        uwire [2:0] xMode;
        uwire activeAttached_inverted;
        if (encModeW == 0) begin
            /*----------------------------------------------------------------
            | The only implemented source mode is Inactive.
            *----------------------------------------------------------------*/
            assign rEncMode                = '0;
            assign mode_active             = '0;
            assign xMode                   = '0;
            assign activeAttached_inverted = '0;
        end else begin :Impl
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            uwire [(encModeW - 1):0] newEncMode;
            if (encModeW == 1) begin
                /*------------------------------------------------------------
                | Implement one active source mode, encoded in 1 bit.
                *------------------------------------------------------------*/
                assign newEncMode =
                       (implModeDetached && iSource_newModeV[1])
                    || (implModeEdge1    && iSource_newModeV[4])
                    || (implModeEdge0    && iSource_newModeV[5])
                    || (implModeLevel1   && iSource_newModeV[6])
                    || (implModeLevel0   && iSource_newModeV[7]);
                assign mode_active = rEncMode;
                assign xMode =
                      (implModeDetached && mode_active ? 3'd1 : '0)
                    | (implModeEdge1    && mode_active ? 3'd4 : '0)
                    | (implModeEdge0    && mode_active ? 3'd5 : '0)
                    | (implModeLevel1   && mode_active ? 3'd6 : '0)
                    | (implModeLevel0   && mode_active ? 3'd7 : '0);
                assign activeAttached_inverted =
                    implModeEdge0 || implModeLevel0;
            end else if (encModeW == 2) begin
                /*------------------------------------------------------------
                | Implement two or three active source modes, encoded in
                | 2 bits.
                *------------------------------------------------------------*/
                if (implModeDetached) begin
                    if (!implModeLevel) begin
                        if (!implModeEdge0) begin
                            /*------------------------------------------------
                            | Implement active modes Detached and Edge1.
                            *------------------------------------------------*/
                            assign newEncMode =
                                {iSource_newModeV[4], iSource_newModeV[1]};
                            assign xMode =
                                rEncMode[1] ? 3'd4 : rEncMode[0] ? 3'd1 : '0;
                            assign activeAttached_inverted = '0;
                        end else if (!implModeEdge1) begin
                            /*------------------------------------------------
                            | Implement active modes Detached and Edge0.
                            *------------------------------------------------*/
                            assign newEncMode =
                                {iSource_newModeV[5], iSource_newModeV[1]};
                            assign xMode =
                                  (rEncMode[1] ? 3'd5 : '0)
                                | (rEncMode[0] ? 3'd1 : '0);
                            assign activeAttached_inverted = '1;
                        end else begin
                            /*------------------------------------------------
                            | Implement active modes Detached, Edge1, and
                            | Edge0.
                            *------------------------------------------------*/
                            assign newEncMode =
                                {iSource_newMode_4_5, iSource_newMode_1_5};
                            assign xMode =
                                (rEncMode[1] ? 3'd4 : '0)
                                    | (rEncMode[0] ? 3'd1 : '0);
                            assign activeAttached_inverted = rEncMode[0];
                        end
                    end else if (!implModeEdge) begin
                        if (!implModeLevel0) begin
                            /*------------------------------------------------
                            | Implement active modes Detached and Level1.
                            *------------------------------------------------*/
                            assign newEncMode =
                                {iSource_newModeV[6], iSource_newModeV[1]};
                            assign xMode =
                                rEncMode[1] ? 3'd6 : rEncMode[0] ? 3'd1 : '0;
                            assign activeAttached_inverted = '0;
                        end else if (!implModeLevel1) begin
                            /*------------------------------------------------
                            | Implement active modes Detached and Level0.
                            *------------------------------------------------*/
                            assign newEncMode =
                                {iSource_newModeV[7], iSource_newModeV[1]};
                            assign xMode =
                                  (rEncMode[1] ? 3'd7 : '0)
                                | (rEncMode[0] ? 3'd1 : '0);
                            assign activeAttached_inverted = '1;
                        end else begin
                            /*------------------------------------------------
                            | Implement active modes Detached, Level1, and
                            | Level0.
                            *------------------------------------------------*/
                            assign newEncMode =
                                {iSource_newMode_6_7, iSource_newMode_1_7};
                            assign xMode =
                                (rEncMode[1] ? 3'd6 : '0)
                                    | (rEncMode[0] ? 3'd1 : '0);
                            assign activeAttached_inverted = rEncMode[0];
                        end
                    end else if (!implModeEdge0 && !implModeLevel0) begin
                        /*----------------------------------------------------
                        | Implement active modes Detached, Edge1, and Level1.
                        *----------------------------------------------------*/
                        assign newEncMode =
                            {iSource_newMode_4_6, iSource_newMode_1_6};
                        assign xMode =
                            rEncMode[1] ? (3'd4 | (rEncMode[0] ? 3'd2 : '0))
                                : (rEncMode[0] ? 3'd1 : '0);
                        assign activeAttached_inverted = '0;
                    end else if (!implModeEdge0 && !implModeLevel1) begin
                        /*----------------------------------------------------
                        | Implement active modes Detached, Edge1, and Level0.
                        *----------------------------------------------------*/
                        assign newEncMode =
                            {iSource_newMode_4_7, iSource_newMode_1_7};
                        assign xMode =
                            {rEncMode[1], (rEncMode == 'b11), rEncMode[0]};
                        assign activeAttached_inverted = rEncMode[0];
                    end else if (!implModeEdge1 && !implModeLevel0) begin
                        /*----------------------------------------------------
                        | Implement active modes Detached, Edge0, and Level1.
                        *----------------------------------------------------*/
                        assign newEncMode =
                            {iSource_newMode_5_6, iSource_newMode_1_5};
                        assign xMode =
                            {rEncMode[1], !rEncMode[0], rEncMode[0]};
                        assign activeAttached_inverted = rEncMode[0];
                    end else begin
                        /*----------------------------------------------------
                        | Implement active modes Detached, Edge0, and Level0.
                        *----------------------------------------------------*/
                        assign newEncMode =
                            {iSource_newMode_5_7, iSource_newMode_1_5};
                        assign xMode = {rEncMode[1], !rEncMode[0], 1'b1};
                        assign activeAttached_inverted = '1;
                    end
                    assign mode_active = (rEncMode != 0);
                end else if (!implModeLevel) begin
                    /*--------------------------------------------------------
                    | Implement active modes Edge1 and Edge0.
                    *--------------------------------------------------------*/
                    assign newEncMode =
                        {iSource_newMode_4_5, iSource_newMode[0]};
                    assign mode_active = rEncMode[1];
                    assign xMode =
                        mode_active ? 3'd4 | (rEncMode[0] ? 3'd1 : '0) : '0;
                    assign activeAttached_inverted = rEncMode[0];
                end else if (!implModeEdge) begin
                    /*--------------------------------------------------------
                    | Implement active modes Level1 and Level0.
                    *--------------------------------------------------------*/
                    assign newEncMode =
                        {iSource_newMode_6_7, iSource_newMode[0]};
                    assign mode_active = rEncMode[1];
                    assign xMode =
                        mode_active ? 3'd6 | (rEncMode[0] ? 3'd1 : '0) : '0;
                    assign activeAttached_inverted = rEncMode[0];
                end else if (!implModeEdge0 && !implModeLevel0) begin
                    /*--------------------------------------------------------
                    | Implement active modes Edge1 and Level1.
                    *--------------------------------------------------------*/
                    assign newEncMode =
                        {iSource_newMode_4_6, iSource_newMode[1]};
                    assign mode_active = rEncMode[1];
                    assign xMode =
                        (mode_active ? 3'd4 : '0) | (rEncMode[0] ? 3'd2 : '0);
                    assign activeAttached_inverted = '0;
                end else if (!implModeEdge0 && !implModeLevel1) begin
                    /*--------------------------------------------------------
                    | Implement active modes Edge1 and Level0.
                    *--------------------------------------------------------*/
                    assign newEncMode =
                        {iSource_newMode_4_7, iSource_newMode[1]};
                    assign mode_active = rEncMode[1];
                    assign xMode =
                        (mode_active ? 3'd4 : '0) | (rEncMode[0] ? 3'd3 : '0);
                    assign activeAttached_inverted = rEncMode[0];
                end else if (!implModeEdge1 && !implModeLevel0) begin
                    /*--------------------------------------------------------
                    | Implement active modes Edge0 and Level1.
                    *--------------------------------------------------------*/
                    assign newEncMode =
                        {iSource_newMode_5_6, iSource_newMode[1]};
                    assign mode_active = rEncMode[1];
                    assign xMode =
                        (mode_active ? {2'b10, !rEncMode[0]} : '0)
                            | (rEncMode[0] ? 3'd2 : '0);
                    assign activeAttached_inverted = !rEncMode[0];
                end else if (!implModeEdge1 && !implModeLevel1) begin
                    /*--------------------------------------------------------
                    | Implement active modes Edge0 and Level0.
                    *--------------------------------------------------------*/
                    assign newEncMode =
                        {iSource_newMode_5_7, iSource_newMode[1]};
                    assign mode_active = rEncMode[1];
                    assign xMode =
                        (mode_active ? 3'd5 : '0) | (rEncMode[0] ? 3'd2 : '0);
                    assign activeAttached_inverted = '1;
                end else begin
                    if (!implModeLevel0) begin
                        /*----------------------------------------------------
                        | Implement active modes Edge1, Edge0, and Level1.
                        *----------------------------------------------------*/
                        assign newEncMode =
                            {!iSource_newMode_4_5, !iSource_newMode_4_6};
                        assign mode_active = (rEncMode != 'b11);
                        assign xMode = {mode_active, rEncMode};
                    end else if (!implModeLevel1) begin
                        /*----------------------------------------------------
                        | Implement active modes Edge1, Edge0, and Level0.
                        *----------------------------------------------------*/
                        assign newEncMode =
                            {!iSource_newMode_4_5, iSource_newMode_5_7};
                        assign mode_active = (rEncMode != 'b10);
                        assign xMode = {mode_active, rEncMode};
                    end else if (!implModeEdge0) begin
                        /*----------------------------------------------------
                        | Implement active modes Edge1, Level1, and Level0.
                        *----------------------------------------------------*/
                        assign newEncMode =
                            {iSource_newMode_6_7, !iSource_newMode_4_6};
                        assign mode_active = (rEncMode != 'b01);
                        assign xMode =
                            {mode_active, rEncMode[1], (rEncMode == 'b11)};
                    end else begin
                        /*----------------------------------------------------
                        | Implement active modes Edge0, Level1, and Level0.
                        *----------------------------------------------------*/
                        assign newEncMode =
                            {iSource_newMode_6_7, iSource_newMode_5_7};
                        assign mode_active = (rEncMode != 'b00);
                        assign xMode = mode_active ? 3'd4 | rEncMode : '0;
                    end
                    assign activeAttached_inverted = rEncMode[0];
                end
            end else begin
                /*------------------------------------------------------------
                | Implement all defined source modes except maybe Detached.
                *------------------------------------------------------------*/
                assign newEncMode = iSource_newMode;
                if (implModeDetached) begin
                    assign mode_active =
                        rEncMode[2] || (rEncMode[1:0] == 'b01);
                    assign xMode = rEncMode;
                end else begin
                    assign mode_active = rEncMode[2];
                    assign xMode = mode_active ? 3'd4 | rEncMode[1:0] : '0;
                end
                assign activeAttached_inverted = rEncMode[0];
            end
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            always_ff @(posedge clock) begin
                if (regs_doWriteISourceMode && regs_selV_1024[s]) begin
                    rEncMode <= newEncMode;
                end
            end
        end
        assign iSourcesV_active[s] = mode_active;
        assign iSources_xMode[s] = xMode;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire in;
        if (!implModeEdge && !implModeLevel) begin
            assign in = '0;
        end else begin
            assign in =
                   (xMode[2] && !activeAttached_inverted &&  intrsInV[s])
                || (xMode[2] &&  activeAttached_inverted && !intrsInV[s]);
        end
        assign iSourcesV_in[s] = in;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire DM;
        if (implDeliverDirect && implDeliverMSI && implModeLevel) begin
            assign DM = domains_DM[iSources_domainNum[s]];
        end else begin
            assign DM = implDeliverMSI;
        end
        uwire active_mode_level = implModeLevel && xMode[1];
        uwire active_IPEqualsIn = (DM == 0) && active_mode_level;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        if (encModeW == 0) begin
            /*----------------------------------------------------------------
            | The only implemented source mode is Inactive.
            *----------------------------------------------------------------*/
            assign iSourcesV_IP[s] = '0;
        end else begin :IP
            /*----------------------------------------------------------------
            | A "modify" operation on an IP bit is due to an external access
            | to an APLIC register or the forwarding of an interrupt by MSI.
            *----------------------------------------------------------------*/
            uwire doSet_in;
            if (implEdgeDetect) begin :DetectEdge
                reg prevIn;
                always_ff @(posedge clock) prevIn <= in;
                assign doSet_in = (active_IPEqualsIn || !prevIn) && in;
            end else begin
                assign doSet_in = in;
            end
            uwire doClear_in = !holdIPs && active_mode_level && !in;
            uwire enModify =
                (implModeDetached || implEdgeDetect)
                    && iVRegV_doModify_ip[s/32];
            uwire doSet_modify = enModify && iVReg_setV[s%32];
            uwire doClear_modify =
                enModify && iVReg_clearV[s%32]
                    && (regs_sourcecfg_doInitISource || !active_IPEqualsIn);
            reg rIP;
            always_ff @(posedge clock) begin
                rIP <=
                    mode_active
                        ? (rIP || doSet_in || doSet_modify)
                              && !doClear_in && !doClear_modify
                        : '0;
            end
            assign iSourcesV_IP[s] = rIP;
        end
    end
    /*------------------------------------------------------------------------
    | Write to IE bits.  Write only to whole 32-bit ie registers, by merging
    | 'regs_zIVReg_ie' from earlier with the set/clear masks 'iVReg_setV' and
    | 'iVReg_clearV'.
    *------------------------------------------------------------------------*/
    uwire [31:0] regs_newIVRegValue_ie =
        (regs_zIVReg_ie | iVReg_setV) & ~iVReg_clearV;
    for (genvar r = 0; r < numIVRegs; r += 1) begin
        localparam int nomCapISourceNum = r*32 + 31;
        localparam int capISourceNum =
            (nomCapISourceNum <= numISources) ? nomCapISourceNum : numISources;
        always_ff @(posedge clock) begin
            if (iVRegV_doModify_ie[r]) begin
                iSourcesV_internalIE[capISourceNum:r*32] <=
                    regs_newIVRegValue_ie[capISourceNum%32:0];
            end
        end
    end
    /*------------------------------------------------------------------------
    | (Although the stored "internal" IE bit of an inactive interrupt
    | source can be spurious, its IP bit will be zero, preventing a spurious
    | interrupt signal.)
    *------------------------------------------------------------------------*/
    assign iSourcesV_signal = iSourcesV_IP & iSourcesV_internalIE;

endmodule

