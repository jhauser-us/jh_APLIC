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
    APLIC_topIntrPrio2M#(int intrPrioW) (
        input uwire [1:0] mask,
        input uwire [(intrPrioW - 1):0] prios[2],
        output uwire [(intrPrioW - 1):0] topPrio
    );

    for (genvar i = intrPrioW - 1; i >= 0; i -= 1) begin :Bit
        uwire [1:0] succExcludes;
        uwire [1:0] excludes;
        if (i == intrPrioW - 1) begin
            assign excludes = ~mask;
        end else begin
            assign excludes = Bit[i + 1].succExcludes;
        end
        uwire [1:0] bits = {prios[1][i], prios[0][i]};
        uwire topBit = ((excludes | bits) == 'b11);
        assign topPrio[i] = topBit;
        assign succExcludes = excludes | (topBit ? '0 : bits);
    end

endmodule

/*----------------------------------------------------------------------------
*----------------------------------------------------------------------------*/

module
    APLIC_topIntrPrio3M#(int intrPrioW) (
        input uwire [2:0] mask,
        input uwire [(intrPrioW - 1):0] prios[3],
        output uwire [(intrPrioW - 1):0] topPrio
    );

    for (genvar i = intrPrioW - 1; i >= 0; i -= 1) begin :Bit
        uwire [2:0] succExcludes;
        uwire [2:0] excludes;
        if (i == intrPrioW - 1) begin
            assign excludes = ~mask;
        end else begin
            assign excludes = Bit[i + 1].succExcludes;
        end
        uwire [2:0] bits = {prios[2][i], prios[1][i], prios[0][i]};
        uwire topBit = ((excludes | bits) == 'b111);
        assign topPrio[i] = topBit;
        assign succExcludes = excludes | (topBit ? '0 : bits);
    end

endmodule

/*----------------------------------------------------------------------------
| Parameter 'numIntrs' must be greater than 1.
|   There appears to be a bug in at least some versions of AMD/Xilinx
| Vivado ML that requires a default value be specified for parameter
| 'intrPrioW' (probably due to the module being recursively defined).
*----------------------------------------------------------------------------*/

module
    APLIC_topIntrPrioM#(int numIntrs, int intrPrioW = 1) (
        input uwire [(numIntrs - 1):0] intrsV,
        input uwire [(intrPrioW - 1):0] intrs_prio[numIntrs],
        output uwire [(intrPrioW - 1):0] topIntrPrio
    );

    /*------------------------------------------------------------------------
    | Assuming argument 'numIntrs' is greater than 1, function
    | 'splitFactorFromNumIntrs' returns either 2 or 3.
    *------------------------------------------------------------------------*/
    function automatic int splitFactorFromNumIntrs(int numIntrs);
        int n = numIntrs;
        forever begin
            if (n <= 3) return n;
            n = (n + 2)/3;
        end
    endfunction
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    if (numIntrs == 2) begin
        APLIC_topIntrPrio2M#(intrPrioW)
            findTopIntrPrio(intrsV, intrs_prio, topIntrPrio);
    end else if (numIntrs == 3) begin
        APLIC_topIntrPrio3M#(intrPrioW)
            findTopIntrPrio(intrsV, intrs_prio, topIntrPrio);
    end else begin
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam int splitFactor = splitFactorFromNumIntrs(numIntrs);
        localparam int splitNumIntrs = numIntrs/splitFactor;
        localparam int remNumIntrs = numIntrs - splitNumIntrs*splitFactor;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        if (splitFactor == 2) begin :SplitBy2
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            localparam int splitNumIntrs1 = splitNumIntrs + (remNumIntrs > 0);
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            uwire [(intrPrioW - 1):0] partTopIntrPrios[2];
            APLIC_topIntrPrioM#(splitNumIntrs1, intrPrioW)
                findTopIntrPrio_1(
                    intrsV[(splitNumIntrs1 - 1):0],
                    intrs_prio[0:(splitNumIntrs1 - 1)],
                    partTopIntrPrios[0]
                );
            APLIC_topIntrPrioM#(splitNumIntrs, intrPrioW)
                findTopIntrPrio_2(
                    intrsV[(numIntrs - 1):splitNumIntrs1],
                    intrs_prio[splitNumIntrs1:(numIntrs - 1)],
                    partTopIntrPrios[1]
                );
            APLIC_topIntrPrio2M#(intrPrioW)
                findTopIntrPrioMerge(2'b11, partTopIntrPrios, topIntrPrio);
        end else begin :SplitBy3
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            localparam int splitNumIntrs1 = splitNumIntrs + (remNumIntrs > 0);
            localparam int splitNumIntrs2 = splitNumIntrs + (remNumIntrs > 1);
            localparam int splitNumIntrsNot3 = splitNumIntrs1 + splitNumIntrs2;
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            uwire [(intrPrioW - 1):0] partTopIntrPrios[3];
            APLIC_topIntrPrioM#(splitNumIntrs1, intrPrioW)
                findTopIntrPrio_1(
                    intrsV[(splitNumIntrs1 - 1):0],
                    intrs_prio[0:(splitNumIntrs1 - 1)],
                    partTopIntrPrios[0]
                );
            APLIC_topIntrPrioM#(splitNumIntrs2, intrPrioW)
                findTopIntrPrio_2(
                    intrsV[(splitNumIntrsNot3 - 1):splitNumIntrs1],
                    intrs_prio[splitNumIntrs1:(splitNumIntrsNot3 - 1)],
                    partTopIntrPrios[1]
                );
            APLIC_topIntrPrioM#(splitNumIntrs, intrPrioW)
                findTopIntrPrio_3(
                    intrsV[(numIntrs - 1):splitNumIntrsNot3],
                    intrs_prio[splitNumIntrsNot3:(numIntrs - 1)],
                    partTopIntrPrios[2]
                );
            APLIC_topIntrPrio3M#(intrPrioW)
                findTopIntrPrioMerge(3'b111, partTopIntrPrios, topIntrPrio);
        end
    end

endmodule

