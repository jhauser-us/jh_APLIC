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
    APLIC_topIntrIdentM#(
        int numISources,
        localparam int intrIdentW = $clog2(numISources + 1)
    ) (
        input uwire [numISources:0] intrsV,
        input uwire [(intrIdentW - 1):0] maxExcludedIntrIdent,
        output uwire [(intrIdentW - 1):0] intrIdent
    );

    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam int log2_maxSplit1Factor = 3;
    localparam int log2_maxSplit2Factor = 6;
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    localparam int minNumISourcesForSplit1 = 2<<log2_maxSplit1Factor;
    localparam int numIntrs0 =
        (numISources < minNumISourcesForSplit1) ? numISources + 1
            : 1<<(log2_maxSplit1Factor
                      - (numISources < 6<<log2_maxSplit1Factor));
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [(intrIdentW - 1):0] intrIdent2, intrIdent1;
    uwire [(numIntrs0 - 1):0] intrs0V;
    if (numISources < minNumISourcesForSplit1) begin :NoSplit1
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        assign intrIdent2 = '0;
        assign intrIdent1 = '0;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire [(numIntrs0 - 1):0] allowMask;
        APLIC_maskAfterIxM#(numIntrs0)
            createAllowMask(maxExcludedIntrIdent, allowMask);
        assign intrs0V = allowMask & intrsV;
    end else begin :Split1
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        localparam int minNumISourcesForSplit2 = 2<<log2_maxSplit2Factor;
        localparam int numIntrs1 =
            (numISources < minNumISourcesForSplit2) ? numISources + 1
                : 1<<(log2_maxSplit2Factor
                          - (numISources < 6<<log2_maxSplit2Factor));
        localparam int numReduced0Intrs1 = (numIntrs1 - 1)/numIntrs0 + 1;
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire [(numReduced0Intrs1 - 1):0] reduced0Intrs1V;
        uwire [(numIntrs0 - 1):0] intrs0Vs[numReduced0Intrs1];
        if (numISources < minNumISourcesForSplit2) begin :NoSplit2
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            assign intrIdent2 = '0;
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            uwire [numReduced0Intrs1:0] reduced0AllowMask;
            assign reduced0AllowMask[numReduced0Intrs1] = 1'b1;
            APLIC_maskAfterIxM#(numReduced0Intrs1)
                createReduced0AllowMask(
                    maxExcludedIntrIdent[(intrIdentW - 1):$clog2(numIntrs0)],
                    reduced0AllowMask[(numReduced0Intrs1 - 1):0]
                );
            uwire [(numIntrs0 - 1):0] allowMask0;
            APLIC_maskAfterIxM#(numIntrs0)
                createAllowMask0(
                    maxExcludedIntrIdent[($clog2(numIntrs0) - 1):0], allowMask0
                );
            for (
                genvar I = 0; I < numReduced0Intrs1; I += 1
            ) begin :Reduced0Piece
                localparam nomCapIx = (I + 1)*numIntrs0 - 1;
                localparam capIx =
                    (nomCapIx <= numISources) ? nomCapIx : numISources;
                uwire [(numIntrs0 - 1):0] piece =
                    ({numIntrs0{reduced0AllowMask[I]}} | allowMask0)
                        & intrsV[capIx:I*numIntrs0];
                assign reduced0Intrs1V[I] =
                    reduced0AllowMask[I + 1] && (|piece);
                assign intrs0Vs[I] = piece;
            end
        end else begin :Split2
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            localparam int numReduced1Intrs = numISources/numIntrs1 + 1;
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            uwire [numReduced1Intrs:0] reduced1AllowMask;
            assign reduced1AllowMask[numReduced1Intrs] = 1'b1;
            APLIC_maskAfterIxM#(numReduced1Intrs)
                createReduced1AllowMask(
                    maxExcludedIntrIdent[(intrIdentW - 1):$clog2(numIntrs1)],
                    reduced1AllowMask[(numReduced1Intrs - 1):0]
                );
            uwire [(numIntrs1 - 1):0] allowMask1;
            APLIC_maskAfterIxM#(numIntrs1)
                createAllowMask1(
                    maxExcludedIntrIdent[($clog2(numIntrs1) - 1):0], allowMask1
                );
            uwire [(numReduced1Intrs - 1):0] reduced1IntrsV;
            uwire [(numIntrs1 - 1):0] intrs1Vs[numReduced1Intrs];
            for (
                genvar I = 0; I < numReduced1Intrs; I += 1
            ) begin :Reduce1Piece
                localparam nomCapIx = (I + 1)*numIntrs1 - 1;
                localparam capIx =
                    (nomCapIx <= numISources) ? nomCapIx : numISources;
                uwire [(numIntrs1 - 1):0] piece =
                    ({numIntrs1{reduced1AllowMask[I]}} | allowMask1)
                        & intrsV[capIx:I*numIntrs1];
                assign reduced1IntrsV[I] =
                    reduced1AllowMask[I + 1] && (|piece);
                assign intrs1Vs[I] = piece;
            end
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            uwire [(numReduced1Intrs - 1):0] topReduced1IntrV;
            APLIC_maskFirstM#(numReduced1Intrs)
                findTopReduced1Intr(reduced1IntrsV, topReduced1IntrV);
            uwire [($clog2(numReduced1Intrs) - 1):0] topReduced1IntrIndex;
            APLIC_indexOneM#(numReduced1Intrs)
                getIndexTopReduced1Intr(
                    topReduced1IntrV, topReduced1IntrIndex);
            assign intrIdent2 = topReduced1IntrIndex<<$clog2(numIntrs1);
            uwire [(numIntrs1 - 1):0] intrs1V;
            APLIC_selectOneM#(numReduced1Intrs, numIntrs1)
                selIntrs1(topReduced1IntrV, intrs1Vs, intrs1V);
            /*----------------------------------------------------------------
            *----------------------------------------------------------------*/
            for (
                genvar I = 0; I < numReduced0Intrs1; I += 1
            ) begin :Reduce0Piece
                localparam nomCapIx = (I + 1)*numIntrs0 - 1;
                localparam capIx =
                    (nomCapIx < numIntrs1) ? nomCapIx : numIntrs1 - 1;
                uwire [(numIntrs0 - 1):0] piece = intrs1V[capIx:I*numIntrs0];
                assign reduced0Intrs1V[I] = |piece;
                assign intrs0Vs[I] = piece;
            end
        end
        /*--------------------------------------------------------------------
        *--------------------------------------------------------------------*/
        uwire [(numReduced0Intrs1 - 1):0] topReduced0Intr1V;
        APLIC_maskFirstM#(numReduced0Intrs1)
            findTopReduced0Intr1(reduced0Intrs1V, topReduced0Intr1V);
        uwire [($clog2(numReduced0Intrs1) - 1):0] topReduced0Intr1Index;
        APLIC_indexOneM#(numReduced0Intrs1)
            getIndexTopReduced0Intr1(topReduced0Intr1V, topReduced0Intr1Index);
        assign intrIdent1 = topReduced0Intr1Index<<$clog2(numIntrs0);
        APLIC_selectOneM#(numReduced0Intrs1, numIntrs0)
            selIntrs0(topReduced0Intr1V, intrs0Vs, intrs0V);
    end
    /*------------------------------------------------------------------------
    *------------------------------------------------------------------------*/
    uwire [(numIntrs0 - 1):0] topIntr0V;
    APLIC_maskFirstM#(numIntrs0) findTopIntr0(intrs0V, topIntr0V);
    uwire [($clog2(numIntrs0) - 1):0] topIntr0Index;
    APLIC_indexOneM#(numIntrs0) getIndexTopIntr0(topIntr0V, topIntr0Index);
    assign intrIdent = intrIdent2 | intrIdent1 | topIntr0Index;

endmodule

