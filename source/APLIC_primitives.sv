// SPDX-License-Identifier: Apache-2.0

/*============================================================================

This file is part of an implementation of a RISC-V Advanced Platform-Level
Interrupt Controller (APLIC) by John R. Hauser.

Copyright 2022, 2023 John R. Hauser.

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
    APLIC_selectOneM#(int N, int elemW) (
        input uwire [(N - 1):0] mask,
        input uwire [(elemW - 1):0] A[N],
        output uwire [(elemW - 1):0] elem
    );

    for (genvar n = 0; n < N; n += 1) begin :Elem
        uwire [(elemW - 1):0] zElem = mask[n] ? A[n] : '0;
        uwire [(elemW - 1):0] zReduct;
        if (n == 0) begin
            assign zReduct = zElem;
        end else begin
            assign zReduct = Elem[n - 1].zReduct | zElem;
        end
    end
    assign elem = Elem[N - 1].zReduct;

endmodule

/*----------------------------------------------------------------------------
*----------------------------------------------------------------------------*/

module
    APLIC_indexOneM#(int W) (
        input uwire [(W - 1):0] mask,
        output uwire [($clog2(W) - 1):0] index
    );

    for (genvar i = 0; i < W; i = i + 1) begin :Bit
        uwire [($clog2(W) - 1):0] zReduct;
        if (i == 0) begin
            assign zReduct = '0;
        end else begin
            assign zReduct = Bit[i - 1].zReduct | (mask[i] ? i : '0);
        end
    end
    assign index = Bit[W - 1].zReduct;

endmodule

/*----------------------------------------------------------------------------
*----------------------------------------------------------------------------*/

module
    APLIC_maskFirstM#(int W) (
        input uwire [(W - 1):0] in,
        output uwire [(W - 1):0] out
    );

    assign out[0] = in[0];
    for (genvar i = 1; i < W; i = i + 1) begin
        assign out[i] = !(|in[(i - 1):0]) && in[i];
    end

endmodule

/*----------------------------------------------------------------------------
*----------------------------------------------------------------------------*/

module
    APLIC_maskAfterIxM#(int W) (
        input uwire [($clog2(W) - 1):0] n,
        output uwire [(W - 1):0] out
    );

    localparam int xW = 1<<$clog2(W);
    uwire [($clog2(W) - 1):0] complN = ~n;
    uwire [(xW - 1):0] xOut = $signed({1'b1, {(xW - 1){1'b0}}})>>>complN;
    assign out[0] = '0;
    if (W > 1) assign out[(W - 1):1] = xOut[(W - 2):0];

endmodule

