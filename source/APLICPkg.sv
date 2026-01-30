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

package APLICPkg;

    localparam int maxNumSources = 1023;
    localparam int maxNumValidHartIndexRanges =
        APLIC_limitsPkg::maxNumValidHartIndexRanges;
    localparam int maxNumDomains = APLIC_limitsPkg::maxNumDomains;

    typedef struct packed {
        shortint num;
        bit [maxNumSources:0] implDetachedV;
        bit [maxNumSources:0] implEdge1V;
        bit [maxNumSources:0] implEdge0V;
        bit [maxNumSources:0] implLevel1V;
        bit [maxNumSources:0] implLevel0V;
    } sourcesConfigT;

    typedef struct packed {
        shortint first;
        shortint last;
    } validHartIndexRangeT;

    typedef struct packed {
        shortint maxHartIndex;
        byte numValidHartIndexRanges;
        validHartIndexRangeT [(maxNumValidHartIndexRanges - 1):0]
            validHartIndexRanges;
        bit implLittleEndian;
        bit implBigEndian;
        byte maxGEILEN;
    } hartsConfigT;

    typedef struct packed {
        bit L;
        bit [4:0] HHXS;
        bit [2:0] LHXS;
        bit [2:0] HHXW;
        bit [3:0] LHXW;
        bit [43:0] basePPN;
    } mmsiaddrcfgT;

    typedef struct packed {
        bit [2:0] LHXS;
        bit [43:0] basePPN;
    } smsiaddrcfgT;

    typedef enum logic {
        privLevel_M = 0,
        privLevel_S = 1
    } privLevelT;

    typedef struct packed {
        byte parentNum;
        privLevelT privLevel;
        shortint firstHartIndex;
        shortint numHartIndices;
        shortint S_perceivedFirstHartIndex;
        bit implDeliverDirect;
        bit implDeliverMSI;
        bit M_msiaddrcfg_visible;
    } domainConfigT;

    typedef struct packed {
        sourcesConfigT sources;
        hartsConfigT harts;
        validHartIndexRangeT deliverDirectHartIndexRange;
        bit implOutTopMIntrPrios;
        bit implOutTopSIntrPrios;
        validHartIndexRangeT outTopIntrPriosHartIndexRange;
        mmsiaddrcfgT mmsiaddrcfg_writable;
        mmsiaddrcfgT mmsiaddrcfg_const;
        smsiaddrcfgT smsiaddrcfg_writable;
        smsiaddrcfgT smsiaddrcfg_const;
        byte numDomains;
        domainConfigT [(maxNumDomains - 1):0] domains;
        byte intrPrioW;
        byte EIIDW;
    } configT;

endpackage

