
# John Hauser's RISC-V APLIC

John R. Hauser<BR>
2026 January 29

This Git repository contains a SystemVerilog implementation of an APLIC
(Advanced Platform-Level Interrupt Controller) that conforms to the
[Advanced Interrupt Architecture (AIA)](https://github.com/riscv/riscv-aia)
standard for [RISC-V](https://riscv.org/) computers.

The files in this repository are licensed under the Apache License,
Version 2.0.  The full license text is available at<BR>
<https://www.apache.org/licenses/LICENSE-2.0>

THIS APLIC IMPLEMENTATION IS PROVIDED AS-IS, FOR FREE.  IT MAY CONTAIN FLAWS
THAT CAUSE IT TO FAIL.  USE AT YOUR OWN RISK.

## Version 0.6.1

The current version of this APLIC is a preliminary one labeled 0.6.1.
This version has some known shortcomings compared to what is expected for
version 0.7.  Notably:

- The only bus interface supported for now is AXI4-Lite.  Version 0.7 is
expected also to support AMBA APB (Advanced Peripheral Bus) and
[OBI (Open Bus Interface)](https://github.com/openhwgroup/obi) connections.

- Some APLIC functionality has been only lightly tested at best.  This is
especially true of direct delivery mode, which signals interrupts to harts
by wires.  Most testing so far has concerned only MSI delivery mode.
For more about the current state of testing, see the documentation in the
[`doc`](doc) subdirectory.

- There are plans for version 0.7 to improve some parts of the internal
implementation.

It is hoped that version 0.7 will be available sometime around mid-2026.

## Features

The features of this APLIC include:

- Both little-endian and big-endian are supported, either as
little-endian-only, big-endian-only, or bi-endian.

- Up to 1023 interrupt sources are supported.

- The set of implemented source modes is configurable separately for each
interrupt source.

- Any arbitrary interrupt domain tree is supported, up to an implausibly
large size of 127 domains.

- Both direct delivery mode and MSI delivery mode are supported.  The
interrupt delivery mode of each domain is individually configurable
as either direct-delivery-only, MSI-delivery-only, or controlled by
`domaincfg`.DM.

- For direct delivery mode, the maximum number of harts per interrupt domain
is 512.  MSI delivery mode allows for as many as 16,384 harts.

- The hart index numbers used to identify actual harts need not all be
contiguous; there can be unused gaps in the sequence of index numbers.
Each interrupt domain may signal interrupts to a different subset of harts,
where each subset covers a range of index numbers from a lowest to a highest
number (and all valid numbers in between).  These subsets may overlap
partially or entirely.

- Within each supervisor-level interrupt domain, the hart index numbers can
be offset by a fixed constant if necessary.

## Documentation

For additional documentation, see file [`jh_APLIC.txt`](doc/jh_APLIC.txt) in
the [`doc`](doc) subdirectory.

