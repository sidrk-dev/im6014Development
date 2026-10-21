# Portable controller source verification

`im6014_control` ports only the reviewed committed C12 pure controller closure
from [im6014-arm-control at 227ec4e](https://github.com/sidrk-dev/im6014-arm-control/tree/227ec4ecc712c0cee48a50c9e234e70dd065b8e3).
The [provenance map](../../im6014_control/provenance.json) records original source
and test hashes. No dirty C13 files, bench UI, serial transport, CAD or environment
was copied.

The parent reran all 108 regressions against both the committed original
snapshot and the new package. All passed. A [source comparison](verification.json)
shows identical ASTs in the five source modules after the necessary relative
import changes. All seven test bodies also match after import/module-alias
normalization. An initial overly strict test comparison flagged the namespace
rename; normalizing that rename confirmed the preserved test bodies.

The configured Gemini reviewer inspected the original closure and identified
codec/gain, one-shot consumption and timing concerns for simulator integration.
The parent retained these as integration constraints. They do not justify
weakening codec limits or changing intentional accepted C12 behavior during a
mechanical port. The reviewer ran no tests; it did not establish physical
acceptance. Its completed receipt used pinned-argv model evidence.

This is a portable software foundation, not SA04 completion. A simulator station
must still define the virtual drive and gain relation, observe measured logical
coordinates, enforce effort/fault/ownership/expiry/final-release rules, and
verify 30 Hz policy versus 100 Hz local control under delay. C13-C16 remain
unaccepted. Importing the package performs no transport or motor I/O; the bench
keeps its existing serial-session owner and startup path.
