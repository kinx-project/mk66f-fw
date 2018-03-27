## Installation

You can find all released firmware versions in the `archive` directory.

To install a firmware file, first install the `teensy-loader-cli` Debian
package, then run e.g.:
```
teensy_loader_cli -w -v --mcu=TEENSY36 fw-2018-03-24.hex
```
…and press the physical programming mode switch on the kinX board.

## Development

### Dependencies

On Debian stretch, use:
```
apt install make gcc-arm-none-eabi
```

### Compilation

```
(cd Debug && make -f ../Makefile)
```

Follow the Installation section above for how to install the resulting .hex
file.
