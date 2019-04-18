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
mkdir Debug
(cd Debug && make -f ../Makefile)
```

Follow the Installation section above for how to install the resulting .hex
file.

### Compilation (with Docker)

```
mkdir Debug
docker run -v $PWD:/usr/src/mk66f-fw kinxproject/mk66f-fw bash -c 'cd /usr/src/mk66f-fw/Debug && make -f ../Makefile'
```
