# Pi pico usbserial plus commandline

This project exposes three UARTs (the two hardware UARTs and a PIO uart) as three USB CDC devices. A fourth USB CDC device acts as a command line.

Currently, the commandline is only used to toggle some reset/recovery pins to reset/recover an external device. The three UARTs are also routed to that
device, so it is testable with a single USB cable.

All communication is polling at the moment, efficiency can probably be improved a lot by making use of pi pico's DMA.

### Building
Building the project is done either by following the standard procedure for building a pi-pico project, or from within VSCode with the 'cmake-kits' extension.
When building from VSCode, the cmake-kits.json expects the pico-sdk to live in a directory next to the project directory. If that is not the case, the path in
'cmake-kits.json' should be changed accordingly before building.

### Pinout
Numbers below are pin pad numbers, not gpio numbers.

- UART 1 (TX, RX):   (1, 2)
- UART 2 (TX, RX):   (6, 7)
- UART 3 (TX, RX):   (11, 12)
- ExternalReset:     4
- ExternalRecovery:  5
