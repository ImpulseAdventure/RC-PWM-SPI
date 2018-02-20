# RC-PWM-SPI
RC Receiver PWM to SPI for ATtiny / Digispark

Example SPI slave implementation

This code implements a simple SPI slave receiver interface
combined with multi-channel pulse-width modulation (PWM)
measurement. Each channel's pulse width is measured in
microseconds and returned in a channelized register interface.
This code can be useful for using a remote-control transmitter
to control an Arduino / ATtiny microcontroller.

- A watchdog timeout is used to detect the loss of the transmitter.
- Optimized IO commands are used in the ISRs to keep the
  critical sections as fast as possible.
- This example demonstrates 6 channel monitoring, but this can
  be increased/decreased if needed.
