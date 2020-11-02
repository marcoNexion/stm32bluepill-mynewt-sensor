# `semihosting_console`

Mynewt Library that implements the `console` interface for displaying
messages on the RTT Segger console. 

All messages are cached in memory until `console_flush()` is called,
or when the console enters blocking mode.
