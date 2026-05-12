# Protocol

- UDP packet layout: Fixed size binary packet.
- byte order: Little-endian (Host byte order assumed for both PCs in x86/ARM).
- seq: 32-bit integer for sequence number. Wraparound is handled.
- timestamp: 64-bit nanosecond timestamp from epoch.
- CRC: CRC32 applied to all fields before it.
- right/left port: Separate ports are used for right and left arms (e.g. 50000 and 50001).
- loss detection: Loss is detected by `diff = current_seq - last_seq`. `diff > 1` means missed packets.
- stale packet handling: Out of order packets (`seq < last_seq` logically) are dropped immediately.
