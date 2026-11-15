# Monocypher (vendored)

The crypto under the host channel's Noise layer (`kernel/mind/net/noise.*`,
`kernel/README.md` "Who may talk to Kilo: keys"): X25519, BLAKE2b and the
IETF ChaCha20-Poly1305 AEAD, in one C file with no dependencies (no libc;
the compiler may emit `memset`/`memcpy`, which Circle and the host's
freestanding check allow).

| | |
|---|---|
| Version | 4.0.3 (released 2026-06-15) |
| From | https://monocypher.org/download/monocypher-4.0.3.tar.gz (byte-identical to the GitHub release asset https://github.com/LoupVaillant/Monocypher/releases/download/4.0.3/monocypher-4.0.3.tar.gz) |
| Tarball SHA-256 | `8cc9bc341a66249016db9bd70e9142d8d0aef9945973744b1ac05dbc55d8ee66` |
| Tarball SHA-512 | `40904ada5c7ee4f7741733e38b69a30a4b0561cbffba5ffe7c2dce16136d540251ec0d9056ff606510d3b5b708fb8a40db7e0870d4a0b2dc17ba2bfb880f8965` (matches the published `monocypher-4.0.3.tar.gz.sha512`) |
| Tarball BLAKE2b | `5c586019e8e78ce6fb70f73f4ded10c852335138394b22fb704d0b4379edfc1adaea342f0e4c4b8c52fcf053df401d1fe47812ee5cb5062813de19fc51f8ae93` (matches the published `.blake2b`) |
| Files | `src/monocypher.c`, `src/monocypher.h`, `LICENCE.md`, unchanged |
| Licence | dual: 2-clause BSD or CC-0, at our choice (`LICENCE.md`) |

The files are upstream's, byte for byte: never edit them. To upgrade,
download the new tarball, check its published checksum, copy the same three
files and update this table. They are compiled as C (`kernel/mind/sources.mk`)
with the kernel's and the host's usual flags; they build without warnings
under `-Wall -Wextra -Werror`.

The Noise layer calls `crypto_x25519`, `crypto_x25519_public_key`,
`crypto_blake2b*`, `crypto_aead_init_ietf`/`crypto_aead_write`/`crypto_aead_read`,
`crypto_verify32` and `crypto_wipe`. The kernel links the whole file
anyway (about 55 KB of ARM code, EdDSA and Argon2 included): Circle builds
without `GC_SECTIONS`, and turning it on for the whole kernel to save
some 25 KB is not worth a change nobody has run on the Pi.
