# legacy/

Six files from Petoi's original OpenCat firmware for the Nybble
(https://github.com/PetoiCamp/OpenCat, by Rongzhong Li), kept unchanged as
reference data. Nothing here is built. The rest of that firmware
(`OpenCat.ino` and the top-level `src/`) was deleted on 2026-11-20; git
history has it, and `mind/firmware/opencat-legacy.hex` is its built image.

| File | Read by |
|---|---|
| `InstinctNybble.h` | `body/test/test_misc.cpp` (built-in postures), `mind/kilo/tricks/legacy.py` (the trick importer's default source; `mind/tests/test_tricks.py` checks the library is fresh) |
| `imu.h`, `skill.h` | `body/test/test_attitude.cpp` (attitude A/B against the legacy `ypr`) |
| `motion.h` | `body/test/test_balance.cpp` (balance gains and `adjust()`) |
| `mpu6050/MPU6050_6Axis_MotionApps_V6_12.h` | `body/test/test_misc.cpp` (the DMP image, byte for byte) |
| `infrared.h` | `mind/tests/test_ir.py` (the IR remote's key table) |

Licence: MIT, as OpenCat's (the top-level `LICENSE`, copyright Rongzhong Li).
`imu.h` and the MPU6050 header also carry Jeff Rowberg's MIT notice for the
I2Cdev library; keep the notices in the files.
