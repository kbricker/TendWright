# hardware/

- `bench/` — P2-prep bring-up CLI tools for the SO-101 servos + camera
  (plan #618); see `bench/README.md` for the assembly-day order.
- `mockbay/` — the mock CNC bay: parametric nest fixture (OpenSCAD +
  STL) with the KW12-3 part-present switch (plan #619); measure-first
  notes in `mockbay/README.md`.
- `conveyor/` — mini modular conveyor motor bridge (plan #835): Pico 2
  MicroPython firmware (`conveyor/firmware/`) + host `ConveyorDriver` +
  `hardware.conveyor.run` bring-up CLI. `hardware.conveyor.selftest`
  exercises the command protocol with no hardware attached.
- `pico/` — Pico bridge: MicroPython firmware (`pico/firmware/`) + host
  `NestReader` (the PicoCell sensor backend) + `hardware.pico.watch`.
- `so101-print/` — vendored SO-101 follower print STLs
  (TheRobotStudio/SO-ARM100, Apache-2.0) + bench mounts.
- P6 (plan #610) adds the real drivers here: GRBL serial driver, arm
  driver, HIL wiring for the full tend-inspect-log loop.
