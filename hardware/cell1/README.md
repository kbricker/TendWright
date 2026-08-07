# cell1 — host configuration

Machine-specific config for the box that runs the cell. Everything here
is installed **on cell1**, not on the desk.

cell1 is a **Minisforum UM350** (Besstar Tech), AMD Ryzen 5 3550H with
Radeon Vega, 8192 MB DDR4 @ 2400, AMI Aptio Setup 2.20.1274, BIOS
`AF5PL01` built 04/06/2022. Two Realtek NICs on `r8169`:

| interface | MAC | state |
|---|---|---|
| `enp4s0` | `1c:83:41:30:ec:2d` | **live** — 192.168.86.202 |
| `enp3s0` | `1c:83:41:30:ec:2c` | no carrier |

The MACs differ in the last digit. Both are visible in the BIOS under
Advanced, as `MAC:1C834130EC2C` and `MAC:1C834130EC2D`. Arm the wrong
one and WoL fails in a way that looks identical to WoL not working.

## Wake-on-LAN

**The BIOS was already correct.** Advanced → PowerManagement
Configuration has exactly three items, and two of them already had the
values we wanted:

| item | value | verdict |
|---|---|---|
| AC Failure Resume | `[Always On]` | already right — a switched outlet is a working power path |
| Lan Wake Up From COM RI1 | `[Enabled]` | already right — this is the WoL enable, despite the legacy ring-indicator name |
| Wake system by RTC | `[Disabled]` | not needed |

There is **no ErP/EuP setting and no Fast Boot setting** in this menu.

**The OS was the problem.** Measured 2026-07-28 with the box running:

```
$ cat /sys/class/net/enp4s0/device/power/wakeup
disabled
```

PCI PME# assertion was disabled on both NICs. The NIC can recognise a
magic packet all it likes; if it may not assert PME#, the board never
powers on. Firmware willing, OS not, and nothing reports the mismatch —
which is exactly why WoL "doesn't work" and stays mysterious.

Two switches, both needed, neither persistent on `r8169`:

```bash
sudo ethtool -s enp4s0 wol g                                    # arm the filter
sudo sh -c 'echo enabled > /sys/class/net/enp4s0/device/power/wakeup'   # allow PME#
sudo ethtool enp4s0 | grep Wake-on                              # want: Wake-on: g
```

To make it survive reboots, install the unit in this directory:

```bash
sudo cp hardware/cell1/wol-enp4s0.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now wol-enp4s0.service
```

Then verify from the desk — the only test that counts is a real power
cycle:

```bash
ssh cell1 'sudo poweroff'
uv run python -m hardware.bench.wake cell1     # expect: up in ~40s
```

**Persistence is not proven until the box has been rebooted and the
values re-checked.** A unit that runs once at install time and a unit
that runs at every boot look identical the day you install them.

**Proven 2026-08-06:** from full power-off, `wake cell1` from the desk
reached sshd in 24 s, with `wol-enp4s0.service` reading `active` after
boot. The wake→shutdown→wake cycle is now routine — cell1 stays off
when idle.

## Memory budget

8192 MB installed, and only 5.23 GiB reaches the OS. Where the rest goes:

| consumer | size | reclaimable? |
|---|---|---|
| iGPU frame buffer (`amdgpu` VRAM) | 2048 MB | **yes** — BIOS, Chipset menu. Don't set it to zero; the bench uses a monitor. |
| `crashkernel` reservation | 512 MB | **yes** — kernel cmdline. We have never collected a crash dump from this box. |
| firmware / kernel | ~270 MB | no |
| **available to the OS** | **5.23 GiB** | |

Both reclaims together take usable RAM from 5.23 GiB to roughly 7.2 GiB,
a ~38% increase. That matters directly: camserve reached 3.7 GB of 5.2
GB before the leak work in #704, and headroom is the cheapest mitigation
available.

### Reclaiming the crashkernel 512 MB

**Not** a `/etc/default/grub` edit — that file is stock on cell1
(`GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`) and editing it would have
done nothing. The reservation comes from a package drop-in:

```
/etc/default/grub.d/kdump-tools.cfg
GRUB_CMDLINE_LINUX_DEFAULT="$GRUB_CMDLINE_LINUX_DEFAULT crashkernel=2G-4G:320M,4G-32G:512M,..."
```

`grub-mkconfig` sources `/etc/default/grub.d/*.cfg` **after**
`/etc/default/grub`, so the drop-in always wins. Nobody chose this
setting; it arrived with the `kdump-tools` package.

And kdump was never armed — `USE_KDUMP=0`, `kdump-config show` reads
"Not ready to kdump", `kexec_crash_loaded` is 0 while
`kexec_crash_size` holds the full 536870912 bytes. The memory was
reserved for a capability that could not fire.

```bash
sudo apt purge kdump-tools           # removes the package AND its grub.d drop-in
sudo apt autoremove --purge          # 9 orphans: crash, kexec-tools, makedumpfile, initramfs-tools*, ...
sudo cp hardware/cell1/no-kdump-tools.pref /etc/apt/preferences.d/no-kdump-tools
sudo update-grub
sudo reboot
```

**The `autoremove` was verified safe for this box specifically, and is
not a blind recommendation.** It offers to remove `initramfs-tools-core`
and `initramfs-tools-bin`; on a machine that used them, losing those
means no initrd can be rebuilt at the next kernel update, which is a
box that stops booting. cell1 builds its initramfs with **dracut**
(110-11) and reverse-depends on `initramfs-tools-core` only from
`initramfs-tools-bin` (circular) and `kdump-tools` itself, so they are
genuinely orphaned here. Check before copying this to another machine.

Afterwards `MemTotal` should rise by ~512 MB and `crashkernel` should be
absent from `/proc/cmdline`.
