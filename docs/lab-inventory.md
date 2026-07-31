# The lab: what exists, where it is, and how to drive it

Everything physical in the cell, its address, and the command that actuates
it. Written 2026-07-29 because the shape of this lab was living in one
session's head, and an agent that cannot name what it controls will ask a
human to go flip a switch it could have flipped itself. (That happened: a
3 h 49 m soak ran with the bench light off, and the light is on the LAN.)

**Rebuild information belongs here, not in scratch.** The rule Kyle set
2026-07-29: if losing the box would mean re-deriving it from hardware, it
goes in the repo. Addresses, MACs, wiring and calibration qualify.

---

## Compute

| | cell1 |
|---|---|
| Hardware | Minisforum UM350 (DMI `BESSTAR TECH LIMITED`), Ryzen 5 3550H |
| OS | **Ubuntu 26.04 LTS** (resolute), kernel 7.0.0-28-generic, **glibc 2.43** |
| RAM | 5.87 GB usable; the integrated Radeon takes the rest as UMA |
| BIOS | AMI 5.14, dated 2022-04-06. **No update exists** — Minisforum publishes none and `fwupdmgr get-updates` reports "No updatable devices" |
| Network | `enp4s0`, Realtek RTL8111/8168, MAC `1c:83:41:30:ec:2d`, **192.168.86.202** |
| Access | `ssh cell1`, repo at `~/TendWright`, `uv` at `~/.local/bin/uv` |
| Display | **Not headless.** Boots to `graphical.target`; Kyle uses the GNOME desktop at the bench. ~280 MB resident, and it is not to be "reclaimed" |

Two more identical UM350s are unimaged spares (future MES box). They are the
reason a provisioning script pays for itself — see plan #744.

## Mains power — TP-Link Kasa, local protocol, no cloud

All three answer the legacy unauthenticated protocol on udp/9999 (discovery)
and tcp/9999 (control). No dependency, no account. Tool:
`hardware/bench/kasa.py`. **Works from the desk as well as cell1** — the
control path does not route through cell1, so power is reachable even when
cell1 is off.

| Address | Model | Name | Outlets |
|---|---|---|---|
| `192.168.86.90` | KP303 | bench strip | `[0] Arm` *(guarded)*, `[1] Light`, `[2] USB` |
| `192.168.86.44` | HS200 | **Garage Workbench** — wall light switch | single relay |
| `192.168.86.50` | HS300 | house strip, **not bench** | Plug 1, Elder1, Elder2, Moon, Plug 5, Stems |

- **`192.168.86.44` is the bench light.** It is the only light switch on the
  LAN. If a vision test needs a lit scene, turn it on — do not ask.
- The KP303 was labelled before anything was wired to it (Kyle 2026-07-29:
  *"nothing is plugged into the new one, I just labeled them"*). Its outlet
  names are intent until the arm and light are physically connected. **Check
  before trusting `Arm` to mean the arm is powered.**
- Never firmware-update any of them: newer firmware moves to KLAP, which
  needs account credentials and kills local control. Kyle has accepted the
  auto-update risk as low and reactive — do not re-raise it.

```bash
uv run python -m hardware.bench.kasa list
uv run python -m hardware.bench.kasa on  192.168.86.44          # bench light
uv run python -m hardware.bench.kasa off 192.168.86.90 Light
```

## The arm

Feetech STS3215 bus servos, 6 joints, over a CH340-family USB adapter.

- **`/dev/ttyACM<N>` and the index CLIMBS with every re-plug** — never
  hardcode `ttyACM0`. Leaving it plugged stops the drift.
- The adapter enumerates even with servo power OFF, because it is
  host-powered. **Adapter presence proves nothing.** The real test is
  `uv run python -m hardware.bench.scan` (read-only).
- **These servos have no brakes.** Cutting power to an arm that is not
  folded makes it fall. `kasa` enforces this — see below.
- Calibration lives in `calibration.json` (per-joint min/rest/max/sign);
  `pan-wiggle.json` is the saved `runner example` output — the clip
  whose `rest` pose IS this arm's captured rest. Its filename matches
  the name inside it on purpose: tools that only have the clip's name
  (a trace header, say) resolve it back to `<name>.json`.

## Cameras

Two, both registered in `cameras.json`, both `tags: true`, both 1920x1080@30
solo. Served by `camserve` on `:8081` — **no auth, LAN only, never
port-forward.**

- `bench` — back wall, 60° printed bracket, lens ~26.6 in above the table,
  looking down the main table at the work zone. Normally sees 4 tags.
- `low` — printed edge stand on the table's short end, lens 75 mm up, 7°
  down, looking along the table. The only view that resolves HEIGHT.

Cameras open only while watched, so `/status` showing `fps 0.0` and
`profile: null` with no viewer is normal, not a fault. Tag detection is
opt-in per request (`?tags=1`) as of #713.6 — **a soak driven by a plain
viewer runs zero detections and proves nothing about the detector.**

`camserve` is hand-launched today and does **not** survive a reboot. Making
it a systemd unit is the first item on #744.

## What may be actuated, and the gates

| Action | Authority | Gate |
|---|---|---|
| Read anything (`/proc`, `/status`, encoders, Kasa state) | always | none |
| Start/stop own processes (soaks, samplers) | always | none |
| Restart camserve | ask Kyle first | standing rule |
| Shut cell1 down | **yes** — `sudo -n /usr/sbin/shutdown -h now` | scoped NOPASSWD, see below |
| Switch a normal outlet / the bench light | **yes** | none |
| Switch a guarded outlet ON (`Arm`, `psu`, `12v`, …) | yes | `--confirm <alias>` |
| Switch a guarded outlet OFF | yes | arm must read within `REST_TOL_TICKS` of rest, verified from the encoders; `--force` overrides |
| `apt` / anything else root | **no** | hand Kyle the command |
| Move the arm unattended | **no** | the e-stop is a keypress that does not exist with nobody at the bench (#712.11) |

cell1's sudoers grants exactly:

```
(root) NOPASSWD: /usr/sbin/poweroff, /usr/sbin/shutdown, /usr/bin/systemctl poweroff
```

`sudo -n true` still FAILS, because that tests the general `(ALL:ALL) ALL`
entry which needs a password. Testing general sudo tells you nothing about a
scoped grant — check `sudo -n -l` instead.

## Gotchas that have each cost real time

- **`pkill -f` / `pgrep -f` over ssh matches your own command line**, because
  the command line *is* the pattern. Has killed a live session three times,
  including once mid-cleanup after being written down twice. Use exact PIDs,
  or a pattern that cannot self-match (`bench[.]camserve`).
- **Detached launches need `nohup setsid CMD < /dev/null > log 2>&1 &`** AND
  verification on a **separate** ssh connection. A subshell `( ... & )` does
  not survive teardown. The launching ssh will appear to hang; that is
  expected — background it.
- **`MUJOCO_GL=egl` is required** for offscreen rendering; the default
  backend dies with `gladLoadGL error` under ssh. `sim/simcam.py` sets it
  when there is no display.
- **`v4l2-ctl` is not installed.** Query cameras through OpenCV, or pull
  frames via `camserve`'s `/cam/<name>/snapshot` — never open a `/dev/video*`
  node directly while camserve is serving, which causes a per-camera 503.
- **`/tmp` does not survive a reboot.** Long-run evidence goes to
  `~/soak-evidence/`.
- **`unattended-upgrades` is active**, so packages move on their own. A
  service restarting mid-soak is a confound worth checking before trusting a
  long run.
