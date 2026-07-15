# TendWright Hardware Shopping List

Ordered by how soon each item gets used. Prices researched 2026-07-15 (USD, volatile — verify at checkout). Full rationale: `spec-tendwright-hardware` in Hive.

---

## Now — used starting at P2 (vision)

### 1. Camera — ELP fixed/manual-focus M12 USB board cam (~$40–55)
The bin camera for P2 picking and P3 inspection. **Must be a fixed/manual-focus variant with the ~3.6mm lens** — not autofocus, not the 2.1mm fisheye.
- [ELP official store](https://www.svpro.cc/shop/)
- [Example: ELP OV2710 1080p module](https://www.elpcctv.com/elp-full-hd-usb-camera-module-1080p-usb20-ov2710-color-sensor-mjpeg-with-wide-angle-21mm-lens-p-204.html) (pick the 3.6mm lens option)
- Amazon: search “ELP USB camera 1080p manual focus 3.6mm”

### 2. Lighting — 10" USB ring light (~$25)
Mounts around the camera over the bin; fixed brightness + locked exposure = repeatable vision.
- [Neewer 10" USB ring light (Amazon)](https://www.amazon.com/Neewer-3200K-5600K-Dimmable-Streaming-Photography/dp/B08733GLS3)

### 3. 3D printer — Bambu Lab A1, no AMS (~$299–349 sale) *(recommended, optional)*
Camera mounts immediately; then the nest fixture (the #1-risk mitigation — same-day design iterations vs. ~1 week per revision from a print service), gripper mod, riser, jigs. Buy early so the learning curve is done before fixtures matter.
- [Bambu Lab A1](https://us.store.bambulab.com/products/a1) (July sale pricing; list $349–399)
- Plus 2× PLA+ spools + 1× PETG spool (~$50, any brand on Amazon)

### 4. Cell controller — your NUC ($0)
Core i-series confirmed. Ubuntu LTS, user in `dialout`, udev rules for stable serial names. Hosts the vision rig at P2, the whole cell at P6.

---

## Mid-project — order around P4/P5 (long lead time + stock volatility)

### 5. Robot arm — SO-101 follower (~$150–275)
Follower arm ONLY (no leader/teleop pair). Kits go in and out of stock — grab one when available. Build + calibration takes a weekend, so buy ahead of P6.
- [Seeed SO-ARM101 motor kit ($240)](https://www.seeedstudio.com/SO-ARM101-Low-Cost-AI-Arm-Kit-p-6426.html)
- [Seeed printed-part set ($35)](https://www.seeedstudio.com/SO-ARM101-3D-printed-Enclosure-p-6428.html) — or print ourselves on the A1
- [WowRobo kit ($199, stock varies)](https://shop.wowrobo.com/products/so-arm101-diy-kit-assembled-version-1)
- [PartaBot follower-only (US)](https://partabot.com/products/so-arm101-follower-only)

### 6. I/O bridge — Raspberry Pi Pico 2 ($5)
Reads door/part switches + force pad, shows up as a COM port. Cheap — bundle with any order.
- [Raspberry Pi Pico 2](https://www.raspberrypi.com/products/raspberry-pi-pico-2/) (reseller links on page: PiShop, Adafruit, Micro Center)

### 7. Switches — KW12-3 roller microswitch 10-pack (~$8)
Door-closed + part-presence sensing.
- [HiLetgo KW12-3 10-pack (Amazon)](https://www.amazon.com/HiLetgo-KW12-3-Roller-Switch-Normally/dp/B07X142VGC)

### 8. Force pad — Adafruit Round FSR ($3.95)
Gripper “actually squeezed something” feedback.
- [Adafruit product 166](https://www.adafruit.com/product/166)

---

## Capstone — order when P6 starts

### 9. Mini-CNC — SainSmart Genmitsu 3018-PROVer V2 ($239)
Factory limit switches, e-stop, Z-probe, GRBL 1.1 with homing — the machine being tended.
- [SainSmart direct](https://www.sainsmart.com/products/genmitsu-3018-prover-v2-upgraded-semi-assembled-cnc-router-kit)
- [Amazon (full kit)](https://www.amazon.com/Genmitsu-3018-PROVer-Beginner-Emergency-Stop-Spoilboard/dp/B0CMTJ6CZC)
- [Amazon (no offline controller, may be ~$200)](https://www.amazon.com/SainSmart-Genmitsu-3018-PROVer-Switches-Emergency-Stop/dp/B07ZFD6SKP)

### 10. Blanks — machinable wax (~$25)
Cut into ~40×40×20mm blanks; chips remelt into new blanks — near-zero consumable cost, near-zero dust.
- [machinablewax.com](https://machinablewax.com/) (or Amazon: “machinable wax block”)

---

**Totals: ~$530–650 without printer · ~$870–1,050 with.**
Recommended path: hybrid — $35 printed-part set + motor kit + the A1 (printer earns its keep on fixtures, not arm parts).
