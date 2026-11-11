# Goggles → Steam Frame (side project, experimental)

> **This is a separate project from the Virtual-Vue gimbal.**
> Nothing in here is needed to build or run the 3-axis gimbal (the `CAD/` folder and
> `VirtualVueIntialCodeBase.py` in the repo root). This folder is an exploration of a
> different idea that happens to live in the same repo.

## The idea

Use a **DJI FPV Goggles V2** as a *video receiver box* only, pull the live FPV feed out of it,
and show it inside a **Valve Steam Frame** headset as a floating screen, with the Frame's
**passthrough** cameras showing the real world around it.

```
 Drone / Air Unit ──5.8 GHz──▶ DJI Goggles V2 ──USB-C──▶ (decoder) ──▶ Steam Frame
                               (sitting on the     (raw H.264)        floating FPV window
                                table / belt)                         over passthrough
```

Why: wear one headset, see your surroundings (spotters, the drone line-of-sight, your
radio) and the FPV feed at the same time, without swapping goggles.

Status: **research / proof-of-concept**. No part of this has been tested end-to-end on a
Steam Frame yet.

---

## What we know (and what still needs verifying)

### Getting video *out* of the Goggles V2

- The V2 goggles have **no HDMI out** (the V1 goggles did). The only practical path is the
  **USB-C port**.
- The FPV community (fpv.wtf, the `voc-poc` project and Android apps built on it) found that
  the goggles will stream the **raw H.264 live feed over USB** after the host sends a small
  "magic" packet (`RMVT`) to a USB bulk endpoint. This is the same mechanism DJI uses for
  phone live-view.
- The stream is plain H.264 elementary stream, so any decoder (ffmpeg, GStreamer, hardware
  decoders) can handle it.
- No goggle rooting is required for this path.

To verify on real hardware:
- [ ] USB IDs / interface / endpoints on *your* goggle firmware (`lsusb -v`, see below).
- [ ] Whether the feed keeps streaming with the goggles' screens off / face sensor covered.
- [ ] Added latency of the USB path vs. watching the goggles directly.

### The Steam Frame side

- The Steam Frame is a standalone headset running **SteamOS (Linux on ARM)**, and can also
  act as a PC VR headset streaming from SteamVR.
- It has **passthrough cameras** (reported as monochrome), so a "video window in the room"
  setup is possible.
- It has a **USB-C port**. Whether that port can act as a **USB host** for an arbitrary device
  (the goggles) from user-space apps is **unknown — this is the #1 thing to test.**

To verify on real hardware:
- [ ] Does the Frame's USB-C port enumerate the goggles in host mode? (`lsusb` in desktop mode)
- [ ] Can user apps access raw USB (libusb / pyusb), or are udev rules / permissions needed?
- [ ] Hardware H.264 decode available to apps (V4L2 / Vulkan Video / GStreamer)?
- [ ] How apps draw a quad layer over passthrough (OpenXR passthrough extension vs. SteamVR
      overlay with room view).

---

## Possible architectures

### Option A — PC in the middle (easiest prototype, start here)

```
Goggles V2 ──USB──▶ PC (goggles_stream.py ──▶ decoder) ──▶ SteamVR overlay ──wireless──▶ Steam Frame
```

1. Plug goggles into a Linux/Windows PC.
2. Run `usb_video_out/goggles_stream.py` and pipe it to `ffplay` / a decoder.
3. Show that window in VR as a SteamVR overlay (e.g. a desktop/window overlay tool, or a
   small custom OpenVR overlay app) and enable passthrough / room view.

Pros: uses only known-working pieces. Cons: extra latency (USB → decode → re-encode →
wireless to the Frame), needs a PC nearby.

### Option B — Goggles plugged straight into the Frame (the real goal)

```
Goggles V2 ──USB-C──▶ Steam Frame (libusb reader ──▶ HW H.264 decode ──▶ OpenXR quad over passthrough)
```

1. Goggles connect directly to the Frame's USB-C (possibly via a powered hub so the Frame
   can still charge).
2. A small native app on the Frame does the `RMVT` handshake, reads the stream, decodes it
   in hardware, and renders it as a quad layer with passthrough enabled.

Pros: standalone, lowest latency. Cons: depends on the unknowns listed above.

### Option C — Fallback if the Frame can't host USB

Put a small SBC (e.g. a Raspberry Pi 5) between them: it reads the goggles over USB and
re-streams the untouched H.264 over Wi-Fi/UDP to an app on the Frame. No re-encode needed,
so latency stays lower than Option A.

---

## Roadmap

1. **Phase 0 – Capture on a PC.** Get `goggles_stream.py` → `ffplay` showing live video.
   Measure glass-to-glass latency (film a stopwatch through both paths).
2. **Phase 1 – Option A in VR.** Show the feed on the Frame via a SteamVR overlay + passthrough.
3. **Phase 2 – Probe the Frame.** Desktop mode → plug in goggles → `lsusb`, try the script.
4. **Phase 3 – Native Frame app (Option B or C).** OpenXR app: USB/UDP in → HW decode →
   quad layer over passthrough. Add head-locked vs. world-locked screen modes.
5. **Stretch – tie back to Virtual-Vue.** Send the Frame's head pose to the gimbal so the
   camera follows your head. (Optional; keeps the two projects independent until then.)

---

## Running the PC capture prototype (Phase 0)

```bash
cd GogglesToSteamFrame/usb_video_out
pip install -r requirements.txt

# Linux: allow non-root access to the goggles (once)
sudo cp 99-dji-goggles.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger

# Power on goggles + drone/air unit, plug goggles USB-C into the PC, then:
python goggles_stream.py | ffplay -fflags nobuffer -flags low_delay -framedrop -probesize 32 -f h264 -i -

# or save a raw clip to inspect
python goggles_stream.py --out clip.h264 --seconds 10
```

If the device isn't found, run `lsusb` / `lsusb -v -d 2ca3:` and pass the right values with
`--vid --pid --interface --ep-out --ep-in`. The defaults come from community reverse
engineering and may differ between firmware versions.

Windows: you'll need a WinUSB driver bound to the goggles' bulk interface (e.g. with Zadig)
before pyusb can open it.

## Hardware list

- DJI FPV Goggles V2 + a compatible video transmitter (DJI FPV drone, Air Unit, Vista)
- Valve Steam Frame
- USB-C ↔ USB-C (or USB-A) data cable; possibly a powered USB-C hub
- (Option C) Raspberry Pi 5 or similar

## Safety / legal

- Flying while wearing a headset still counts as FPV flying: follow local rules about
  spotters / visual line of sight.
- This is unofficial use of DJI hardware; nothing here modifies goggle firmware.
