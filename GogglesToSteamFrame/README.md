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

## Prior art: open-source tools that already pull video from Goggles V1/V2

We don't have to reverse-engineer the goggles: this part is solved. All of these use the
same trick, found by the fpv.wtf community: write the 4-byte magic packet `RMVT`
(`0x524d5654`) to the goggles' USB bulk OUT endpoint, then read raw H.264 from the bulk IN
endpoint.

| Project | What it is | Platform | License | Notes for us |
|---|---|---|---|---|
| [fpv-wtf/voc-poc](https://github.com/fpv-wtf/voc-poc) | The original proof of concept (Node.js) | Win / Linux / macOS | MIT | Reference implementation: VID `0x2ca3`, PID `0x1f`, interface `3`, endpoints picked by index (`[0]` OUT, `[1]` IN). Doesn't reconnect if the link drops. |
| [fpvout/DigiView-Android](https://github.com/fpvout/DigiView-Android) | Android app with live preview (uses ExoPlayer) | Android 7+, USB host/OTG | MIT | **Could run directly on the Steam Frame as a sideloaded APK.** Fastest thing to try. |
| [fpvout/DigiView-SBC](https://github.com/fpvout/DigiView-SBC) | Raspberry Pi image; plug in the goggles and it auto-starts (udev → script → fpv-c → GStreamer / hello_video) | Raspberry Pi 2B+ | GPL-3.0 | Ready-made base for **Option C** (Pi as a bridge). Alpha. |
| [fpvout/fpvout-c](https://github.com/fpvout/fpvout-c) | Small C program, dumps the stream to stdout | Linux (libusb) | Apache-2.0 | Good native building block for a Frame/Linux app (Option B). Used by DigiView-SBC. |
| [fpvout/fpvout-go](https://github.com/fpvout/fpvout-go), [fpvout-cpp](https://github.com/fpvout/fpvout-cpp), [live.fpvout.com](https://github.com/fpvout/live.fpvout.com) | Go, C++ and browser (WebUSB) ports | various | see repos | The WebUSB one is interesting if the Frame's browser allows WebUSB. |
| [d4rken/fpv-dvca](https://github.com/d4rken/fpv-dvca) | Android app: live view, DVR, two goggles, **"VR mode"** | Android | GPL-3.0 | Archived (Aug 2023), but its VR mode is a useful reference. |

Known quirks reported by these projects (expect the same for us):
- Turning on USB video out **stops the goggles' own SD-card DVR**.
- Turn **off "Auto Temperature Control"** on the Vista/Air Unit (goggle menu → Settings → Device),
  or the feed can freeze.
- **50 Mbps** mode reportedly works best.
- If the feed freezes or lags, unplug/replug USB (the magic packet has to be sent again after a
  link drop. `goggles_stream.py` re-sends it automatically).
- Some USB-C ↔ USB-C cables don't work. A USB-C OTG adapter + USB-A cable is more reliable.
- Only **FPV Goggles V1/V2** work. The newer **DJI Goggles 2 / Integra / Goggles 3** use a
  different method and aren't supported by these tools.

### What this means for the Steam Frame

The Steam Frame can **sideload Android APKs** (they run through Valve's Lepton compatibility
layer). So the quickest test is to install DigiView-Android on the Frame and plug the goggles in.

Unknowns (none of these are documented anywhere I could find, so test them):
- Does Lepton give Android apps **USB host access** to a plugged-in device? Valve's docs
  only cover ADB, and USB storage reportedly needs a manual workaround. If it doesn't, the
  APK won't see the goggles, and we fall back to a native Linux reader (fpvout-c /
  `goggles_stream.py`) or a Pi bridge.
- Is **hardware video decoding** available to Android apps in Lepton? At least one app
  developer (Whirligig) has reported that GPU video decoding is missing in Lepton, which
  would add latency.
- Does an Android app's window show over **passthrough**, or only in a virtual room?

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

### Option 0 — Sideload DigiView-Android on the Frame (try this first, ~1 hour)

```
Goggles V2 ──USB-C/OTG──▶ Steam Frame running DigiView-Android (APK via Lepton)
```

No code needed. If it works, it's a working prototype straight away. If it fails, the way it
fails tells us something: device not detected means no USB passthrough into Lepton; choppy
or laggy video means software decoding.

### Option A — PC in the middle (known-good fallback)

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
so latency stays lower than Option A. Start from **DigiView-SBC**, which already does the
USB auto-connect part on a Pi, and add a GStreamer `udpsink`/RTP output.

---

## Roadmap

0. **Quick test – DigiView-Android on the Frame.** Sideload the APK, plug in the goggles,
   and write down what happens (see Option 0).
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
