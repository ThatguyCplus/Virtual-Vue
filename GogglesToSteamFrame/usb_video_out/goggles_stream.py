#!/usr/bin/env python3
"""
Pull the live H.264 FPV feed out of DJI FPV Goggles V2 over USB.

Part of the *Goggles -> Steam Frame* side project (separate from the Virtual-Vue gimbal).

How it works: the goggles expose a USB bulk interface. After the host writes the 4-byte
magic packet b"RMVT" to the OUT endpoint, the goggles start sending the raw H.264
elementary stream on the IN endpoint. This was found by the FPV community: see
https://github.com/fpv-wtf/voc-poc (MIT), and DigiView / fpvout-c for other ports.
VID/PID/interface match voc-poc. voc-poc picks the endpoints by index rather than by
address; this script does the same (override with --ep-out/--ep-in if needed).

Usage:
    python goggles_stream.py | ffplay -fflags nobuffer -flags low_delay -framedrop -f h264 -i -
    python goggles_stream.py --out clip.h264 --seconds 10
"""

import argparse
import sys
import time

import usb.core
import usb.util

DEFAULT_VID = 0x2CA3   # DJI
DEFAULT_PID = 0x001F   # FPV Goggles V1/V2 (may vary by firmware)
DEFAULT_INTERFACE = 3
MAGIC = b"RMVT"

READ_SIZE = 512 * 1024
READ_TIMEOUT_MS = 500
RESEND_AFTER_S = 2.0   # re-send the magic packet if no data arrives for this long


def log(msg):
    print(f"[goggles] {msg}", file=sys.stderr, flush=True)


def open_goggles(vid, pid, interface):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if dev is None:
        raise SystemExit(
            f"Goggles not found (VID {vid:04x}, PID {pid:04x}). "
            "Are they powered on and plugged in? Check `lsusb`."
        )
    try:
        if dev.is_kernel_driver_active(interface):
            dev.detach_kernel_driver(interface)
    except (NotImplementedError, usb.core.USBError):
        pass  # not supported on Windows / not needed
    usb.util.claim_interface(dev, interface)
    return dev


def find_endpoints(dev, interface):
    """Return (out, in) bulk endpoint addresses of the interface, like voc-poc does."""
    intf = dev.get_active_configuration()[(interface, 0)]
    eps = {usb.util.endpoint_direction(e.bEndpointAddress): e.bEndpointAddress for e in intf}
    try:
        return eps[usb.util.ENDPOINT_OUT], eps[usb.util.ENDPOINT_IN]
    except KeyError:
        raise SystemExit(f"Interface {interface} has no bulk IN/OUT pair; check `lsusb -v -d 2ca3:`")


def stream(dev, ep_out, ep_in, sink, seconds=None):
    start = time.monotonic()
    last_data = 0.0
    total = 0
    dev.write(ep_out, MAGIC)
    log("sent RMVT, waiting for video...")

    while seconds is None or time.monotonic() - start < seconds:
        try:
            chunk = dev.read(ep_in, READ_SIZE, timeout=READ_TIMEOUT_MS)
        except usb.core.USBTimeoutError:
            chunk = None
        if chunk:
            if total == 0:
                log("receiving video")
            sink.write(bytes(chunk))
            sink.flush()
            total += len(chunk)
            last_data = time.monotonic()
        elif time.monotonic() - max(last_data, start) > RESEND_AFTER_S:
            # Goggles stop/never started streaming (e.g. no link to the air unit yet).
            dev.write(ep_out, MAGIC)
            last_data = time.monotonic()
            log("no data, re-sent RMVT (is the drone/air unit linked?)")
    log(f"done, {total / 1e6:.1f} MB received")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--vid", type=lambda s: int(s, 0), default=DEFAULT_VID)
    p.add_argument("--pid", type=lambda s: int(s, 0), default=DEFAULT_PID)
    p.add_argument("--interface", type=int, default=DEFAULT_INTERFACE)
    p.add_argument("--ep-out", type=lambda s: int(s, 0), help="override OUT endpoint address")
    p.add_argument("--ep-in", type=lambda s: int(s, 0), help="override IN endpoint address")
    p.add_argument("--out", help="write to this file instead of stdout")
    p.add_argument("--seconds", type=float, help="stop after N seconds")
    args = p.parse_args()

    dev = open_goggles(args.vid, args.pid, args.interface)
    ep_out, ep_in = find_endpoints(dev, args.interface)
    ep_out = args.ep_out or ep_out
    ep_in = args.ep_in or ep_in
    log(f"using interface {args.interface}, OUT 0x{ep_out:02x}, IN 0x{ep_in:02x}")
    sink = open(args.out, "wb") if args.out else sys.stdout.buffer
    try:
        stream(dev, ep_out, ep_in, sink, args.seconds)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        usb.util.release_interface(dev, args.interface)
        usb.util.dispose_resources(dev)
        if args.out:
            sink.close()


if __name__ == "__main__":
    main()
