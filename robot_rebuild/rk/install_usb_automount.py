"""Install a user-selected recording-disk mount on the RK; never format a disk.

Run with sudo on the RK, with robot-usb.mount.in beside this file, the known
USB attached, and recording stopped. Other USB disks are not configured.
"""

import json
import os
import re
from pathlib import Path
import pwd
import shutil
import subprocess
import tempfile
import time


UUID = ""
MOUNT = Path("/media/ubuntu/robot-usb")
APP = Path("/home/ubuntu/fpga-hdmi-viewer")


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def find_usb():
    data = json.loads(run("lsblk", "--json", "--paths", "--output",
                          "NAME,TYPE,TRAN,MOUNTPOINT,UUID,FSTYPE"))
    matches = []

    def visit(node, transport=None):
        transport = node.get("tran") or transport
        if node.get("uuid") == UUID:
            matches.append((node, transport))
        for child in node.get("children", []):
            visit(child, transport)

    for node in data["blockdevices"]:
        visit(node)
    if len(matches) != 1:
        raise RuntimeError("Attach exactly one USB partition with UUID " + UUID)
    node, transport = matches[0]
    if transport != "usb" or node.get("fstype") != "vfat":
        raise RuntimeError("Expected the known USB FAT32 partition")
    if node.get("mountpoint") not in (None, str(MOUNT)):
        raise RuntimeError("USB is mounted elsewhere; stop recording and unmount first")
    return node


def check_recording_idle():
    path = APP / "camera-status.json"
    if path.exists():
        if time.time() - path.stat().st_mtime > 10:
            raise RuntimeError("Camera status is stale; verify recording has stopped")
        recording = json.loads(path.read_text())["recording"]
        if recording.get("active") or recording.get("state") in (
                "starting", "recording", "stopping"):
            raise RuntimeError("Stop and save recording before changing the mount")


def install():
    if not re.fullmatch(r"[A-Fa-f0-9]{4}-[A-Fa-f0-9]{4}", UUID):
        raise ValueError("Pass the actual FAT32 partition UUID with --uuid (format XXXX-XXXX)")
    if os.geteuid() != 0:
        raise RuntimeError("Run this installer with sudo on the RK")
    account = pwd.getpwnam("ubuntu")
    if (account.pw_uid, account.pw_gid) != (1000, 1000):
        raise RuntimeError("Mount options require ubuntu uid/gid 1000")
    node = find_usb()
    check_recording_idle()
    for directory in (MOUNT, *MOUNT.parents):
        if directory.is_symlink():
            raise RuntimeError("Refusing symlink mount directory: " + str(directory))
    if MOUNT.is_mount():
        actual = run("findmnt", "--noheadings", "--mountpoint", str(MOUNT),
                     "--output", "SOURCE")
        if Path(actual).resolve() != Path(node["name"]).resolve():
            raise RuntimeError("Mountpoint belongs to a different device")
    source = Path(__file__).with_name("robot-usb.mount.in")
    unit_name = run("systemd-escape", "--path", "--suffix=mount", str(MOUNT))
    device_unit = run("systemd-escape", "--path", "--suffix=device", "/dev/disk/by-uuid/" + UUID)
    template = source.read_text(encoding="utf-8")
    if "@UUID@" not in template or "@DEVICE_UNIT@" not in template:
        raise RuntimeError("Expected the generic robot-usb.mount.in template")
    unit_text = template.replace("@UUID@", UUID).replace("@DEVICE_UNIT@", device_unit)
    destination = Path("/etc/systemd/system") / unit_name
    # Validate with the real escaped Linux filename before changing the system.
    with tempfile.TemporaryDirectory(prefix="robot-usb-unit-") as stage:
        candidate = Path(stage) / unit_name
        candidate.write_text(unit_text, encoding="utf-8")
        subprocess.run(["systemd-analyze", "verify", str(candidate)], check=True)
    if destination.exists():
        backup = Path(tempfile.mkdtemp(prefix="robot-usb-automount-", dir="/var/backups"))
        shutil.copy2(destination, backup / unit_name)
        print("Previous unit saved in", backup)
    destination.write_text(unit_text, encoding="utf-8")
    destination.chmod(0o644)
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    subprocess.run(["systemctl", "enable", unit_name], check=True)
    MOUNT.mkdir(parents=True, exist_ok=True)
    check_recording_idle()
    os.sync()
    if MOUNT.is_mount():
        # Ordinary unmount first: a busy recorder must abort migration rather
        # than detach a still-active recording with the unit's lazy cleanup.
        subprocess.run(["umount", str(MOUNT)], check=True)
    # Wait for PID1 to finish observing the external clean unmount before
    # starting the native unit. The volume is already detached at this point.
    subprocess.run(["systemctl", "stop", unit_name], check=True)
    subprocess.run(["systemctl", "start", unit_name], check=True)
    print(run("findmnt", "--mountpoint", str(MOUNT), "--output",
              "TARGET,SOURCE,FSTYPE,OPTIONS"))
    print("Automatic mount enabled for UUID", UUID)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uuid", required=True, help="Actual FAT32 USB partition UUID from lsblk -f")
    UUID = parser.parse_args().uuid
    install()
