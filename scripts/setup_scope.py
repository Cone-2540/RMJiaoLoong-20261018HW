"""Configure Scope v0.4.5 through its local API and install project presets."""
import argparse
import json
import shutil
import time
import urllib.request
import webbrowser
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--scope-dir", type=Path, default=Path.home()/"Desktop/Scope-v0.4.5")
parser.add_argument("--url", default="http://127.0.0.1:8001")
parser.add_argument("--no-browser", action="store_true")
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
elf = root/"MDK-ARM/20261018/20261018.axf"
openocd = shutil.which("openocd")
if not elf.is_file():
    raise SystemExit("Build MDK-ARM/20261018.uvprojx in Keil first.")
if not openocd:
    raise SystemExit("openocd.exe must be on PATH.")


def api(path, data=None):
    request = urllib.request.Request(args.url+path,
        data=json.dumps(data).encode() if data is not None else None,
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.load(response)


api("/api/openocd/settings", {
    "bin_path": openocd, "config_files": str(args.scope_dir/"_internal/openocd/stlink.cfg"),
    "working_dir": str(args.scope_dir), "mode": "managed", "auto_start": True,
    "telnet_port": 4444, "tcl_port": 6666, "rpc_protocol": "auto", "profile_id": "stlink"})
session = api("/api/session")
if not session.get("connected") or Path(session.get("elf_path", "")) != elf:
    api("/api/openocd/start", {})
    session = api("/api/session/connect", {"profile_id": "stlink", "elf_path": str(elf)})
if not session.get("connected"):
    raise SystemExit("ST-Link connection failed: "+str(session))

uints = ["imu_init_ok", "imu_acc_id", "imu_gyro_id", "imu_calibrated", "imu_calibration_count",
         "imu_sample_count", "bmi088_dma_count", "bmi088_spi_fail_count", "imu_skipped_ticks",
         "imu_algorithm", "imu_recalibrate", "imu_use_dma"]
common = ["imu_sample_hz", "imu_dt_ms"]+uints
groups = {
    "angles": ["roll_deg", "pitch_deg", "yaw_deg"],
    "filters": [f"{f}_{a}_deg" for f in ["comp", "mahony"] for a in ["roll", "pitch", "yaw"]],
    "bias": [f"{p}{a}_dps" for a in ["x", "y", "z"] for p in ["gyro_uncal_", "gyro_"]],
    "sensors": ["acc_x", "acc_y", "acc_z", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"],
}
projects = {}
for group, names in groups.items():
    hidden = common+[f"bias_{a}_dps" for a in ["x", "y", "z"]]
    projects[group] = {
        "version": 3, "elfPath": str(elf), "updatedAt": int(time.time()*1000),
        "alias": "BMI088 IMU", "profileId": "stlink", "probe": "stlink", "target": "stm32f4",
        "sampleHz": 50,
        "variables": [{"id": "symbol:"+n, "name": n, "symbolName": n,
                       "type": "uint32" if n in uints else "float", "size": 4,
                       "show": n in names, "expandable": False, "isPointer": False}
                      for n in dict.fromkeys(names+hidden)]}
payload = {"projectKey": str(elf).replace("\\", "/").lower(), "presets": projects}
page = (root/"scripts/scope_setup.html").read_text(encoding="utf-8")
page = page.replace("/*PRESETS*/null", json.dumps(payload, ensure_ascii=False))
target = args.scope_dir/"_internal/frontend/assets/imu-setup.html"
target.write_text(page, encoding="utf-8")
url = args.url+"/assets/imu-setup.html"
# Confirm Scope serves the configuration page at the same origin as its UI.
with urllib.request.urlopen(url, timeout=5) as response:
    if b"imu-scope-setup" not in response.read():
        raise SystemExit("Scope did not serve the preset page.")
print("Connected to real ST-Link; matching Keil AXF loaded.")
print("Preset page: "+url)
if not args.no_browser:
    webbrowser.open(url+"?preset=angles")
