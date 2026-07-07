# Converts the Open Duck Mini v2 URDF into USD for Gemelli, using Isaac Sim's URDF importer
# (Isaac Sim 6.x API: URDFImporter/URDFImporterConfig).
# Run with Isaac Sim's bundled Python:
#   C:\isaacsim\python.bat tools\import_duck_urdf.py
# Output: scenes\duck\robot\robot.usda (gitignored; regenerate any time).

import os
import sys

URDF = r"C:\DataDrive\Open_Duck_Mini\mini_bdx\robots\open_duck_mini_v2\robot.urdf"
DEST_DIR = r"C:\DataDrive\ovGemelli\scenes\duck"

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.app

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

os.makedirs(DEST_DIR, exist_ok=True)

config = URDFImporterConfig(
    urdf_path=URDF,
    usd_path=DEST_DIR,
    merge_fixed_joints=False,
    fix_base=False,                      # floating-base biped
    joint_drive_type="force",
    joint_target_type="position",
    override_joint_stiffness=13.37,      # sts3215 servo kp from the training MJCF
    override_joint_damping=0.3,          # small damping for PhysX stability (MJCF kv=0)
)

try:
    usd_path = URDFImporter(config).import_urdf()
    print(f"[duck] wrote {usd_path}")
    ok = bool(usd_path) and os.path.exists(usd_path)
except Exception as e:  # noqa: BLE001
    print(f"[duck] import failed: {e}", file=sys.stderr)
    ok = False

app.close()
sys.exit(0 if ok else 1)
