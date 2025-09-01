import importlib.metadata
import json
from pathlib import Path
import platform


root = Path(__file__).resolve().parents[1]
record = json.loads((root / "docs/dependencies.json").read_text())
if platform.python_version() != "3.12.11":
    raise SystemExit(f"Expected Python 3.12.11, found {platform.python_version()}")
for package in record["packages"]:
    name, expected = package["package"], package["version"]
    actual = importlib.metadata.version(name)
    if actual != expected:
        raise SystemExit(f"Expected {name}=={expected}, found {actual}")
    if package["latest_allowed_upload"] >= record["cutoff"]:
        raise SystemExit(f"Distribution newer than the cutoff: {name}")
print("Python 3.12.11 and all 12 pinned packages match the 2025 environment record.")
