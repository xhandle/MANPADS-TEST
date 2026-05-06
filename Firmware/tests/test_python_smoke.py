import py_compile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TARGETS = [
  "dashboard.py",
  "Rocket/.pio/libdeps/rocket/Unity/platformio-build.py",
  "Rocket/.pio/libdeps/rocket/Unity/auto/__init__.py",
  "Rocket/.pio/libdeps/rocket/Unity/auto/extract_version.py",
  "Rocket/.pio/libdeps/rocket/Unity/auto/stylize_as_junit.py",
  "Rocket/.pio/libdeps/rocket/Unity/auto/unity_test_summary.py",
  "Rocket/.pio/libdeps/rocket/Adafruit SSD1306/scripts/make_splash.py",
  "Rocket/.pio/libdeps/rocket/Adafruit GFX Library/fontconvert/bdf2adafruit.py"
]

def test_python_sources_compile():
    for relative_path in TARGETS:
        py_compile.compile(str(ROOT / relative_path), doraise=True)

