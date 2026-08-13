"""Apply the one ESP32 compatibility fix required by DW1000 v0.9."""

from pathlib import Path

Import("env")


environment = env.subst("$PIOENV")
library_root = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / environment / "DW1000"
source_path = library_root / "src" / "DW1000.cpp"

if not source_path.is_file():
    raise RuntimeError(f"DW1000 v0.9 source was not installed at {source_path}")

source = source_path.read_text(encoding="utf-8")
upstream = """#ifndef ESP8266
\tSPI.usingInterrupt(digitalPinToInterrupt(irq)); // not every board support this, e.g. ESP8266
#endif"""
esp32_compatible = """#if !defined(ESP8266) && !defined(ESP32)
\tSPI.usingInterrupt(digitalPinToInterrupt(irq)); // unavailable on ESP8266 and ESP32
#endif"""

if esp32_compatible not in source:
    if upstream not in source:
        raise RuntimeError("Unexpected DW1000 v0.9 SPI interrupt code")
    source_path.write_text(source.replace(upstream, esp32_compatible, 1), encoding="utf-8")
    print("Patched DW1000 v0.9 for ESP32 SPI compatibility")
