# Rebuild include/gait_preview_html.h from www/walk.html
# Run after editing the walk preview page:
#   python gen_walk_html.py
from pathlib import Path

html_path = Path(__file__).resolve().parent / "www" / "walk.html"
out_path = Path(__file__).resolve().parent / "include" / "gait_preview_html.h"
html = html_path.read_text(encoding="utf-8")
if ")HTML" in html:
    raise SystemExit("walk.html contains )HTML — pick another delimiter")
out_path.write_text(
    "#pragma once\n"
    "#include <Arduino.h>\n\n"
    'static const char WALK_HTML[] PROGMEM = R"HTML(\n'
    + html
    + '\n)HTML";\n',
    encoding="utf-8",
)
print(f"wrote {out_path} ({out_path.stat().st_size} bytes)")
