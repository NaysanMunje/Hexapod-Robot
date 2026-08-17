# Rebuild gait preview headers from www/*.html
# Run after editing walk or spin preview pages:
#   python gen_walk_html.py
from pathlib import Path

ROOT = Path(__file__).resolve().parent

PAGES = [
    ("www/walk.html", "include/gait_preview_html.h", "WALK_HTML"),
    ("www/spin.html", "include/gait_spin_html.h", "SPIN_HTML"),
]


def write_header(html_path: Path, out_path: Path, var_name: str) -> None:
    html = html_path.read_text(encoding="utf-8")
    if ")HTML" in html:
        raise SystemExit(f"{html_path.name} contains )HTML — pick another delimiter")
    out_path.write_text(
        "#pragma once\n"
        "#include <Arduino.h>\n\n"
        f'static const char {var_name}[] PROGMEM = R"HTML(\n'
        + html
        + '\n)HTML";\n',
        encoding="utf-8",
    )
    print(f"wrote {out_path} ({out_path.stat().st_size} bytes)")


for html_rel, out_rel, var in PAGES:
    write_header(ROOT / html_rel, ROOT / out_rel, var)
