#!/usr/bin/env python3
"""
Convert HTML pages into C string-literal headers for the PTP Slave firmware.

Two embed targets:
   ota.html   -> ota_page.h    (constant: OTA_PAGE_HTML, served at GET /ota)
   index.html -> status_page.h (constant: STATUS_PAGE_HTML, served at GET /)

Usage:
   build_ota_page.py ota.html ota_page.h
   build_ota_page.py index.html status_page.h
   build_ota_page.py --both    (uses ./ota.html, ./index.html in CWD)

The script auto-detects which header to generate from the *output* filename:
if the output ends with `ota_page.h` it picks the OTA constant + favicon;
if it ends with `status_page.h` it uses the STATUS constant and skips the
favicon injection (the status page's index.html already contains its own).
"""

import os
import sys

# ── OTA favicon (orange/red, was injected by the master's build script) ──
# Kept here because the legacy master ota.html relied on this script to add
# it. The slave's ota.html already has a favicon line, so injection is a
# no-op for any ota.html that already contains 'rel="icon"'.
OTA_FAVICON = (
    '<link rel="icon" type="image/svg+xml" '
    'href="data:image/svg+xml,%3Csvg%20xmlns=\'http://www.w3.org/2000/svg\'%20viewBox=\'0%200%2064%2064\'%3E'
    '%3Cdefs%3E%3ClinearGradient%20id=\'g\'%20x1=\'0\'%20y1=\'0\'%20x2=\'1\'%20y2=\'1\'%3E'
    '%3Cstop%20offset=\'0\'%20stop-color=\'%23ffb86c\'/%3E'
    '%3Cstop%20offset=\'1\'%20stop-color=\'%23ff5577\'/%3E'
    '%3C/linearGradient%3E%3C/defs%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'32\'%20r=\'28\'%20fill=\'%230b1018\'%20stroke=\'url(%23g)\'%20stroke-width=\'4\'/%3E'
    '%3Cpath%20d=\'M%2032%2050%20L%2032%2022%20M%2022%2032%20L%2032%2022%20L%2042%2032\'%20stroke=\'%23ffb86c\'%20stroke-width=\'5\'%20stroke-linecap=\'round\'%20stroke-linejoin=\'round\'%20fill=\'none\'/%3E'
    '%3C/svg%3E">'
)

HEADER_TEMPLATES = {
    "ota_page.h": {
        "constant": "OTA_PAGE_HTML",
        "guard": "OTA_PAGE_H",
        "inject_favicon": True,
        "comment": (
            "/* ============================================================================\n"
            " *  ota_page.h  --  Embedded HTML for the PTP Slave OTA firmware update page\n"
            " *\n"
            " *  Served at GET /ota by the web server. The page POSTs the firmware .bin\n"
            " *  to /api/ota and polls /api/ota/info for partition status. See\n"
            " *  ota_handler.h for the C-side endpoints.\n"
            " *\n"
            " *  Project: https://github.com/Montecri/GNSSTimeServer\n"
            " *\n"
            " *  NOTE: Auto-generated from ota.html. Do not edit by hand -- re-run\n"
            " *        build_ota_page.py whenever ota.html changes, then rebuild & flash.\n"
            " * ============================================================================ */\n"
        ),
    },
    "status_page.h": {
        "constant": "STATUS_PAGE_HTML",
        "guard": "STATUS_PAGE_H",
        "inject_favicon": False,
        "comment": (
            "/* ============================================================================\n"
            " *  status_page.h  --  Embedded HTML status page for ESP32-P4 PTP Slave\n"
            " *\n"
            " *  This header embeds, as a single C string literal, a self-contained\n"
            " *  HTML / CSS / JavaScript page that renders runtime diagnostics for the\n"
            " *  PTPv2 slave running on this device. The page polls /api/status every\n"
            " *  5 seconds (XHR/fetch) and updates every field in place.\n"
            " *\n"
            " *  Project: https://github.com/Montecri/GNSSTimeServer\n"
            " *\n"
            " *  NOTE: Auto-generated from index.html. Do not edit by hand -- re-run\n"
            " *        build_ota_page.py whenever index.html changes, then rebuild & flash.\n"
            " * ============================================================================ */\n"
        ),
    },
}


def stringify_line(line: str) -> str:
    """Escape a single line of HTML for inclusion in a C string literal."""
    line = line.rstrip("\r\n")
    line = line.replace("\\", "\\\\").replace('"', '\\"')
    return f'    "{line}\\n"\n'


def detect_template(header_path: str):
    """Pick a template by basename of the output header."""
    base = os.path.basename(header_path)
    for key, tmpl in HEADER_TEMPLATES.items():
        if base.endswith(key):
            return key, tmpl
    raise SystemExit(
        f"error: don't know how to build '{base}'. "
        f"Expected output filename ending in one of: {list(HEADER_TEMPLATES)}"
    )


def convert(html_path: str, header_path: str) -> None:
    key, tmpl = detect_template(header_path)

    with open(html_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    out = []
    out.append(tmpl["comment"])
    out.append("\n")
    out.append(f"#ifndef {tmpl['guard']}\n")
    out.append(f"#define {tmpl['guard']}\n")
    out.append("\n")
    out.append("#ifdef __cplusplus\n")
    out.append("extern \"C\" {\n")
    out.append("#endif\n")
    out.append("\n")
    out.append(f"static const char {tmpl['constant']}[] =\n")

    inject = tmpl["inject_favicon"]
    # If the HTML already contains a favicon link, skip injection regardless.
    if inject and any('rel="icon"' in ln or "rel='icon'" in ln for ln in lines):
        inject = False

    favicon_inserted = False
    for line in lines:
        out.append(stringify_line(line))
        if inject and not favicon_inserted and 'name="viewport"' in line:
            out.append(stringify_line(OTA_FAVICON))
            favicon_inserted = True

    out.append("    ;\n")
    out.append("\n")
    out.append("#ifdef __cplusplus\n")
    out.append("}\n")
    out.append("#endif\n")
    out.append("\n")
    out.append(f"#endif /* {tmpl['guard']} */\n")

    with open(header_path, "w", encoding="utf-8") as f:
        f.writelines(out)

    print(f"Wrote {header_path}  ({key} pattern, {len(lines)} HTML lines)")
    if tmpl["inject_favicon"] and not favicon_inserted and not inject:
        # We chose not to inject because the HTML already had a favicon line.
        pass
    elif tmpl["inject_favicon"] and not favicon_inserted:
        print(
            "WARNING: viewport meta tag not found; favicon was not inserted.",
            file=sys.stderr,
        )


def main(argv):
    if len(argv) == 2 and argv[1] == "--both":
        convert("ota.html", "ota_page.h")
        convert("index.html", "status_page.h")
        return
    if len(argv) != 3:
        print(
            f"usage: {argv[0]} <input.html> <output.h>\n"
            f"   or: {argv[0]} --both    (builds both pages from CWD)\n",
            file=sys.stderr,
        )
        sys.exit(1)
    convert(argv[1], argv[2])


if __name__ == "__main__":
    main(sys.argv)
