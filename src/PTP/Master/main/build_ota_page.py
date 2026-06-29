#!/usr/bin/env python3
"""
Convert ota.html into ota_page.h (C string-literal embedding).
Preserves the favicon <link> line that lives in ota_page.h but not in ota.html.
"""

import sys

FAVICON = (
    '<link rel="icon" type="image/svg+xml" '
    'href="data:image/svg+xml,%3Csvg%20xmlns=\'http://www.w3.org/2000/svg\'%20viewBox=\'0%200%2064%2064\'%3E'
    '%3Cdefs%3E%3ClinearGradient%20id=\'g\'%20x1=\'0\'%20y1=\'0\'%20x2=\'1\'%20y2=\'1\'%3E'
    '%3Cstop%20offset=\'0\'%20stop-color=\'%23ffb86c\'/%3E'
    '%3Cstop%20offset=\'1\'%20stop-color=\'%23ff5577\'/%3E'
    '%3C/linearGradient%3E%3C/defs%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'32\'%20r=\'28\'%20fill=\'%230b1018\'%20stroke=\'url(%23g)\'%20stroke-width=\'4\'/%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'11\'%20r=\'2\'%20fill=\'%23ffb86c\'%20opacity=\'0.6\'/%3E'
    '%3Ccircle%20cx=\'53\'%20cy=\'32\'%20r=\'2\'%20fill=\'%23ffb86c\'%20opacity=\'0.6\'/%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'53\'%20r=\'2\'%20fill=\'%23ffb86c\'%20opacity=\'0.6\'/%3E'
    '%3Ccircle%20cx=\'11\'%20cy=\'32\'%20r=\'2\'%20fill=\'%23ffb86c\'%20opacity=\'0.6\'/%3E'
    '%3Cline%20x1=\'32\'%20y1=\'32\'%20x2=\'32\'%20y2=\'18\'%20stroke=\'%23e6edf7\'%20stroke-width=\'3\'%20stroke-linecap=\'round\'%20opacity=\'0.3\'/%3E'
    '%3Cline%20x1=\'32\'%20y1=\'32\'%20x2=\'42\'%20y2=\'32\'%20stroke=\'%237aa2f7\'%20stroke-width=\'3\'%20stroke-linecap=\'round\'%20opacity=\'0.3\'/%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'32\'%20r=\'3\'%20fill=\'%23e6edf7\'%20opacity=\'0.3\'/%3E'
    '%3Cpath%20d=\'M%2032%2050%20L%2032%2022%20M%2022%2032%20L%2032%2022%20L%2042%2032\'%20stroke=\'%23ffb86c\'%20stroke-width=\'5\'%20stroke-linecap=\'round\'%20stroke-linejoin=\'round\'%20fill=\'none\'/%3E'
    '%3C/svg%3E">'
)

HEADER = '''/* ============================================================================
 *  ota_page.h  --  Embedded HTML for the OTA firmware update page
 *
 *  Served at GET /ota by the web server. The page POSTs the firmware .bin
 *  to /api/ota and polls /api/ota/info for partition status. See
 *  ota_handler.h for the C-side endpoints.
 *
 *  Project: https://github.com/Montecri/GNSSTimeServer
 *
 *  NOTE: Auto-generated from ota.html. Do not edit by hand — re-run
 *        build_ota_page.py whenever ota.html changes, then rebuild & flash.
 * ============================================================================ */

#ifndef OTA_PAGE_H
#define OTA_PAGE_H

#ifdef __cplusplus
extern "C" {
#endif

static const char OTA_PAGE_HTML[] =
'''

FOOTER = '''    ;

#ifdef __cplusplus
}
#endif

#endif /* OTA_PAGE_H */
'''


def stringify_line(line: str) -> str:
    """Escape a single line of HTML for inclusion in a C string literal."""
    # Strip trailing newline; we'll add \n explicitly inside the literal
    line = line.rstrip('\r\n')
    # Escape backslashes and double quotes
    line = line.replace('\\', '\\\\').replace('"', '\\"')
    return f'    "{line}\\n"\n'


def convert(html_path: str, header_path: str) -> None:
    with open(html_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    out_lines = [HEADER]
    favicon_inserted = False

    for line in lines:
        out_lines.append(stringify_line(line))
        # After the viewport meta tag, inject the favicon line if we haven't yet
        if not favicon_inserted and 'name="viewport"' in line:
            out_lines.append(stringify_line(FAVICON))
            favicon_inserted = True

    out_lines.append(FOOTER)

    with open(header_path, 'w', encoding='utf-8') as f:
        f.writelines(out_lines)

    print(f"Wrote {header_path}")
    if not favicon_inserted:
        print("WARNING: viewport meta tag not found; favicon was not inserted.",
              file=sys.stderr)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} ota.html ota_page.h", file=sys.stderr)
        sys.exit(1)
    convert(sys.argv[1], sys.argv[2])