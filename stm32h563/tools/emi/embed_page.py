#!/usr/bin/env python3
"""Turn the EMI build's lab page (lab_page.html) into a C array for the firmware (src/emi_http.h).

  python3 embed_page.py lab_page.html build-emi/gen/emi_page.c
"""
import sys


def main(src, dst):
    with open(src, "rb") as f:
        data = f.read()
    lines = ["/* Generated from tools/emi/lab_page.html by tools/emi/embed_page.py. Do not edit. */",
             "#include <stddef.h>",
             "const unsigned char emi_page_html[] = {"]
    for i in range(0, len(data), 20):
        lines.append("    " + ",".join(str(b) for b in data[i:i + 20]) + ",")
    lines.append("};")
    lines.append(f"const size_t emi_page_html_len = {len(data)}u;")
    with open(dst, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
