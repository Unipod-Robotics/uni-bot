#!/usr/bin/env python3
"""Render a docs/*.md file to a print-quality PDF (Markdown -> HTML -> headless Chrome).

Usage: ~/uni-bot/.venv-bench/bin/python scripts/protocol_pdf.py [OUT.pdf] [--src docs/X.md]
Default: docs/PROTOCOL.md -> docs/PROTOCOL.pdf
Needs `markdown` and `pymdown-extensions` (superfences: code blocks inside list items) in the
benchmark venv, and google-chrome or chromium.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

import markdown

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'docs', 'PROTOCOL.md')

CSS = """
@page { size: A4; margin: 18mm 17mm 18mm 17mm; }
body { font-family: 'DejaVu Sans', Arial, sans-serif; font-size: 9.6pt; line-height: 1.45;
       color: #0b0b0b; }
h1 { font-size: 18pt; margin: 0 0 4pt 0; }
h2 { font-size: 13pt; margin: 16pt 0 5pt 0; padding-bottom: 2pt; border-bottom: 1px solid #d9d8d3;
     page-break-after: avoid; }
h3 { font-size: 10.5pt; margin: 11pt 0 4pt 0; page-break-after: avoid; }
p, li { margin: 3pt 0; }
table { border-collapse: collapse; width: 100%; margin: 6pt 0 8pt 0; font-size: 8.3pt;
        page-break-inside: auto; }
th { background: #f4f4f1; text-align: left; border-bottom: 1.2px solid #52514e; }
th, td { padding: 3pt 4pt; vertical-align: top; border-bottom: 0.5px solid #d9d8d3; }
.url { overflow-wrap: anywhere; word-break: break-all; }
tr { page-break-inside: avoid; }
code { font-family: 'DejaVu Sans Mono', monospace; font-size: 8.3pt; background: #f4f4f1;
       padding: 0 2px; border-radius: 2px; }
pre { background: #f4f4f1; padding: 6pt; font-size: 8pt; border-left: 2px solid #2a78d6;
      white-space: pre-wrap; }
pre code { background: none; padding: 0; }
hr { border: none; border-top: 1px solid #d9d8d3; margin: 10pt 0; }
strong { color: #0b0b0b; }
a { color: #1f5fae; text-decoration: none; }
"""


LIST = re.compile(r'^(\s*)([-*]|\d+\.)\s')


def gfm_to_python_markdown(text):
    """GitHub renders lists that follow a paragraph line directly and nests them at 2 spaces;
    Python-Markdown needs a blank line before a list and 4-space nesting. Convert (outside code
    fences) so the .md source can stay GitHub-friendly."""
    out, in_code, in_list, shift = [], False, False, 0
    for line in text.split('\n'):
        if line.lstrip().startswith('```'):
            if not in_code:
                lead = len(line) - len(line.lstrip())
                # a fence inside a list item: indent it like the item's text (doubled)
                shift = lead if (in_list and lead) else 0
                if shift and out and out[-1].strip():
                    out.append('')
            in_code = not in_code
            out.append(' ' * shift + line)
            if not in_code:
                shift = 0
            continue
        if in_code:
            out.append(' ' * shift + line if line.strip() else line)
            continue
        m = LIST.match(line)
        if m:
            indent = len(m.group(1))
            prev = out[-1] if out else ''
            pm = LIST.match(prev)
            sibling = pm is not None and len(pm.group(1)) == indent * 2
            # Python-Markdown needs a blank line before a list item unless it directly follows a
            # sibling item (after continuation text, a code block or a parent item it merges).
            if prev.strip() and not prev.lstrip().startswith('|') and not sibling:
                out.append('')
            in_list = True
            out.append(' ' * (indent * 2) + line.lstrip())
        elif in_list and line.startswith('  ') and line.strip():
            lead = len(line) - len(line.lstrip())
            out.append(' ' * (lead * 2) + line.lstrip())
        else:
            if not line.strip():
                in_list = False
            elif in_list:
                in_list = False
            out.append(line)
    # long URLs may break anywhere; ordinary words never do
    return re.sub(r'(?<![(<"])(https?://[^\s|<>]+)', r'<span class="url">\1</span>', '\n'.join(out))


def main():
    args = sys.argv[1:]
    src = SRC
    if '--src' in args:
        i = args.index('--src')
        src = os.path.abspath(args[i + 1])
        del args[i:i + 2]
    default_out = os.path.splitext(src)[0] + '.pdf'
    out = args[0] if args else default_out
    with open(src) as f:
        body = markdown.markdown(gfm_to_python_markdown(f.read()),
                                 extensions=['tables', 'pymdownx.superfences', 'sane_lists'])
    html = (f'<!doctype html><html><head><meta charset="utf-8"><title>ubot benchmark protocol'
            f'</title><style>{CSS}</style></head><body>{body}</body></html>')
    chrome = shutil.which('google-chrome') or shutil.which('chromium') or \
        shutil.which('chromium-browser')
    if not chrome:
        raise SystemExit('need google-chrome or chromium')
    with tempfile.TemporaryDirectory() as d:
        page = os.path.join(d, 'protocol.html')
        with open(page, 'w') as f:
            f.write(html)
        subprocess.run([chrome, '--headless=new', '--disable-gpu', '--no-pdf-header-footer',
                        f'--user-data-dir={d}/profile', f'--print-to-pdf={os.path.abspath(out)}',
                        'file://' + page], check=True, capture_output=True)
    print('wrote', os.path.abspath(out))


if __name__ == '__main__':
    main()
