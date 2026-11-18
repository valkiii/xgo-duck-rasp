"""Wrap index.html (artifact body, no <html>/<head>) into a full standalone page for the portfolio.

    python3 build/standalone.py   ->  story/xgo-duck-story.html (uses the same assets/ folder)
"""
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
body = (HERE / 'index.html').read_text(encoding='utf-8')
head = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
        '<style>:root{padding-top:env(safe-area-inset-top,0px);padding-bottom:env(safe-area-inset-bottom,0px)}'
        'body{margin:0}img{max-width:100%}[hidden]{display:none!important}</style>\n</head>\n<body>\n')
(HERE / 'xgo-duck-story.html').write_text(head + body + '\n</body>\n</html>\n', encoding='utf-8')
print('wrote', HERE / 'xgo-duck-story.html')
