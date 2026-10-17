#!/usr/bin/env python3
"""Build the MkDocs source tree from a polka checkout, so README.md and doc/ stay the only copy."""
import re
import shutil
import sys
from pathlib import Path

REPO = 'https://github.com/Pana1v/polka'
BRANCH = 'humble'
PAGES = {
    'CONFIGURATION.md': 'configuration.md',
    'PIPELINE.md': 'pipeline.md',
    'DESKEW.md': 'deskew.md',
    'PERFORMANCE.md': 'performance.md',
}


def relink_pages(text):
    for src, dst in PAGES.items():
        text = re.sub(rf'\]\({re.escape(src)}', f']({dst}', text)
    return text


def convert_readme(text):
    text = text.replace('doc/images/', 'images/').replace('doc/media/gifs/', 'gifs/')
    text = text.replace('](doc/media/)', f']({REPO}/tree/{BRANCH}/doc/media)')
    text = text.replace('](../../tree/', f']({REPO}/tree/')
    text = text.replace('](MAINTAINING.md)', f']({REPO}/blob/{BRANCH}/MAINTAINING.md)')
    text = text.replace('href="LICENSE"', f'href="{REPO}/blob/{BRANCH}/LICENSE"')
    text = text.replace('](doc/', '](')
    return relink_pages(text)


def convert_doc(text):
    text = text.replace('](../config/', f']({REPO}/blob/{BRANCH}/config/')
    return relink_pages(text)


def main(src, out):
    src, out = Path(src), Path(out)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    (out / 'index.md').write_text(convert_readme((src / 'README.md').read_text()))
    for name, dst in PAGES.items():
        (out / dst).write_text(convert_doc((src / 'doc' / name).read_text()))
    shutil.copytree(src / 'doc' / 'images', out / 'images')
    shutil.copytree(src / 'doc' / 'media' / 'gifs', out / 'gifs')


if __name__ == '__main__':
    main(*sys.argv[1:3])
