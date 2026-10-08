#!/usr/bin/env python3
"""Fetch pinned PX4 v1.15.4 SITL submodule archives from GitHub.

Run explicitly when preparing this simulation; archives are extracted only
inside the PX4 checkout. This avoids an unrelated recursive NuttX download.
"""
import configparser
import io
import json
from pathlib import Path
import re
import subprocess
import tarfile
from urllib.request import urlopen


PACKAGE = Path(__file__).resolve().parents[1]
PX4 = PACKAGE.parents[1] / 'external' / 'PX4-Autopilot'
REQUIRED = (
    'src/lib/heatshrink/heatshrink',
    'src/lib/events/libevents',
    'src/lib/crypto/monocypher',
    'src/lib/crypto/libtomcrypt',
    'src/lib/crypto/libtommath',
    'src/modules/mavlink/mavlink',
)


def submodule_urls():
    parser = configparser.ConfigParser()
    parser.read(PX4 / '.gitmodules')
    return {entry['path']: entry['url'] for entry in parser.values()
            if 'path' in entry and 'url' in entry}


def gitlink(path):
    result = subprocess.check_output(['git', '-C', str(PX4), 'ls-tree', 'HEAD', path], text=True)
    match = re.match(r'160000 commit ([0-9a-f]{40})\t', result)
    if not match:
        raise RuntimeError('Missing PX4 gitlink for ' + path)
    return match.group(1)


def get(url):
    with urlopen(url, timeout=120) as response:
        return response.read()


def unpack(archive, dest):
    dest.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(archive), mode='r:gz') as tar:
        for member in tar:
            bits = Path(member.name).parts[1:]
            if not bits or '..' in bits:
                continue
            target = dest.joinpath(*bits)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(member) as source, target.open('wb') as output:
                    output.write(source.read())


def fetch(url, sha, dest):
    if dest.exists() and any(dest.iterdir()):
        print('Already populated:', dest)
        return
    match = re.fullmatch(r'https://github.com/([^/]+)/([^/]+?)(?:\.git)?', url)
    if not match:
        raise RuntimeError('Unsupported source URL: ' + url)
    archive_url = 'https://codeload.github.com/{}/{}/tar.gz/{}'.format(
        match.group(1), match.group(2), sha)
    print('Downloading:', archive_url, flush=True)
    unpack(get(archive_url), dest)


def main():
    urls = submodule_urls()
    for path in REQUIRED:
        fetch(urls[path], gitlink(path), PX4 / path)
    mavlink = PX4 / 'src/modules/mavlink/mavlink'
    pymavlink = mavlink / 'pymavlink'
    if not pymavlink.exists() or not any(pymavlink.iterdir()):
        tree = json.loads(get('https://api.github.com/repos/mavlink/mavlink/git/trees/'
                              + gitlink('src/modules/mavlink/mavlink')))
        sha = next(item['sha'] for item in tree['tree'] if item['path'] == 'pymavlink')
        fetch('https://github.com/ardupilot/pymavlink.git', sha, pymavlink)
    gazebo = PX4 / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic'
    optical = gazebo / 'external/OpticalFlow'
    if not optical.exists() or not any(optical.iterdir()):
        # The Gazebo archive omits gitlink contents. GitHub's tree API exposes
        # the exact OpticalFlow commit recorded by that pinned Gazebo revision.
        root = json.loads(get('https://api.github.com/repos/PX4/PX4-SITL_gazebo-classic/'
                              'git/trees/da7206e057703cc645770f02437013358b71e1c0'))
        external_sha = next(item['sha'] for item in root['tree'] if item['path'] == 'external')
        child = json.loads(get('https://api.github.com/repos/PX4/PX4-SITL_gazebo-classic/'
                               'git/trees/' + external_sha))
        sha = next(item['sha'] for item in child['tree'] if item['path'] == 'OpticalFlow')
        fetch('https://github.com/PX4/OpticalFlow.git', sha, optical)


if __name__ == '__main__':
    main()
