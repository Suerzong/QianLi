#!/usr/bin/env python3
"""Fetch official ROS bootstrap files on a host whose GitHub connection works."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def fetch(url, path):
    subprocess.run(['curl.exe' if __import__('os').name=='nt' else 'curl',
                    '-fsSL','--retry','3','--connect-timeout','20','--max-time','180',
                    '-o',str(path),url],check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,required=True)
    args=parser.parse_args()
    root=args.directory.resolve()
    root.mkdir(parents=True,exist_ok=True)
    fetch('https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest',root/'release.json')
    release=json.loads((root/'release.json').read_text(encoding='utf-8'))
    asset=next(a for a in release['assets'] if a['name']==f'ros2-apt-source_{release["tag_name"]}.jammy_all.deb')
    fetch(asset['browser_download_url'],root/'ros2-apt-source.deb')
    digest=hashlib.sha256((root/'ros2-apt-source.deb').read_bytes()).hexdigest()
    assert asset.get('digest')=='sha256:'+digest, 'Publisher package checksum mismatch'
    fetch('https://api.github.com/repos/ros/rosdistro/commits/master',root/'rosdistro-commit.json')
    commit=json.loads((root/'rosdistro-commit.json').read_text(encoding='utf-8'))['sha']
    urls={'ros2-apt-source.deb':asset['browser_download_url']}
    for name in ('base.yaml','python.yaml','ruby.yaml'):
        urls[name]=f'https://raw.githubusercontent.com/ros/rosdistro/{commit}/rosdep/{name}'
    urls['index-v4.yaml']=f'https://raw.githubusercontent.com/ros/rosdistro/{commit}/index-v4.yaml'
    urls['humble-distribution.yaml']=f'https://raw.githubusercontent.com/ros/rosdistro/{commit}/humble/distribution.yaml'
    fetch(urls['index-v4.yaml'],root/'index-v4.yaml')
    import yaml
    cache_url=yaml.safe_load((root/'index-v4.yaml').read_text())['distributions']['humble']['distribution_cache']
    urls['humble-cache.yaml.gz']=cache_url.replace('http://','https://',1)
    for name,url in urls.items():
        if name not in ('ros2-apt-source.deb','index-v4.yaml'): fetch(url,root/name)
    manifest={'format':1,'rosdistro_commit':commit,'apt_source_version':release['tag_name'],
              'files':{name:dict(url=url,size=(root/name).stat().st_size,
                                 sha256=hashlib.sha256((root/name).read_bytes()).hexdigest())
                       for name,url in urls.items()}}
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print('[OK] Verified official bootstrap cache:',root)


if __name__=='__main__': main()
