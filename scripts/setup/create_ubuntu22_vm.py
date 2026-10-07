#!/usr/bin/env python3
"""Create a persistent VMware Jammy VM from a verified Canonical cloud disk.

Run on Windows with Python 3.10 and pycdlib; the source disk is never modified.
The guest starts with key-only SSH. Desktop/project installation follows over SSH.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import secrets
import subprocess
import sys

IMAGE_URL = ('https://cloud-images.ubuntu.com/releases/jammy/release-20260807/'
             'ubuntu-22.04-server-cloudimg-amd64.vmdk')
IMAGE_SHA256 = 'd4fc0c1160aa158f43bf3f8525a383938a6cefea75156b04350ab5a0967b35fc'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--public-key', type=Path, required=True)
    parser.add_argument('--address', default='192.168.26.22')
    parser.add_argument('--gateway', default='192.168.26.2')
    parser.add_argument('--memory', type=int, default=8192)
    parser.add_argument('--cpus', type=int, default=8)
    parser.add_argument('--disk-gb', type=int, default=120)
    parser.add_argument('--connect-project-usb', action='store_true',
                        help='Auto-connect the CH343 arm adapter and 05a3:9230 external camera')
    parser.add_argument('--vmware', type=Path,
                        default=Path('C:/Program Files/VMware/VMware Workstation'))
    args = parser.parse_args()
    target = args.directory.resolve()
    target.mkdir(parents=True, exist_ok=True)
    if (target/'qianli-humble.vmx').exists():
        parser.error('VM already exists; use its start shortcut rather than recreating it')
    source = target/'ubuntu-22.04-server-cloudimg-amd64.vmdk'
    if not source.is_file():
        parser.error(f'Download {IMAGE_URL} to {source} first')
    digest = hashlib.sha256()
    with source.open('rb') as handle:
        for chunk in iter(lambda: handle.read(8*1024*1024), b''):
            digest.update(chunk)
    if digest.hexdigest() != IMAGE_SHA256:
        parser.error('Canonical disk checksum mismatch; no guest disk was created')
    key = args.public_key.read_text(encoding='utf-8').strip()
    if not key.startswith(('ssh-ed25519 ', 'ssh-rsa ', 'ecdsa-sha2-')) or '\n' in key:
        parser.error('Expected one OpenSSH public key')
    bootstrap = target/'bootstrap'
    bootstrap.mkdir(exist_ok=True)
    sys.path.insert(0, str(bootstrap/'pythonlib'))
    import pycdlib
    import yaml
    password = secrets.token_urlsafe(18)
    config = dict(hostname='qianli-humble', manage_etc_hosts=True, timezone='Asia/Shanghai',
                  users=[dict(name='ros', gecos='QianLi', shell='/bin/bash',
                              groups='sudo,dialout,video,render',
                              sudo='ALL=(ALL) NOPASSWD:ALL', lock_passwd=False,
                              plain_text_passwd=password, ssh_authorized_keys=[key])],
                  ssh_pwauth=False, disable_root=True, growpart=dict(mode='auto',devices=['/']),
                  resize_rootfs=True, package_update=True,
                  packages=['openssh-server','open-vm-tools','python3-venv','git','curl'],
                  runcmd=[['systemctl','enable','--now','ssh'],
                          ['touch','/var/lib/qianli-bootstrap-ready']])
    network = dict(version=2, ethernets=dict(primary=dict(
        match=dict(name='en*'), dhcp4=False, addresses=[args.address+'/24'],
        routes=[dict(to='default', via=args.gateway)],
        nameservers=dict(addresses=[args.gateway,'1.1.1.1']))))
    documents = {'user-data':'#cloud-config\n'+yaml.safe_dump(config,sort_keys=False),
                 'meta-data':yaml.safe_dump({'instance-id':'qianli-humble-'+secrets.token_hex(6),
                                             'local-hostname':'qianli-humble'}),
                 'network-config':yaml.safe_dump(network,sort_keys=False)}
    iso = pycdlib.PyCdlib()
    iso.new(interchange_level=3, joliet=3, rock_ridge='1.09', vol_ident='CIDATA')
    handles = []
    for index,(name,document) in enumerate(documents.items()):
        data=document.encode('utf-8')
        (bootstrap/name).write_bytes(data)
        handle=io.BytesIO(data)
        handles.append(handle)
        iso.add_fp(handle,len(data),iso_path=f'/DATA{index};1',
                   rr_name=name,joliet_path='/'+name)
    iso.write(str(target/'seed.iso'))
    iso.close()
    (bootstrap/'credentials.txt').write_text(
        f'Local desktop user: ros\nLocal desktop password: {password}\n'
        f'SSH: key-only, ros@{args.address}\n',encoding='utf-8')
    # Restrict initial credentials and cloud-init material to this Windows user.
    subprocess.run(['icacls',str(bootstrap),'/inheritance:r','/grant:r',
                    f'{subprocess.check_output(["whoami"],text=True).strip()}:(OI)(CI)F',
                    'SYSTEM:(OI)(CI)F'],check=True,stdout=subprocess.DEVNULL)
    disk=target/'qianli-humble.vmdk'
    if disk.exists():
        parser.error('Guest disk already exists; refusing to overwrite it')
    manager=args.vmware/'vmware-vdiskmanager.exe'
    subprocess.run([str(manager),'-r',str(source),'-t','0',str(disk)],check=True)
    subprocess.run([str(manager),'-x',f'{args.disk_gb}GB',str(disk)],check=True)
    settings={'.encoding':'UTF-8','config.version':'8','virtualHW.version':'21',
              'displayName':'QianLi Ubuntu 22.04 ROS 2 Humble','guestOS':'ubuntu-64',
              'memsize':str(args.memory),'numvcpus':str(args.cpus),'cpuid.coresPerSocket':'4',
              'firmware':'efi','uefi.secureBoot.enabled':'FALSE',
              'pciBridge0.present':'TRUE','pciBridge4.present':'TRUE',
              'pciBridge4.virtualDev':'pcieRootPort','pciBridge4.functions':'8',
              'scsi0.present':'TRUE','scsi0.virtualDev':'lsilogic',
              'scsi0:0.present':'TRUE','scsi0:0.fileName':disk.name,
              'sata0.present':'TRUE','sata0:0.present':'TRUE',
              'sata0:0.deviceType':'cdrom-image','sata0:0.fileName':'seed.iso',
              'sata0:0.startConnected':'TRUE','ethernet0.present':'TRUE',
              'ethernet0.connectionType':'nat','ethernet0.virtualDev':'e1000e',
              'ethernet0.addressType':'generated','ethernet0.startConnected':'TRUE',
              'usb.present':'TRUE','ehci.present':'FALSE','usb_xhci.present':'TRUE',
              'sound.present':'FALSE','floppy0.present':'FALSE','vmci0.present':'TRUE',
              'mks.enable3d':'TRUE','svga.autodetect':'TRUE','svga.vramSize':'268435456',
              'tools.syncTime':'TRUE','uuid.action':'create',
              'serial0.present':'TRUE','serial0.fileType':'file',
              'serial0.fileName':(bootstrap/'serial.log').as_posix(),'serial0.startConnected':'TRUE'}
    if args.connect_project_usb:
        settings['usb.autoConnect.device0']='0x1a86:0x55d3'
        settings['usb.autoConnect.device1']='0x05a3:0x9230'
    (target/'qianli-humble.vmx').write_text(
        ''.join(f'{key} = {json.dumps(value,ensure_ascii=False)}\n' for key,value in settings.items()),
        encoding='utf-8',newline='\n')
    (bootstrap/'image-provenance.json').write_text(json.dumps(
        dict(url=IMAGE_URL,sha256=IMAGE_SHA256,address=args.address,disk_gb=args.disk_gb,
             memory_mb=args.memory,cpus=args.cpus),indent=2),encoding='utf-8')
    print(f'[OK] Persistent VM created: {target / "qianli-humble.vmx"}')
    print(f'[OK] Key-only SSH address: ros@{args.address}')


if __name__ == '__main__':
    main()
