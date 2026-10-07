param([string]$Vmx = 'D:\VMs\QianLi-Ubuntu22-Humble\qianli-humble.vmx')
$ErrorActionPreference = 'Stop'
$vmwareDir = 'C:\Program Files\VMware\VMware Workstation'
$vmxPath = (Resolve-Path -LiteralPath $Vmx).Path
$running = & (Join-Path $vmwareDir 'vmrun.exe') -T ws list
if ($LASTEXITCODE -ne 0) { throw 'Cannot query VMware Workstation' }
if ($running -notcontains $vmxPath) {
    & (Join-Path $vmwareDir 'vmrun.exe') -T ws start $vmxPath gui
    if ($LASTEXITCODE -ne 0) { throw 'Cannot start the Humble virtual machine' }
}
# This is the interactive VM window requested by the user.
Start-Process -FilePath (Join-Path $vmwareDir 'vmware.exe') -ArgumentList @('-t', ('"' + $vmxPath + '"')) -WindowStyle Normal
