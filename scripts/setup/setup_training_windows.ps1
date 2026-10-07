param([string]$ProjectRoot = (Resolve-Path "$PSScriptRoot\..\.."), [switch]$Verify)
$ErrorActionPreference = 'Stop'
$env:PYTHONUTF8 = '1'
$venvPath = Join-Path $ProjectRoot '.venv-train-win'
$pythonPath = Join-Path $venvPath 'Scripts\python.exe'
if (-not (Test-Path -LiteralPath $pythonPath)) {
    & py -3.10 -m venv $venvPath
    if ($LASTEXITCODE -ne 0) { throw 'Python 3.10 virtual environment creation failed' }
}
& $pythonPath -c 'import sys; assert sys.version_info[:2] == (3, 10)'
if ($LASTEXITCODE -ne 0) { throw 'Training requires Python 3.10' }
& $pythonPath -m pip install pip==25.1.1 setuptools==69.5.1 wheel==0.45.1
if ($LASTEXITCODE -ne 0) { throw 'pip bootstrap failed' }
$wheelPath = Join-Path $ProjectRoot 'migration_assets\torch-2.8.0+cu128-cp310-cp310-win_amd64.whl'
if ((Test-Path -LiteralPath $wheelPath) -and
    (Get-FileHash -LiteralPath $wheelPath -Algorithm SHA256).Hash.ToLowerInvariant() -eq '43938e9a174c90e5eb9e906532b2f1e21532bbfa5a61b65193b4f54714d34f9e') {
    & $pythonPath -m pip install $wheelPath
} else {
    & $pythonPath -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cu128
}
if ($LASTEXITCODE -ne 0) { throw 'PyTorch installation failed' }
& $pythonPath -m pip install -r (Join-Path $ProjectRoot 'scripts\setup\requirements\train.txt')
if ($LASTEXITCODE -ne 0) { throw 'Training dependencies installation failed' }
& $pythonPath -m pip check
if ($LASTEXITCODE -ne 0) { throw 'Training dependency check failed' }
& $pythonPath -m pip freeze | Set-Content -LiteralPath (Join-Path $venvPath 'resolved-requirements.txt') -Encoding UTF8
if ($Verify) {
    Push-Location $ProjectRoot
    try {
        foreach ($size in @('0.02','0.04')) {
            & $pythonPath scripts/tools/train_smoke.py --device cuda --obj-size $size --steps 1024
            if ($LASTEXITCODE -ne 0) { throw "CUDA training smoke failed: $size" }
        }
    } finally { Pop-Location }
}
Write-Output '[OK] Persistent Windows CUDA training environment ready.'
