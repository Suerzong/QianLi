param([ValidateSet('0.02','0.04')][string]$ObjectSize = '0.04', [switch]$Smoke, [int]$Steps = 600000, [string]$Pretrain = '')
$ErrorActionPreference = 'Stop'
$env:PYTHONUTF8 = '1'
$projectPath = (Resolve-Path "$PSScriptRoot\..\..").Path
$pythonPath = Join-Path $projectPath '.venv-train-win\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $pythonPath)) { throw 'Run scripts/setup/setup_training_windows.ps1 first' }
Push-Location $projectPath
try {
    if ($Smoke) {
        & $pythonPath scripts/tools/train_smoke.py --device cuda --obj-size $ObjectSize --steps 1024
    } else {
        $tag = 'gpu_' + $ObjectSize + '_' + (Get-Date -Format 'yyyyMMdd_HHmmss')
        $trainingArguments = @('dual_twin/scripts/train.py','--device','cuda','--obj-size',$ObjectSize,'--steps',$Steps,'--n-envs','2','--batch-size','256','--tag',$tag)
        if ($Pretrain) { $trainingArguments += @('--pretrain',$Pretrain) }
        & $pythonPath @trainingArguments
    }
    if ($LASTEXITCODE -ne 0) { throw 'CUDA training failed; inspect the terminal output' }
} finally { Pop-Location }
