# Run the full research pipeline in a detached process tree.
#
# Git Bash background jobs die when the terminal closes, which has stopped this
# run twice. Start-Process gives a process tree that is not parented to the
# shell, so the computation survives closing the terminal or ending a session.
#
#   powershell -ExecutionPolicy Bypass -File run_detached.ps1        # 11 workers
#   powershell -ExecutionPolicy Bypass -File run_detached.ps1 -W 6   # 6 workers
#
# Progress:  Get-Content logs\grid.log -Tail 3
# Stop:      Get-Process python | Stop-Process -Force
#
# Note: ASCII only. Windows PowerShell 5.1 reads .ps1 as ANSI unless the file
# has a UTF-8 BOM, so non-ASCII text here breaks the parser.

param([int]$W = 11)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

$bash = "C:\Program Files\Git\bin\bash.exe"
if (-not (Test-Path $bash)) { $bash = "bash" }

$existing = Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
            Where-Object { $_.CommandLine -match 'run_grid|run_sensitivity|run_verification' }
if ($existing) {
    Write-Host "Already running (PID: $($existing.ProcessId -join ', ')). Not starting a second copy."
    exit 1
}

New-Item -ItemType Directory -Force -Path "$root\logs" | Out-Null
$p = Start-Process -FilePath $bash -ArgumentList @("run_all.sh", "$W") `
        -WorkingDirectory $root -WindowStyle Hidden -PassThru
$p.Id | Out-File -FilePath "$root\logs\run_all.pid" -Encoding ascii

Write-Host "Started. parent PID $($p.Id), workers $W"
Write-Host "Progress: Get-Content logs\grid.log -Tail 3"
Write-Host "Stop:     Get-Process python | Stop-Process -Force"
