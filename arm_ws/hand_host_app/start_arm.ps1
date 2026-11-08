# start_arm.ps1 -- start CAN bridge in WSL, then the Windows UI
# Usage:
#   .\start_arm.ps1            # real hardware
#   .\start_arm.ps1 -DryRun    # fake data (UI only)
param(
    [switch]$DryRun,
    [int]$Port = 5599,
    [string]$Distro = "Ubuntu-24.04"
)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$wslTools = "/home/raki/ros2_arm_ws/src/arm_urdf/tools"

Write-Host "[1/3] starting CAN bridge in WSL ($Distro, port $Port) ..." -ForegroundColor Cyan
$dry = if ($DryRun) { "--dry-run" } else { "" }
wsl -d $Distro -u root bash -c "pkill -f can_bridge.py 2>/dev/null; sleep 0.5; cd $wslTools && ./arm_up.sh --port $Port $dry" 2>&1 | ForEach-Object { "    $_" }

Write-Host "[2/3] waiting for port $Port ..." -ForegroundColor Cyan
$ok = $false
for ($i = 0; $i -lt 20; $i++) {
    Start-Sleep -Milliseconds 300
    $out = wsl -d $Distro -u root bash -c "ss -ltn | grep -c ':$Port '" 2>$null
    if ($out -match "1") { $ok = $true; break }
}
if (-not $ok) { Write-Host "    bridge not ready, check /tmp/arm_bridge.log in WSL" -ForegroundColor Yellow }
else { Write-Host "    bridge ready on $Port" -ForegroundColor Green }

Write-Host "[3/3] launching UI ..." -ForegroundColor Cyan
Set-Location $here
Start-Process -FilePath "python" -ArgumentList "main.py" -WorkingDirectory $here
Write-Host "done. In the UI: Connection -> Connect -> Enable all motors." -ForegroundColor Green
