# Run ONCE in PowerShell **as Administrator**:
#   powershell -ExecutionPolicy Bypass -File windows_network_setup.ps1
# Optional: -Adapter "Ethernet 2"   (the wired port the G1 cable is in)
#
# 1. Gives the wired adapter a static IP 192.168.123.222/24 (robot subnet).
# 2. Turns on WSL mirrored networking so WSL sees that adapter directly
#    (needed for DDS multicast to/from the robot).
# 3. Allows inbound traffic into WSL through the Hyper-V firewall.
param([string]$Adapter = "", [string]$Ip = "192.168.123.222")
$ErrorActionPreference = "Stop"

if (-not ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Host "Please run this in an Administrator PowerShell." -ForegroundColor Red; exit 1
}

Write-Host "`nWired adapters:" -ForegroundColor Cyan
$wired = Get-NetAdapter -Physical | Where-Object { $_.MediaType -eq "802.3" }
$wired | Format-Table Name, InterfaceDescription, Status, LinkSpeed -AutoSize
if (-not $Adapter) {
    $up = @($wired | Where-Object Status -eq "Up")
    if ($up.Count -ne 1) {
        Write-Host "Could not pick one automatically. Re-run with -Adapter ""<Name>"" (the one the robot cable is in)." -ForegroundColor Yellow
        exit 1
    }
    $Adapter = $up[0].Name
}
$ans = Read-Host "Set $Adapter to static IP $Ip/24 (no gateway, so your Wi-Fi internet is untouched)? [y/N]"
if ($ans -ne "y") { exit 0 }

Set-NetIPInterface -InterfaceAlias $Adapter -Dhcp Disabled
Get-NetIPAddress -InterfaceAlias $Adapter -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
New-NetIPAddress -InterfaceAlias $Adapter -IPAddress $Ip -PrefixLength 24 | Out-Null
Write-Host "IP set." -ForegroundColor Green

# --- .wslconfig: networkingMode=mirrored ---------------------------------------
$cfg = Join-Path $env:USERPROFILE ".wslconfig"
if (Test-Path $cfg) {
    Copy-Item $cfg "$cfg.bak" -Force
    $txt = Get-Content $cfg -Raw
    if ($txt -match "(?m)^\s*networkingMode\s*=") {
        $txt = $txt -replace "(?m)^\s*networkingMode\s*=.*$", "networkingMode=mirrored"
    } elseif ($txt -match "(?m)^\[wsl2\]") {
        $txt = $txt -replace "(?m)^\[wsl2\]", "[wsl2]`r`nnetworkingMode=mirrored"
    } else {
        $txt += "`r`n[wsl2]`r`nnetworkingMode=mirrored`r`n"
    }
    Set-Content $cfg $txt
    Write-Host ".wslconfig updated (backup: .wslconfig.bak)" -ForegroundColor Green
} else {
    Set-Content $cfg "[wsl2]`r`nnetworkingMode=mirrored`r`n"
    Write-Host ".wslconfig created" -ForegroundColor Green
}

# --- Hyper-V firewall: let the robot's DDS packets into WSL ---------------------
try {
    Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
    Write-Host "Hyper-V firewall: inbound allowed for WSL" -ForegroundColor Green
} catch { Write-Host "Hyper-V firewall setting skipped: $_" -ForegroundColor Yellow }

wsl --shutdown
Write-Host "`nDone. WSL restarted. Open Ubuntu and run:  ping -c 2 192.168.123.164" -ForegroundColor Cyan
