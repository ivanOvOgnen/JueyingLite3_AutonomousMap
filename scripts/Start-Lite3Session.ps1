<#
.SYNOPSIS
  Discover the current robot Wi-Fi route, open an SSH tunnel to the robot API,
  and verify the read-only status endpoint.
.DESCRIPTION
  Run on the Windows laptop after the robot and Retroid are on the same robot
  Wi-Fi. The SSH window is visible so first-use host-key/password prompts are
  handled by OpenSSH. No credentials are saved. No robot controls are issued.
#>
[CmdletBinding()]
param(
    [string]$PerceptionHost = "",
    [ValidateRange(1024, 65535)][int]$LocalPort = 18080,
    [switch]$NoBrowser
)

$ErrorActionPreference = "Stop"

function Test-IPv4([string]$Value) {
    $parsed = $null
    return [System.Net.IPAddress]::TryParse($Value, [ref]$parsed) -and
        $parsed.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork
}

function ConvertTo-PsStringLiteral([string]$Value) {
    return "'" + $Value.Replace("'", "''") + "'"
}

Write-Host "Lite3 safe connection startup (read-only status checks only)" -ForegroundColor Cyan
Write-Host "Before continuing: robot stationary, Retroid on robot Wi-Fi, vendor app in manual mode."
Write-Host "This script will not start mapping/navigation, publish motion, change firewall rules, or stop vendor services."

$sshCommand = Get-Command ssh.exe -ErrorAction SilentlyContinue
if (-not $sshCommand) { throw "Windows OpenSSH client (ssh.exe) was not found. Install/enable the Windows OpenSSH Client first." }

$wifiConfigs = @(Get-NetIPConfiguration | Where-Object {
    $_.NetAdapter.Status -eq "Up" -and
    $_.NetAdapter.InterfaceDescription -match "(?i)wi-?fi|wireless|802\.11" -and
    $_.IPv4Address -and $_.IPv4DefaultGateway
})
if (-not $wifiConfigs) {
    throw "No active Wi-Fi adapter with an IPv4 address and default gateway was found. Connect the laptop to the robot Wi-Fi and retry."
}

if ($wifiConfigs.Count -gt 1) {
    $namedRobotWifi = @()
    foreach ($candidate in $wifiConfigs) {
        try {
            $candidateProfile = Get-NetConnectionProfile -InterfaceIndex $candidate.InterfaceIndex -ErrorAction Stop | Select-Object -First 1
            if ($candidateProfile -and $candidateProfile.Name -match '(?i)YSC|JYML|LITE.?3') { $namedRobotWifi += $candidate }
        } catch { }
    }
    if ($namedRobotWifi.Count -eq 1) {
        $wifi = $namedRobotWifi[0]
    } else {
        $sshWifiCandidates = @()
        foreach ($candidate in $wifiConfigs) {
            $candidateGateway = $candidate.IPv4DefaultGateway[0].NextHop
            if (Test-IPv4 $candidateGateway -and (Test-NetConnection -ComputerName $candidateGateway -Port 22 -InformationLevel Quiet -WarningAction SilentlyContinue)) {
                $sshWifiCandidates += $candidate
            }
        }
        if ($sshWifiCandidates.Count -ne 1) {
            throw "Multiple Wi-Fi routes are active and the robot gateway could not be identified uniquely. Disconnect other Wi-Fi networks, leave the laptop on the robot Wi-Fi, and rerun; no IP selection is needed."
        }
        $wifi = $sshWifiCandidates[0]
    }
} else {
    $wifi = $wifiConfigs[0]
}

$laptopIPv4 = $wifi.IPv4Address[0].IPAddress
$gateway = $wifi.IPv4DefaultGateway[0].NextHop
if (-not (Test-IPv4 $laptopIPv4) -or -not (Test-IPv4 $gateway)) { throw "The selected Wi-Fi interface did not provide valid IPv4 addressing." }

try {
    $profile = Get-NetConnectionProfile -InterfaceIndex $wifi.InterfaceIndex -ErrorAction Stop | Select-Object -First 1
    $ssid = if ($profile) { $profile.Name } else { $null }
    if (-not $ssid) {
        $ssidOutput = netsh.exe wlan show interfaces 2>$null
        $ssidLine = $ssidOutput | Where-Object { $_ -match '^\s*SSID\s*:' -and $_ -notmatch 'BSSID' } | Select-Object -First 1
        $ssid = if ($ssidLine) { ($ssidLine -split ':', 2)[1].Trim() } else { "(SSID unavailable)" }
    }
} catch { $ssid = $null }
if (-not $ssid) { $ssid = "(unavailable; the gateway SSH/config check will identify the robot network)" }

Write-Host ""
Write-Host ("Wi-Fi adapter: {0}" -f $wifi.InterfaceAlias)
Write-Host ("SSID:          {0}" -f $ssid)
Write-Host "Checking the current gateway and reading the perception address automatically."

$perceptionSshLocalPort = $LocalPort + 1
if ($perceptionSshLocalPort -gt 65535) {
    throw "-LocalPort must be 65534 or lower so a separate loopback SSH-forward port can be reserved."
}
foreach ($requiredPort in @($LocalPort, $perceptionSshLocalPort)) {
    if (Get-NetTCPConnection -State Listen -LocalPort $requiredPort -ErrorAction SilentlyContinue) {
        throw "Local port $requiredPort already has a listener. Close the previous Lite3 tunnel or choose another -LocalPort."
    }
}

if ([string]::IsNullOrWhiteSpace($PerceptionHost)) {
    Write-Host "Reading the perception address from YSC's current network.toml. SSH may ask for the YSC password." -ForegroundColor Cyan
    $networkConfigQuery = "grep -nE '^[[:space:]]*ip[[:space:]]*=' ~/jy_exe/conf/network.toml"
    $networkConfigLines = @(& $sshCommand.Source -o ConnectTimeout=10 "ysc@$gateway" $networkConfigQuery)
    $discoveryExitCode = $LASTEXITCODE
    if ($discoveryExitCode -ne 0) {
        throw "Could not read ~/jy_exe/conf/network.toml on YSC (ssh/grep exit $discoveryExitCode). Confirm the YSC login and that the laptop is on robot Wi-Fi; no perception address was guessed."
    }
    $configuredAddresses = @()
    foreach ($line in $networkConfigLines) {
        if ($line -match '^\s*(?:\d+:)?\s*ip\s*=\s*["'']?((?:\d{1,3}\.){3}\d{1,3})["'']?\s*$') {
            if (Test-IPv4 $Matches[1]) { $configuredAddresses += $Matches[1] }
        }
    }
    $configuredAddresses = @($configuredAddresses | Sort-Object -Unique)
    if ($configuredAddresses.Count -ne 1) {
        throw "YSC's network.toml did not contain one unambiguous IPv4 value on an 'ip =' line. No address was guessed. Check the config format before retrying."
    }
    $PerceptionHost = $configuredAddresses[0]
    Write-Host "Perception address discovered from YSC configuration." -ForegroundColor Green
}
if ($PerceptionHost -notmatch '^[A-Za-z0-9][A-Za-z0-9.-]*$') { throw "PerceptionHost must be a hostname or IPv4 address." }
$tunnelBind = "${laptopIPv4}:${LocalPort}:${PerceptionHost}:8080"
$perceptionSshTunnelBind = "127.0.0.1:${perceptionSshLocalPort}:${PerceptionHost}:22"
$sshArgs = @(
    "-v",
    "-o", "ExitOnForwardFailure=yes",
    "-o", "ServerAliveInterval=15",
    "-o", "ServerAliveCountMax=3",
    "-L", $tunnelBind,
    "-L", $perceptionSshTunnelBind,
    "-N",
    "ysc@$gateway"
)

Write-Host ""
Write-Host "Starting SSH tunnels through YSC to the perception API and its SSH port in a separate visible window." -ForegroundColor Cyan
Write-Host "HTTP status uses YSC's current internal route. The perception SSH forward is loopback-only and is used only if the UI needs deployment. Accept host keys only if they match a robot you recognize; enter passwords at the prompts."
Write-Host "No password or key is written by this script. Close that SSH window or press Ctrl+C there to disconnect."
$helperPath = Join-Path $env:TEMP "lite3_ssh_tunnel_$PID.ps1"
$exitPath = Join-Path $env:TEMP "lite3_ssh_tunnel_$PID.exit"
Remove-Item -LiteralPath $exitPath -Force -ErrorAction SilentlyContinue
$helperLines = @(
    '$ErrorActionPreference = "Continue"',
    ('$sshExe = ' + (ConvertTo-PsStringLiteral $sshCommand.Source)),
    '$sshArgs = @('
) + @($sshArgs | ForEach-Object { '    ' + (ConvertTo-PsStringLiteral $_) }) + @(
    ')',
    '& $sshExe @sshArgs',
    '$tunnelExitCode = $LASTEXITCODE',
    ('Set-Content -LiteralPath ' + (ConvertTo-PsStringLiteral $exitPath) + ' -Value $tunnelExitCode'),
    'Write-Host "SSH API tunnel ended with exit code $tunnelExitCode. Read the OpenSSH output above for the cause."'
)
Set-Content -LiteralPath $helperPath -Value $helperLines -Encoding utf8
$powershellExe = Join-Path $env:WINDIR "System32\WindowsPowerShell\v1.0\powershell.exe"
$helperArguments = @("-NoProfile", "-NoExit", "-ExecutionPolicy", "Bypass", "-File", ('"' + $helperPath + '"'))
$tunnelWindow = Start-Process -FilePath $powershellExe -ArgumentList $helperArguments -PassThru -WindowStyle Normal

$url = "http://${laptopIPv4}:${LocalPort}/"
$statusUrl = "http://${laptopIPv4}:${LocalPort}/api/v1/status"
$deadline = (Get-Date).AddSeconds(100)
$verified = $false
Write-Host "Waiting up to 100 seconds for the robot API status endpoint..."
while ((Get-Date) -lt $deadline) {
    if (Test-Path -LiteralPath $exitPath) {
        $tunnelExitCode = (Get-Content -LiteralPath $exitPath -Raw).Trim()
        throw "The SSH API tunnel exited with code $tunnelExitCode. Its window remains open with the exact OpenSSH error; resolve that network, host, or authentication error and retry."
    }
    if ($tunnelWindow.HasExited) { throw "The SSH tunnel window was closed before the API status check completed." }
    try {
        $status = Invoke-RestMethod -Uri $statusUrl -Method Get -TimeoutSec 3 -Headers @{ "Cache-Control" = "no-cache" }
        if ($status.ok -eq $true -and $status.components) { $verified = $true; break }
    } catch { }
    Start-Sleep -Seconds 2
}

if (-not $verified) {
    Write-Host "The status endpoint did not answer. Keep the SSH window open to inspect errors." -ForegroundColor Yellow
    Write-Host "The current address was read from YSC network.toml. Check the visible SSH window for host-key, authentication, or API startup errors."
    Write-Host "Do not launch another API process until its current state has been checked."
    exit 2
}

$servedAppUrl = "http://${laptopIPv4}:${LocalPort}/app.js"
$servedApp = $null
try { $servedApp = (Invoke-WebRequest -Uri $servedAppUrl -UseBasicParsing -TimeoutSec 5).Content } catch { }
if ($servedApp -notmatch 'UI_DEPLOY_VERSION\s*=\s*24') {
    if (-not $status.components.global_localization -or -not $status.components.safety_stop -or
        -not $status.components.localization -or
        -not $status.components.transforms -or -not $status.components.costmaps -or
        $status.components.localization.PSObject.Properties.Name -notcontains 'converged' -or
        $status.components.global_localization.PSObject.Properties.Name -notcontains 'refresh_ready' -or
        $status.components.navigation.PSObject.Properties.Name -notcontains 'goal_requests_opt_in_available' -or
        $status.components.navigation.PSObject.Properties.Name -notcontains 'active_map_name') {
        throw "The robot API lacks the localization, transform, costmap, or planner opt-in fields required by local UI version 24. No UI files were changed. Deploy/build the matching API before installing the UI."
    }
    $localIndex = Join-Path $PSScriptRoot "..\webapp\index.html"
    $localApp = Join-Path $PSScriptRoot "..\webapp\app.js"
    $localStyles = Join-Path $PSScriptRoot "..\webapp\styles.css"
    if (-not (Test-Path $localIndex) -or -not (Test-Path $localApp) -or -not (Test-Path $localStyles)) {
        throw "The robot UI is older than this project copy, and local HTML, JS, or CSS files were not found beside this script."
    }
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $backupDir = "~/lite3_autonomy_ws/backups/$(Get-Date -Format yyyyMMdd)/startup_ui_predeploy_$stamp"
    $backupCommand = "mkdir -p $backupDir && cp -p ~/lite3_autonomy_ws/webapp/index.html ~/lite3_autonomy_ws/webapp/app.js ~/lite3_autonomy_ws/webapp/styles.css $backupDir/"
    Write-Host "The robot's served UI is older than this project copy. Backing up and installing only index.html, app.js, and styles.css (no API restart)." -ForegroundColor Yellow
    Write-Host "The perception SSH connection is forwarded through YSC; its first use may ask you to confirm the perception host key and enter its password."
    & $sshCommand.Source -p $perceptionSshLocalPort "ysc@127.0.0.1" $backupCommand
    if ($LASTEXITCODE -ne 0) { throw "Could not create the dated robot UI backup; no UI files were copied." }
    $scpCommand = Get-Command scp.exe -ErrorAction SilentlyContinue
    if (-not $scpCommand) { throw "scp.exe was not found. The robot UI backup exists; enable the Windows OpenSSH Client and retry." }
    & $scpCommand.Source -P $perceptionSshLocalPort $localIndex $localApp $localStyles "ysc@127.0.0.1:/home/ysc/lite3_autonomy_ws/webapp/"
    if ($LASTEXITCODE -ne 0) { throw "UI file copy failed. The dated pre-deploy backup is at $backupDir on the perception computer." }
    try { $servedApp = (Invoke-WebRequest -Uri $servedAppUrl -UseBasicParsing -TimeoutSec 5).Content } catch { $servedApp = $null }
    if ($servedApp -notmatch 'UI_DEPLOY_VERSION\s*=\s*24') { throw "The UI files were copied, but the served app did not report UI deployment version 24. Backup: $backupDir. Do not restart robot services as a workaround." }
    Write-Host "Updated UI assets are being served. Backup: $backupDir" -ForegroundColor Green
}

Write-Host ""
Write-Host "Read-only API status verified." -ForegroundColor Green
Write-Host ("Laptop browser: {0}" -f $url)
Write-Host ("Retroid browser: {0} (same robot Wi-Fi; Windows Firewall may block laptop-hosted forwarding)" -f $url)
Write-Host "Open the page and press ‘Connect & check status’. The button performs session/status GETs only."
Write-Host ("SSH API tunnel process: {0}; close its separate window or press Ctrl+C there when finished." -f $tunnelWindow.Id)
if (-not $NoBrowser) {
    Start-Process $url
    Write-Host "The browser opened. Press 'Connect & check status' to create its session and refresh component states."
}
