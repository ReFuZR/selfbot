# install.ps1 - one-shot setup for ac-autodriver on Windows.
# Run it through install.bat (double-click). Safe to re-run: every step checks first.
#
#   1. Python 3.11+        (installs Python 3.12 with winget, or from python.org)
#   2. ViGEmBus driver     (virtual Xbox 360 pad; winget, or the official release)
#   3. .venv + packages    (numpy, opencv-python, mss, vgamepad, pynput)
#   4. self-tests          (every module checks itself; no AC needed)

param([switch]$SkipTests)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'     # makes Invoke-WebRequest much faster on PS 5.1
Set-Location -LiteralPath $PSScriptRoot
# Windows PowerShell 5.1 may default to TLS 1.0, which GitHub and python.org refuse
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

$PythonVersion = '3.12.10'
$ViGEmUrl = 'https://github.com/nefarius/ViGEmBus/releases/download/v1.22.0/ViGEmBus_1.22.0_x64_x86_arm64.exe'

function Step($msg) { Write-Host ''; Write-Host "==> $msg" -ForegroundColor Cyan }
function Ok($msg)   { Write-Host "    OK  $msg" -ForegroundColor Green }
function Warn($msg) { Write-Host "    !!  $msg" -ForegroundColor Yellow }
function Die($msg)  { Write-Host "    XX  $msg" -ForegroundColor Red; exit 1 }

function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}

function Has-Winget { [bool](Get-Command winget -ErrorAction SilentlyContinue) }

# Returns the command (as an array: exe + args) of a usable Python 3.11+, or $null.
function Find-Python {
    $candidates = @(
        @('py', '-3.12'), @('py', '-3.13'), @('py', '-3.11'), @('py', '-3'),
        @('python'),
        @("$env:LOCALAPPDATA\Programs\Python\Python312\python.exe")
    )
    foreach ($cand in $candidates) {
        $exe = $cand[0]
        $extra = @($cand | Select-Object -Skip 1)
        if (-not (Get-Command $exe -ErrorAction SilentlyContinue)) { continue }
        try {
            $ver = & $exe @extra -c "import sys; print('%d.%d' % sys.version_info[:2])" 2>$null
        } catch { continue }
        # the Microsoft Store 'python' stub prints nothing and fails - skip it
        if ($LASTEXITCODE -ne 0 -or -not $ver) { continue }
        $parts = "$ver".Trim().Split('.')
        if ([int]$parts[0] -eq 3 -and [int]$parts[1] -ge 11) { return ,(@($exe) + $extra) }
    }
    return $null
}

function Test-ViGEm {
    if (Get-Service -Name 'ViGEmBus' -ErrorAction SilentlyContinue) { return $true }
    $keys = @('HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
              'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*')
    $hit = Get-ItemProperty $keys -ErrorAction SilentlyContinue |
           Where-Object { $_.DisplayName -like '*Virtual Gamepad Emulation Bus*' }
    return [bool]$hit
}

Write-Host 'ac-autodriver installer' -ForegroundColor White
Write-Host "folder: $PSScriptRoot"

# --------------------------------------------------------------------------- 1
Step 'Python 3.11+'
$py = Find-Python
if ($py) {
    Ok ("found: " + ($py -join ' '))
} else {
    Warn "not found - installing Python $PythonVersion"
    if (Has-Winget) {
        winget install -e --id Python.Python.3.12 --scope user --silent `
            --accept-package-agreements --accept-source-agreements
    } else {
        $setup = Join-Path $env:TEMP "python-$PythonVersion-amd64.exe"
        Invoke-WebRequest -UseBasicParsing "https://www.python.org/ftp/python/$PythonVersion/python-$PythonVersion-amd64.exe" -OutFile $setup
        Start-Process $setup -Wait -ArgumentList '/quiet', 'InstallAllUsers=0', 'PrependPath=1', 'Include_launcher=1'
    }
    Refresh-Path
    $py = Find-Python
    if (-not $py) {
        Die "Python install didn't take. Install Python 3.12 from python.org (tick 'Add to PATH'), then re-run install.bat."
    }
    Ok ("installed: " + ($py -join ' '))
}

# --------------------------------------------------------------------------- 2
Step 'ViGEmBus virtual gamepad driver'
if (Test-ViGEm) {
    Ok 'already installed'
} else {
    Warn 'not installed - a Windows admin prompt will appear, click Yes'
    $done = $false
    if (Has-Winget) {
        try {
            winget install -e --id ViGEm.ViGEmBus --silent --accept-package-agreements --accept-source-agreements
            $done = Test-ViGEm
        } catch { }
    }
    if (-not $done) {
        try {
            $setup = Join-Path $env:TEMP 'ViGEmBus_setup.exe'
            Invoke-WebRequest -UseBasicParsing $ViGEmUrl -OutFile $setup
            Write-Host '    running the ViGEmBus installer - follow its window'
            Start-Process $setup -Wait -Verb RunAs
            $done = Test-ViGEm
        } catch { Warn "download/install failed: $($_.Exception.Message)" }
    }
    if ($done) { Ok 'installed' }
    else { Warn "still missing - installing the packages next will offer vgamepad's own driver installer; accept it" }
}

# --------------------------------------------------------------------------- 3
Step 'Virtual environment + packages'
$venvPy = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path $venvPy)) {
    $exe = $py[0]
    $extra = @($py | Select-Object -Skip 1)
    & $exe @extra -m venv .venv
    if ($LASTEXITCODE -ne 0) { Die 'could not create .venv' }
    Ok 'created .venv'
} else {
    Ok '.venv exists'
}
& $venvPy -m pip install --upgrade pip --quiet --disable-pip-version-check
& $venvPy -m pip install -r requirements.txt --disable-pip-version-check
if ($LASTEXITCODE -ne 0) { Die 'pip install failed - see the messages above' }
Ok 'packages installed'
if (-not (Test-ViGEm)) { Warn 'ViGEmBus still not detected: get it from https://github.com/nefarius/ViGEmBus/releases' }

# --------------------------------------------------------------------------- 4
if ($SkipTests) { Step 'Self-tests skipped' } else {
    Step 'Self-tests (no Assetto Corsa needed)'
    # PS 5.1: with 'Stop', any stderr line from a redirected native command is fatal
    $ErrorActionPreference = 'Continue'
    $failed = @()
    foreach ($mod in 'telemetry', 'controller', 'vision', 'planner', 'sim') {
        $out = & $venvPy "$mod.py" --selftest 2>&1 | Select-Object -Last 1
        if ($LASTEXITCODE -eq 0) { Ok "$mod" } else { Warn "$mod : $out"; $failed += $mod }
    }
    if ($failed.Count) { Warn ("failed: " + ($failed -join ', ') + ' - send me the output above') }
}

Step 'Done'
Write-Host '    Next: double-click run.bat. Start with option 1 (telemetry) while sitting in an'
Write-Host '    AC practice session, then option 2 to bind the virtual pad in Content Manager.'
exit 0
