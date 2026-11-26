# install.ps1 - one-shot setup for ac-autodriver on Windows.
# Run it through install.bat (double-click). Safe to re-run: every step checks first.
#
#   1. Python 3.11+        (installs Python 3.12 with winget, or from python.org)
#   2. vJoy driver         (the virtual wheel the bot drives; your Logitech stays as it is)
#   3. .venv + packages    (numpy, opencv-python, mss, pynput, pygame)
#   4. self-tests          (every module checks itself; no AC needed)
#   5. wheel setup         (learn your Logitech's axes, ~30 s)
#
#   install.bat -Xbox      also installs the virtual Xbox pad (ViGEmBus + vgamepad)
#   install.bat -SkipTests

param([switch]$SkipTests, [switch]$Xbox)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'     # makes Invoke-WebRequest much faster on PS 5.1
Set-Location -LiteralPath $PSScriptRoot
# Windows PowerShell 5.1 may default to TLS 1.0, which GitHub and python.org refuse
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

$PythonVersion = '3.12.10'
$VJoyUrl = 'https://github.com/jshafer817/vJoy/releases/download/v2.1.9.1/vJoySetup.exe'
$VJoyPage = 'https://github.com/jshafer817/vJoy/releases'
$ViGEmUrl = 'https://github.com/nefarius/ViGEmBus/releases/download/v1.22.0/ViGEmBus_1.22.0_x64_x86_arm64.exe'
$UninstallKeys = @('HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
                   'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*')

function Step($msg) { Write-Host ''; Write-Host "==> $msg" -ForegroundColor Cyan }
function Ok($msg)   { Write-Host "    OK  $msg" -ForegroundColor Green }
function Warn($msg) { Write-Host "    !!  $msg" -ForegroundColor Yellow }
function Die($msg)  { Write-Host "    XX  $msg" -ForegroundColor Red; exit 1 }

function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}

function Has-Winget { [bool](Get-Command winget -ErrorAction SilentlyContinue) }

function Installed($pattern) {
    [bool](Get-ItemProperty $UninstallKeys -ErrorAction SilentlyContinue |
           Where-Object { $_.DisplayName -like $pattern })
}

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

function Test-VJoy {
    (Test-Path (Join-Path $env:ProgramFiles 'vJoy\x64\vJoyInterface.dll')) -or (Installed 'vJoy*')
}

function Test-ViGEm {
    if (Get-Service -Name 'ViGEmBus' -ErrorAction SilentlyContinue) { return $true }
    Installed '*Virtual Gamepad Emulation Bus*'
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
Step 'vJoy virtual wheel driver'
if (Test-VJoy) {
    Ok 'already installed'
} else {
    Warn 'not installed - Windows will ask for admin rights and to trust the driver: click Yes / Install'
    try {
        $setup = Join-Path $env:TEMP 'vJoySetup.exe'
        Invoke-WebRequest -UseBasicParsing $VJoyUrl -OutFile $setup
        Start-Process $setup -Wait -Verb RunAs -ArgumentList '/SILENT', '/NORESTART'
    } catch { Warn "automatic install failed: $($_.Exception.Message)" }
    if (Test-VJoy) {
        Ok 'installed'
    } else {
        Warn "Please install vJoy by hand: the page is opening now. Run vJoySetup.exe, then re-run install.bat."
        Start-Process $VJoyPage
    }
}

if ($Xbox) {
    Step 'ViGEmBus virtual Xbox pad driver (-Xbox)'
    if (Test-ViGEm) {
        Ok 'already installed'
    } else {
        Warn 'not installed - a Windows admin prompt will appear, click Yes'
        try {
            if (Has-Winget) {
                winget install -e --id ViGEm.ViGEmBus --silent --accept-package-agreements --accept-source-agreements
            }
            if (-not (Test-ViGEm)) {
                $setup = Join-Path $env:TEMP 'ViGEmBus_setup.exe'
                Invoke-WebRequest -UseBasicParsing $ViGEmUrl -OutFile $setup
                Start-Process $setup -Wait -Verb RunAs
            }
        } catch { Warn "install failed: $($_.Exception.Message)" }
        if (Test-ViGEm) { Ok 'installed' } else { Warn 'still missing: https://github.com/nefarius/ViGEmBus/releases' }
    }
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
if ($Xbox) {
    & $venvPy -m pip install 'vgamepad>=0.1.0' --disable-pip-version-check
    if ($LASTEXITCODE -ne 0) { Warn 'vgamepad install failed' }
}
Ok 'packages installed'

# --------------------------------------------------------------------------- 4
if ($SkipTests) { Step 'Self-tests skipped' } else {
    Step 'Self-tests (no Assetto Corsa needed)'
    # PS 5.1: with 'Stop', any stderr line from a redirected native command is fatal
    $ErrorActionPreference = 'Continue'
    $failed = @()
    foreach ($mod in 'telemetry', 'controller', 'vision', 'planner', 'main', 'diagnose', 'sim') {
        $out = & $venvPy "$mod.py" --selftest 2>&1 | Select-Object -Last 1
        if ($LASTEXITCODE -eq 0) { Ok "$mod" } else { Warn "$mod : $out"; $failed += $mod }
    }
    if ($failed.Count) { Warn ("failed: " + ($failed -join ', ') + ' - send me the output above') }
    $ErrorActionPreference = 'Stop'
}

# --------------------------------------------------------------------------- 5
Step 'Your wheel'
if (Test-Path (Join-Path $PSScriptRoot 'wheel_calibration.json')) {
    Ok 'wheel_calibration.json exists (run.bat -> 2 to redo it)'
} else {
    $ans = Read-Host '    Set up your Logitech wheel now? Plug it in first. [Y/n]'
    if ($ans -notmatch '^[nN]') {
        $ErrorActionPreference = 'Continue'
        & $venvPy controller.py --wheel-setup
        $ErrorActionPreference = 'Stop'
    } else {
        Warn 'skipped - do it later with run.bat -> 2'
    }
}

Step 'Done'
Write-Host '    Next (once): bind the virtual wheel in Content Manager - see README.md,'
Write-Host '    "Content Manager setup". Then double-click run.bat and pick 1 to drive.'
exit 0
