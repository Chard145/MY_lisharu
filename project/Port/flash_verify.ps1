# ============================================================================
#  flash_verify.ps1  --  build, flash, then PROVE what is actually on the chip
# ============================================================================
#  Why this exists
#  ---------------
#  The old flow was hand-assembled:
#       cmake --build            -> build\Debug\project.elf
#       objcopy -O ihex          -> project.hex        (path typed by hand)
#       JLink loadfile           -> build\Debug\project.hex  (path in flash.jlink)
#  Those two paths were NEVER compared. objcopy and JLink both return 0, so
#  every step reported success while the board kept running a build from hours
#  ago. That silently poisoned later debugging: freshly built symbol addresses
#  were used to interpret memory of an OLD firmware, producing confident but
#  wrong conclusions.
#
#  This script removes the possibility of that class of error:
#     1. refuses to continue if project.hex is older than project.elf
#     2. flashes build\Debug\project.hex  (the only image it ever uses)
#     3. READS THE FLASH BACK and compares it byte-for-byte with project.bin
#     4. prints PASS / FAIL and appends everything to Port\flash_log.txt
#
#  If step 3 passes, the board provably runs exactly this build.
#
#  ASCII only on purpose: Windows PowerShell 5.1 reads non-BOM .ps1 as GBK,
#  so non-ASCII comments break the parser.
#
#  Usage (from anywhere):
#      powershell -ExecutionPolicy Bypass -File <this file>
#      powershell -ExecutionPolicy Bypass -File <this file> -SkipBuild
#      powershell -ExecutionPolicy Bypass -File <this file> -KeepBridge
# ============================================================================

[CmdletBinding()]
param(
    [switch]$SkipBuild,     # flash the existing build without rebuilding
    [switch]$KeepBridge     # do not stop the RTT bridge first
)

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------- paths
$portDir  = $PSScriptRoot
$projDir  = Split-Path -Parent $portDir
$repoDir  = Split-Path -Parent $projDir
$buildDir = Join-Path $projDir 'build\Debug'

$elfPath  = Join-Path $buildDir 'project.elf'
$hexPath  = Join-Path $buildDir 'project.hex'
$binPath  = Join-Path $buildDir 'project.bin'
$logPath  = Join-Path $portDir  'flash_log.txt'

$cmakeExe = 'D:\AI_Models\CMark\bin\cmake.exe'
$gccBin   = 'D:\AI_Models\ARM_GNU_Toolchain\arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi\bin'
$ninjaBin = 'D:\AI_Models\ninja\ninja-win'
$jlinkExe = 'D:\MY_tool_at_ALL\J_Flash_APP\J-linkV9.64SEGGER\JLink_V964\JLink.exe'
$objcopy  = Join-Path $gccBin 'arm-none-eabi-objcopy.exe'

$flashBase = 0x08000000

function Fail([string]$msg) {
    Write-Host ''
    Write-Host "*** FAIL: $msg" -ForegroundColor Red
    Write-Host ''
    exit 1
}

function Sha256([string]$p) {
    return (Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash
}

# ---------------------------------------------------------------- banner
Write-Host ''
Write-Host '============================================================' -ForegroundColor Cyan
Write-Host ' STM32 flash with read-back verification' -ForegroundColor Cyan
Write-Host '============================================================' -ForegroundColor Cyan

# ---------------------------------------------------------------- 0) bridge
# J-Link is exclusive. A running RTT bridge will silently steal the session
# or make the connect fail.
if (-not $KeepBridge) {
    $busy = Get-Process python, pythonw, JLink, JLinkExe -ErrorAction SilentlyContinue
    if ($busy) {
        Write-Host ('[0] stopping RTT bridge / J-Link holders: ' + (($busy | ForEach-Object { $_.ProcessName }) -join ', '))
        $busy | Stop-Process -Force
        Start-Sleep -Milliseconds 800
    } else {
        Write-Host '[0] no bridge running'
    }
}

# ---------------------------------------------------------------- 1) build
if (-not $SkipBuild) {
    Write-Host '[1] building ...'
    $env:PATH = "$gccBin;$ninjaBin;$env:PATH"
    & $cmakeExe --build $buildDir
    if ($LASTEXITCODE -ne 0) { Fail "build returned $LASTEXITCODE" }
} else {
    Write-Host '[1] build skipped (-SkipBuild)'
}

foreach ($p in @($elfPath, $hexPath)) {
    if (-not (Test-Path -LiteralPath $p)) { Fail "missing artifact: $p" }
}

# ---------------------------------------------------------------- 2) freshness
# The exact bug this guard exists for: a stale hex that is not the linked ELF.
$elfT = (Get-Item -LiteralPath $elfPath).LastWriteTimeUtc
$hexT = (Get-Item -LiteralPath $hexPath).LastWriteTimeUtc

if ($hexT -lt $elfT) {
    Fail ("project.hex is OLDER than project.elf - the flashed image would not be this build.`n" +
          "      elf : $elfT`n      hex : $hexT")
}
Write-Host '[2] artifact freshness OK (hex belongs to this link)'

# ---------------------------------------------------------------- 3) parse the hex
# The read-back reference IS the hex: we compare exactly the bytes the image
# programs. Gaps between records are deliberately ignored - erased flash there
# is 0xFF and has no meaning. This avoids every VMA/LMA and gap-fill pitfall.
$img    = @{}
$upper  = 0
$badRec = 0

foreach ($line in [System.IO.File]::ReadAllLines($hexPath)) {
    $line = $line.Trim()
    if ($line.Length -lt 11 -or $line[0] -ne ':') { continue }

    $raw = New-Object byte[] (($line.Length - 1) / 2)
    for ($i = 0; $i -lt $raw.Length; $i++) {
        $raw[$i] = [Convert]::ToByte($line.Substring(1 + $i * 2, 2), 16)
    }

    $len  = $raw[0]
    $addr = ([int]$raw[1] -shl 8) -bor [int]$raw[2]
    $type = $raw[3]

    if ($type -eq 0) {
        if (($raw.Length - 5) -lt $len) { $badRec++; continue }
        $base = $upper + $addr
        for ($k = 0; $k -lt $len; $k++) { $img[$base + $k] = $raw[4 + $k] }
    } elseif ($type -eq 4) {
        $upper = ((([int]$raw[4] -shl 8) -bor [int]$raw[5]) * 0x10000)
    }
}

if ($img.Count -eq 0) { Fail "hex parsed to zero data bytes - refusing to flash" }
if ($badRec -gt 0) { Fail "hex contained $badRec malformed records" }

$addrs   = @($img.Keys)
# Measure-Object returns Double for numeric input; force integer types so the
# X8 format specifier below is valid (it rejects floating point).
$minAddr = [long](($addrs | Measure-Object -Minimum).Minimum)
$maxAddr = [long](($addrs | Measure-Object -Maximum).Maximum)
$span    = $maxAddr - $minAddr + 1

if ($minAddr -ne $flashBase) {
    Fail ("hex does not start at the flash base.`n" +
          ('      expected 0x{0:X8}, got 0x{1:X8}' -f $flashBase, $minAddr))
}
if ($span -gt (1024 * 1024)) {
    Fail ("hex spans $span bytes - that is not a plausible flash image")
}

$hexHash = Sha256 $hexPath

# ---------------------------------------------------------------- 3b) repo identity
$gitHash  = 'unknown'
$gitDirty = 'unknown'
Push-Location $repoDir
try {
    $gitHash  = (& git rev-parse --short HEAD 2>$null)
    $gitDirty = (& git status --porcelain 2>$null | Measure-Object).Count
} catch { }
Pop-Location

Write-Host '[3] identity'
Write-Host "      git      : $gitHash   (dirty files: $gitDirty)"
Write-Host "      elf      : $(Split-Path -Leaf $elfPath)  $((Get-Item -LiteralPath $elfPath).Length) bytes"
Write-Host "      hex      : $hexHash"
Write-Host ("      image    : {0} bytes programmed, span 0x{1:X8}..0x{2:X8}" -f $img.Count, $minAddr, $maxAddr)

# ---------------------------------------------------------------- 4) flash + read back
$tmpDir    = Join-Path $env:TEMP ('flashverify_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
New-Item -ItemType Directory -Path $tmpDir | Out-Null
$readBack  = Join-Path $tmpDir 'readback.bin'
$jlinkScr  = Join-Path $tmpDir 'flash.jlink'

$script = @()
$script += 'si SWD'
$script += 'speed 4000'
$script += 'device STM32G431C8'
$script += 'connect'
$script += 'erase'
$script += ('loadfile "' + $hexPath + '"')
$script += ('savebin "' + $readBack + '", ' + ('0x{0:X8}' -f $minAddr) + ', ' + ('0x{0:X}' -f $span))
$script += 'r'
$script += 'g'
$script += 'qc'
$script | Out-File -FilePath $jlinkScr -Encoding ascii

Write-Host '[4] flashing + reading back ...'
$jout = & $jlinkExe -CommanderScript $jlinkScr 2>&1
$jout | Out-File -FilePath (Join-Path $tmpDir 'jlink.log') -Encoding ascii

if ($LASTEXITCODE -ne 0) {
    $jout | Select-Object -Last 20 | ForEach-Object { Write-Host "      $_" }
    Fail "JLink exited with $LASTEXITCODE"
}
if (-not (Test-Path -LiteralPath $readBack)) {
    $jout | Select-Object -Last 20 | ForEach-Object { Write-Host "      $_" }
    Fail "read-back file was not produced by savebin"
}

# ---------------------------------------------------------------- 5) compare
# Compare ONLY the bytes the image programs, at their real addresses.
$onChip = [System.IO.File]::ReadAllBytes($readBack)

if ($onChip.Length -lt $span) {
    Fail "read-back is shorter than the image span ($($onChip.Length) < $span)"
}

$bad      = 0
$firstBad = -1
foreach ($a in $img.Keys) {
    if ($onChip[$a - $minAddr] -ne $img[$a]) {
        if ($firstBad -lt 0) { $firstBad = $a }
        $bad++
    }
}

$verdict = if ($bad -eq 0) { 'PASS' } else { 'FAIL' }

# ---------------------------------------------------------------- 6) report
Write-Host ''
if ($verdict -eq 'PASS') {
    Write-Host '  ##########################################################' -ForegroundColor Green
    Write-Host '  #  PASS - every programmed byte is on the chip, verified  #' -ForegroundColor Green
    Write-Host '  #  The board provably runs THIS build.                    #' -ForegroundColor Green
    Write-Host '  ##########################################################' -ForegroundColor Green
    Write-Host ("      verified {0} bytes across 0x{1:X8}..0x{2:X8}" -f $img.Count, $minAddr, $maxAddr)
} else {
    Write-Host '  ##########################################################' -ForegroundColor Red
    Write-Host '  #  FAIL - read-back DIFFERS from the image                #' -ForegroundColor Red
    Write-Host '  #  Do NOT trust any symbol-address reading on the target. #' -ForegroundColor Red
    Write-Host '  ##########################################################' -ForegroundColor Red
    Write-Host ("      $bad of $($img.Count) programmed bytes differ")
    Write-Host ("      first mismatch at 0x{0:X8}" -f $firstBad)
    Write-Host ("      image    = 0x{0:X2}" -f $img[$firstBad])
    Write-Host ("      on chip  = 0x{0:X2}" -f $onChip[$firstBad - $minAddr])
}

$line = '{0}  {1}  git={2} dirty={3}  hex={4}  bytes={5}  elfT={6}' -f `
        (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $verdict, $gitHash, $gitDirty, $hexHash.Substring(0,16), $img.Count, $elfT.ToString('HH:mm:ss')
Add-Content -LiteralPath $logPath -Value $line

Write-Host ''
Write-Host "      log : $logPath"
Write-Host "      tmp : $tmpDir"
Write-Host ''

if ($verdict -ne 'PASS') { exit 1 }
