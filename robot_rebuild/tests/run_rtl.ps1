[CmdletBinding()]
param([string]$SimulatorBin = $env:MODELSIM_BIN)

$ErrorActionPreference = 'Stop'
function Resolve-SimulatorTool([string]$Name) {
    if ($SimulatorBin) {
        foreach ($candidateName in @("$Name.exe", $Name)) {
            $candidate = Join-Path $SimulatorBin $candidateName
            if (Test-Path -LiteralPath $candidate -PathType Leaf) {
                return (Resolve-Path -LiteralPath $candidate).Path
            }
        }
        throw "Missing $Name in simulator directory: $SimulatorBin"
    }
    foreach ($candidateName in @("$Name.exe", $Name)) {
        $command = Get-Command $candidateName -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($command) { return $command.Source }
    }
    throw "Missing $Name; set -SimulatorBin, MODELSIM_BIN, or PATH."
}
$vlib = Resolve-SimulatorTool 'vlib'
$vlog = Resolve-SimulatorTool 'vlog'
$vsim = Resolve-SimulatorTool 'vsim'
$testRoot = $PSScriptRoot
$fpgaRoot = Join-Path (Split-Path $testRoot -Parent) 'fpga'
Push-Location $testRoot
try {
    & $vlib work
    if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
    $rtlFiles = Get-ChildItem (Join-Path $fpgaRoot 'rtl') -Filter '*.v' | ForEach-Object { $_.FullName }
    $vendorFiles = Get-ChildItem (Join-Path $fpgaRoot 'vendor') -Filter '*.v' | Where-Object { $_.Name -ne 'gowin_pll_mod.v' } | ForEach-Object { $_.FullName }
    $testFiles = @(Get-ChildItem $testRoot -Filter 'tb_*.sv' | ForEach-Object { $_.FullName })
    & $vlog -sv @rtlFiles @vendorFiles @testFiles
    if ($LASTEXITCODE -ne 0) { throw 'RTL compile failed' }
    foreach ($test in (Get-ChildItem $testRoot -Filter 'tb_*.sv')) {
        $testLog = Join-Path $testRoot "$($test.BaseName).log"
        & $vsim -batch -l $testLog "work.$($test.BaseName)" -do 'onerror {quit -code 1}; onbreak {quit -code 1}; run -all; quit -code 0'
        $logText = Get-Content -LiteralPath $testLog -Raw
        if ($LASTEXITCODE -ne 0 -or $logText -match '\*\* (Fatal|Error)' -or $logText -notmatch 'PASS ') {
            throw "Simulation failed: $($test.Name)"
        }
    }
} finally { Pop-Location }
