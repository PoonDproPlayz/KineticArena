param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]*$')]
    [string]$Version,
    [string]$OutputDirectory
)

$ErrorActionPreference = 'Stop'
if (-not $OutputDirectory) { $OutputDirectory = Join-Path $PSScriptRoot 'release-output' }

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$exporterSource = Join-Path $repositoryRoot 'Fusion360Exporter'
$requiredFiles = @(
    (Join-Path $exporterSource 'CombatSimExport.py'),
    (Join-Path $exporterSource 'config.json')
)
foreach ($requiredFile in $requiredFiles) {
    if (-not (Test-Path -LiteralPath $requiredFile -PathType Leaf)) {
        throw "Required Fusion exporter file is missing: $requiredFile"
    }
}

$outputRoot = [IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
$packageName = "CombatSim-$Version-fusion360-exporter"
$zipPath = Join-Path $outputRoot "$packageName.zip"
$hashPath = Join-Path $outputRoot "$packageName.sha256.txt"
$stageRoot = Join-Path $outputRoot ".staging-$([guid]::NewGuid().ToString('N'))"
$packageRoot = Join-Path $stageRoot $packageName

try {
    $exporterPackage = Join-Path $packageRoot 'Fusion360Exporter'
    New-Item -ItemType Directory -Path $exporterPackage -Force | Out-Null
    Copy-Item -LiteralPath $requiredFiles -Destination $exporterPackage

    @"
CombatSim Fusion 360 Exporter $Version

Install and run:
1. Extract this ZIP.
2. In Fusion 360, open Utilities > Scripts and Add-ins > Scripts.
3. Click the + button and select the extracted Fusion360Exporter folder.
4. Select CombatSimExport and click Run.
5. Choose an output folder for the exported CombatSim bundle.

Keep CombatSimExport.py and config.json together inside the Fusion360Exporter
folder. The exporter runs inside Fusion 360 and is not a standalone program.
"@ | Set-Content -LiteralPath (Join-Path $packageRoot 'README.txt') -Encoding UTF8

    if (Test-Path -LiteralPath $zipPath -PathType Leaf) {
        Remove-Item -LiteralPath $zipPath -Force
    }
    if (Test-Path -LiteralPath $hashPath -PathType Leaf) {
        Remove-Item -LiteralPath $hashPath -Force
    }
    Compress-Archive -Path (Join-Path $packageRoot '*') `
        -DestinationPath $zipPath -CompressionLevel Optimal
    $hash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
    "$hash  $([IO.Path]::GetFileName($zipPath))" |
        Set-Content -LiteralPath $hashPath -Encoding ASCII

    Write-Host "Fusion exporter ZIP: $zipPath"
    Write-Host "SHA-256: $hash"
}
finally {
    $outputPrefix = $outputRoot.TrimEnd([IO.Path]::DirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
    $stageFullPath = [IO.Path]::GetFullPath($stageRoot)
    if ((Test-Path -LiteralPath $stageFullPath -PathType Container) -and
        $stageFullPath.StartsWith($outputPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        Remove-Item -LiteralPath $stageFullPath -Recurse -Force
    }
}
