param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]*$')]
    [string]$Version,
    [string]$ReleaseDirectory
)

$ErrorActionPreference = 'Stop'
if (-not $ReleaseDirectory) { $ReleaseDirectory = Join-Path $PSScriptRoot 'release-output' }
if (-not $env:GH_TOKEN) { throw 'GH_TOKEN is required to publish a GitHub Release.' }
if (-not $env:GITHUB_REPOSITORY) { throw 'GITHUB_REPOSITORY is not set.' }

$packageName = "CombatSim-$Version-windows-x64"
$assets = @(
    (Join-Path $ReleaseDirectory "$packageName.zip"),
    (Join-Path $ReleaseDirectory "$packageName.sha256.txt")
)
foreach ($asset in $assets) {
    if (-not (Test-Path -LiteralPath $asset -PathType Leaf)) {
        throw "Release asset was not generated: $asset"
    }
}

$apiRoot = "https://api.github.com/repos/$env:GITHUB_REPOSITORY"
$headers = @{
    Authorization = "Bearer $env:GH_TOKEN"
    Accept = 'application/vnd.github+json'
    'X-GitHub-Api-Version' = '2022-11-28'
}
$releaseUri = "$apiRoot/releases/tags/$([uri]::EscapeDataString($Version))"
try {
    $release = Invoke-RestMethod -Method Get -Uri $releaseUri -Headers $headers
}
catch {
    $statusCode = 0
    if ($_.Exception.Response -and $_.Exception.Response.StatusCode) {
        $statusCode = [int]$_.Exception.Response.StatusCode
    }
    if ($statusCode -ne 404) { throw }

    $releaseBody = @{
        tag_name = $Version
        target_commitish = 'main'
        name = "CombatSim $Version"
        generate_release_notes = $true
    } | ConvertTo-Json
    $release = Invoke-RestMethod -Method Post `
        -Uri "$apiRoot/releases" -Headers $headers `
        -ContentType 'application/json' -Body $releaseBody
}

$uploadUri = ($release.upload_url -replace '\{\?.*$', '')
foreach ($assetPath in $assets) {
    $assetName = [IO.Path]::GetFileName($assetPath)
    $oldAsset = $release.assets | Where-Object { $_.name -eq $assetName } | Select-Object -First 1
    if ($oldAsset) {
        Invoke-RestMethod -Method Delete `
            -Uri "$apiRoot/releases/assets/$($oldAsset.id)" -Headers $headers | Out-Null
    }

    $assetUri = "$uploadUri`?name=$([uri]::EscapeDataString($assetName))"
    Invoke-RestMethod -Method Post -Uri $assetUri -Headers $headers `
        -ContentType 'application/octet-stream' -InFile $assetPath | Out-Null
    Write-Host "Uploaded $assetName"
}

Write-Host "Published https://github.com/$env:GITHUB_REPOSITORY/releases/tag/$Version"
