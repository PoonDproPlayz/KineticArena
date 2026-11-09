$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

if (-not (Get-Command node -ErrorAction SilentlyContinue)) {
  if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
    throw "Node.js is missing and WinGet is unavailable. Install Node.js LTS, then click Set up online service again."
  }
  Write-Host "Installing Node.js LTS for the one-time online service setup..."
  & winget install --id OpenJS.NodeJS.LTS --exact --source winget `
    --accept-source-agreements --accept-package-agreements
  if ($LASTEXITCODE -ne 0) { throw "WinGet could not install Node.js LTS." }
  $machinePath = [Environment]::GetEnvironmentVariable('Path', 'Machine')
  $userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
  $env:Path = "$machinePath;$userPath;$env:Path"
  if (-not (Get-Command node -ErrorAction SilentlyContinue)) {
    throw "Node.js was installed, but PowerShell cannot find it yet. Close this window, restart KineticArena, and click Set up online service again."
  }
}

if (-not (Test-Path -LiteralPath (Join-Path $PSScriptRoot "node_modules/wrangler/bin/wrangler.js"))) {
  npm install
  if ($LASTEXITCODE -ne 0) { throw "Could not install Wrangler." }
}

if (-not $env:CLOUDFLARE_API_TOKEN) {
  npx wrangler login
  if ($LASTEXITCODE -ne 0) { throw "Cloudflare login did not complete." }
}

$deployOutput = & npx wrangler deploy 2>&1
$deployExitCode = $LASTEXITCODE
$deployOutput | ForEach-Object { Write-Host $_ }
if ($deployExitCode -ne 0) { throw "Cloudflare deployment failed." }

$outputText = $deployOutput -join "`n"
$endpointMatch = [regex]::Match($outputText, 'https://[A-Za-z0-9.-]+\.workers\.dev')
if ($endpointMatch.Success) {
  $endpointPath = Join-Path $PSScriptRoot "worker-url.txt"
  [System.IO.File]::WriteAllText($endpointPath, $endpointMatch.Value + [Environment]::NewLine)
  Write-Host "Worker URL saved to $endpointPath"
}
