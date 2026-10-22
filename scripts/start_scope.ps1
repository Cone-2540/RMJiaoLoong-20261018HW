param([string]$ScopeDir = "$env:USERPROFILE\Desktop\Scope-v0.4.5")
$ErrorActionPreference = 'Stop'
function Test-Scope {
    try { $null = Invoke-RestMethod 'http://127.0.0.1:8001/api/session' -TimeoutSec 2; return $true }
    catch { return $false }
}
if (-not (Test-Scope)) {
    Start-Process -FilePath (Join-Path $ScopeDir 'Scope.exe') -WorkingDirectory $ScopeDir -WindowStyle Hidden
    for ($attempt=0; $attempt -lt 30; $attempt++) {
        Start-Sleep -Milliseconds 500
        if (Test-Scope) { break }
    }
}
if (-not (Test-Scope)) { throw 'Scope HTTP service did not start on port 8001.' }
python (Join-Path $PSScriptRoot 'setup_scope.py') --scope-dir $ScopeDir
if ($LASTEXITCODE -ne 0) { throw 'Scope setup failed.' }
