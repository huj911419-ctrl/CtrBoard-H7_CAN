param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$originalPath = $env:PATH
$compilerDir = Split-Path -Parent (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = "$compilerDir;$env:PATH"
Push-Location $repo
try {
    New-Item -ItemType Directory -Force 'build/host-tests' | Out-Null
    foreach ($motorId in @(0, 6)) {
        $exe = "build/host-tests/can-motors-$motorId.exe"
        & $Compiler -std=c11 -Wall -Wextra -Werror "-DDJI_MOTOR_ID=$motorId" `
            -Itests/stubs -IUser tests/test_can_motors.c User/dji_motor.c User/bsp_fdcan.c -o $exe
        if ($LASTEXITCODE -ne 0) { throw "Host compile failed: ID=$motorId" }
        & (Join-Path $repo $exe)
        if ($LASTEXITCODE -ne 0) { throw "Host tests failed: ID=$motorId" }
    }
} finally { $env:PATH = $originalPath; Pop-Location }
