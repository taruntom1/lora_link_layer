param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ExtraArgs
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDocsExe = Join-Path $repoRoot '.venv\Scripts\build-docs.exe'

if (-not (Test-Path $buildDocsExe)) {
    throw "build-docs executable not found at '$buildDocsExe'. Create/activate .venv and install docs requirements first."
}

$env:PYTHONUTF8 = '1'

$baseArgs = @(
    '--project-path', 'docs',
    '--source-dir', 'docs',
    '--doxyfile_dir', 'docs',
    '-t', 'esp32s3',
    '-l', 'en',
    'build'
)

& $buildDocsExe @baseArgs @ExtraArgs
