param(
    [string]$OutputPath = ""
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$includedPaths = @(
    'README.md',
    'STATUS.md',
    'GROUND_TRUTH.md',
    'docs',
    'scripts',
    'webapp',
    'ros2_ws/src/lite3_odom_tf_adapter'
)

foreach ($relativePath in $includedPaths) {
    $sourcePath = Join-Path $projectRoot $relativePath
    if (-not (Test-Path -LiteralPath $sourcePath)) {
        throw "Required package content is missing: $relativePath"
    }
}

if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
    $OutputPath = Join-Path $projectRoot "dist/Lite3RoomMapper_$stamp.zip"
} elseif (-not [System.IO.Path]::IsPathRooted($OutputPath)) {
    $OutputPath = Join-Path (Get-Location).Path $OutputPath
}
$outputFullPath = [System.IO.Path]::GetFullPath($OutputPath)
if (Test-Path -LiteralPath $outputFullPath) {
    throw "Refusing to overwrite an existing package: $outputFullPath"
}
$outputDirectory = Split-Path -Parent $outputFullPath
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null

$tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath()).TrimEnd(
    [System.IO.Path]::DirectorySeparatorChar,
    [System.IO.Path]::AltDirectorySeparatorChar)
$stageRoot = Join-Path $tempRoot ("Lite3RoomMapper_" + [guid]::NewGuid().ToString('N'))
$stageFullPath = [System.IO.Path]::GetFullPath($stageRoot)
if ([System.IO.Path]::GetDirectoryName($stageFullPath) -ne $tempRoot) {
    throw 'Refusing to stage package outside the system temporary directory.'
}

try {
    New-Item -ItemType Directory -Path $stageFullPath | Out-Null
    foreach ($relativePath in $includedPaths) {
        $sourcePath = Join-Path $projectRoot $relativePath
        $destinationPath = Join-Path $stageFullPath $relativePath
        New-Item -ItemType Directory -Path (Split-Path -Parent $destinationPath) -Force | Out-Null
        Copy-Item -LiteralPath $sourcePath -Destination $destinationPath -Recurse -Force
    }

    Compress-Archive -Path (Join-Path $stageFullPath '*') -DestinationPath $outputFullPath -CompressionLevel Optimal
    Write-Output "Package created: $outputFullPath"
} finally {
    if ((Test-Path -LiteralPath $stageFullPath) -and
        [System.IO.Path]::GetDirectoryName($stageFullPath) -eq $tempRoot) {
        Remove-Item -LiteralPath $stageFullPath -Recurse -Force
    }
}
