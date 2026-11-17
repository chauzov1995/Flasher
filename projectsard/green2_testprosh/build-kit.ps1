param(
    [string]$ArduinoCli = "$env:LOCALAPPDATA/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe",
    [string]$ArduinoConfig = "$env:USERPROFILE/.arduinoIDE/arduino-cli.yaml",
    [string]$BuildDirectory = "$env:TEMP/aventa-green2-test-build"
)
$ErrorActionPreference = 'Stop'
$sketch = $PSScriptRoot
$repo = Split-Path (Split-Path $sketch -Parent) -Parent
$kit = Join-Path $repo 'projects/testgreen2'
if (!(Test-Path -LiteralPath (Join-Path $sketch 'green2_wifi.local.h'))) {
    throw 'Copy green2_wifi.example.h to green2_wifi.local.h and configure bench Wi-Fi before building the kit.'
}
$fqbn = 'esp32:esp32:esp32c6:FlashSize=8M,PartitionScheme=default_8MB,CDCOnBoot=default'
& $ArduinoCli compile --config-file $ArduinoConfig --fqbn $fqbn --build-path $BuildDirectory $sketch
if ($LASTEXITCODE -ne 0) { throw 'Arduino build failed; kit not updated.' }
$artifacts = @('green2_testprosh.ino.bin', 'green2_testprosh.ino.bootloader.bin', 'green2_testprosh.ino.partitions.bin')
foreach ($name in $artifacts) {
    $source = Join-Path $BuildDirectory $name
    if (!(Test-Path -LiteralPath $source) -or (Get-Item -LiteralPath $source).Length -eq 0) {
        throw "Missing build output: $source"
    }
}
foreach ($name in $artifacts) { Copy-Item -LiteralPath (Join-Path $BuildDirectory $name) -Destination (Join-Path $kit $name) }
$hashes = [ordered]@{}
foreach ($name in $artifacts) { $hashes[$name] = (Get-FileHash -LiteralPath (Join-Path $kit $name) -Algorithm SHA256).Hash.ToLowerInvariant() }
$sources = [ordered]@{}
foreach ($name in 'green2_testprosh.ino', 'registration.h', 'console.h') {
    $sources[$name] = (Get-FileHash -LiteralPath (Join-Path $sketch $name) -Algorithm SHA256).Hash.ToLowerInvariant()
}
[ordered]@{
    firmware = 'green2-test-1.0.2'
    builtUtc = [DateTime]::UtcNow.ToString('o')
    fqbn = $fqbn
    core = 'esp32:esp32@3.3.12'
    arduinoJson = '7.4.3'
    esptool = '5.3.1'
    benchWifiIncluded = $true
    sha256 = $hashes
    sourceSha256 = $sources
} | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $kit 'build-info.json') -Encoding utf8
Write-Output "Kit ready: $kit"
