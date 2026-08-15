# powershell script to build the pc/ example tests against the INSTALLED espp
# package. Run ../lib/build.ps1 first; it installs espp into <repo>/install,
# which we point find_package(espp) at via CMAKE_PREFIX_PATH.

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$prefix = Join-Path $repoRoot "install"
$buildDir = Join-Path $scriptDir "build"

cmake -S $scriptDir -B $buildDir `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_PREFIX_PATH=$prefix

cmake --build $buildDir --config Release --parallel 4
