# powershell script to build espp and install the find_package-able package
# (C++ static library + headers + esppConfig.cmake) together with the python
# `espp` package into a local staging prefix (<repo>/install). Point ../pc (and
# any external consumer) at it with -DCMAKE_PREFIX_PATH=<repo>/install.

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$prefix = Join-Path $repoRoot "install"
$buildDir = Join-Path $scriptDir "build"

# ESPP_BUILD_PYTHON=ON also builds/installs the python package (what CI
# publishes). ESPP_INSTALL=ON installs the find_package package into $prefix.
cmake -S $scriptDir -B $buildDir `
  -DCMAKE_BUILD_TYPE=Release `
  -DESPP_INSTALL=ON `
  -DESPP_BUILD_PYTHON=ON `
  -DCMAKE_INSTALL_PREFIX=$prefix

cmake --build $buildDir --config Release --target install --parallel 4
