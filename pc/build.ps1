# powershell script to build the project using cmake

# Create build directory if it doesn't exist
$buildDir = "build"
if (-not (Test-Path -Path $buildDir)) {
    New-Item -ItemType Directory -Path $buildDir
}

# Change to the build directory
Set-Location -Path $buildDir

# Run cmake
cmake ..

# Run cmake --build . --config Release
cmake --build . --config Release

# Change back to the original directory
Set-Location -Path ..
