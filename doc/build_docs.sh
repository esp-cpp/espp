# build-docs -t esp32 esp32s2 esp32c3 esp32s3 esp32h2 esp32c2 esp32c6 -l en --project-path ../
build-docs -t esp32 -l en --project-path ../
cp -rf _build/en/esp32/html/* ../docs/.
