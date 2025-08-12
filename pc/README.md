# ESPP PC Code

This folder contains some test python scripts for loading the `espp` static
library built within the `espp/lib` folder into c++ on a host system running
Linux, MacOS, or Windows.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ESPP PC Code](#espp-pc-code)
  - [Setup](#setup)
  - [Quick Start](#quick-start)

<!-- markdown-toc end -->

## Setup

First, ensure that you have built the shared objects in the `espp/lib` folder.
If you haven't done so yet, navigate to the `espp/lib` folder and run the
following:

```console
# if macos/linux:
./build.sh
# if windows
./build.ps1
```

## Quick Start

To run a test, you can simply run the executable from the terminal:

```console
# if windows:
./build/Release/udp_client.exe
# if macos / linux:
./build/udp_client
```
