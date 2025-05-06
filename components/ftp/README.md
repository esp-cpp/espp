# FTP (File Transfer Protocol) Component

The `ftp` component provides an implementation of various parts of the file
transfer protocol.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [FTP (File Transfer Protocol) Component](#ftp-file-transfer-protocol-component)
  - [FTP Server](#ftp-server)
  - [Example](#example)

<!-- markdown-toc end -->

## FTP Server

The `FtpServer` class implements a simple FTP server. It accepts new connections
and spawns a new `FtpClientSession` for each one. Each session is handled in its
own thread.

The `FtpClientSession` class implements the FTP protocol. It is responsible for
handling the commands and sending the responses.

Note that the FTP server does not implement any authentication mechanism. It
accepts any username and password.

## Example

The [example](./example) showcases the use of the `FtpServer` from the `ftp`
component.

