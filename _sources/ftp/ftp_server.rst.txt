FTP Server
**********

The `FtpServer` class implements a simple FTP server. It accepts new connections
and spawns a new `FtpClientSession` for each one. Each session is handled in its
own thread.

The `FtpClientSession` class implements the FTP protocol. It is responsible for
handling the commands and sending the responses.

Note that the FTP server does not implement any authentication mechanism. It
accepts any username and password.

.. ------------------------------- Example -------------------------------------

.. toctree::

   ftp_server_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/ftp_server.inc
.. include-build-file:: inc/ftp_client_session.inc
