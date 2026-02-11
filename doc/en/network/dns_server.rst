DNS Server
**********

The DNS server provides a simple DNS responder that can be used to implement
captive portal functionality. It responds to all DNS queries with a configured
IP address, which is useful for redirecting clients to a local web server during
WiFi provisioning or configuration.

The server uses UDP socket for receiving DNS queries and can be easily integrated
with WiFi AP mode to create a captive portal experience.

.. ------------------------------- Example -------------------------------------

.. toctree::

    dns_server_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/dns_server.inc
