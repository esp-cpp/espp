Stream Frame APIs
*****************

.. toctree::
    :maxdepth: 1

    stream_frame

The `stream_frame` component is a tiny, dependency-free wire-framing codec:
CRC-32-verified, length-delimited, typed frames plus an incremental,
resynchronizing ``StreamParser`` for carrying messages over any raw byte stream
(USB vendor / CDC, TCP/UDP sockets, UART, ...). It is the shared substrate under
the OTA stream protocol, the crash-dump service and the :doc:`../dispatcher/index`
multiplexer.
