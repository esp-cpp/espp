import socket
import sys
import threading
import time

import cv2
import io
import struct
import numpy as np

import io

'''

NOTE: This code is designed to handle MJPEG video streams over RTP/RTCP.

Some useful references:
* https://www.rfc-editor.org/rfc/rfc2435
  RTP Payload Format for JPEG-compressed Video
* https://en.wikipedia.org/wiki/JPEG_File_Interchange_Format
  JFIF - the JPEG File Interchange Format
* https://en.wikipedia.org/wiki/JPEG
  Wikipedia page for JPEG

The huffman tables are not transferred with the images, but the quantization
tables are. The rest of the JPEG header/data is stripped from the stream, such
that to properly display it, you need to rebuild the jpeg header data based on
the simplified RTP & JFIF header data. Once you've done that, the jpeg frames
can be decoded properly.

Somewhat unrelated, you can convert mp4 to mjpeg and mp3:

```bash
ffmpeg -i input.mp4 -vf "fps=30,scale=-1:176:flags=lanczos,crop=220:in_h:(in_w-220)/2:0" -q:v 9 220_30fps.mjpeg
# for MP3
ffmpeg -i input.mp4 -ar 44100 -ac 1 -q:a 9 44100.mp3
# for PCM
ffmpeg -i input.mp4 -f u16be -acodec pcm_u16le -ar 44100 -ac 1 44100_u16le.pcm
```

'''

dc_luminance_table = bytearray([
    0x00,
    0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
])

#default luminance AC Huffman table
ac_luminance_table = bytearray([
    0x00,
    0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04,
    0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0,
    0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34,
    0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26,
    0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
    0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4,
    0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3,
    0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2,
    0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
    0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9,
    0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,
    0xF9, 0xFA
])

dc_chrominance_table = bytearray([
    0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
])

ac_chrominance_table = bytearray([
    0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0,
    0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26,
    0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
    0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3,
    0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
    0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,
    0xF9, 0xFA
])

huffman_table = [
    # Huffman table DC (luminance)
    0xff, 0xc4,
    0x00, 0x1f, 0x00,
    0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
    # Huffman table AC (luminance)
    0xff, 0xc4,
    0x00, 0xb5, 0x10,
    0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
    # Huffman table DC (chrominance)
    0xff, 0xc4,
    0x00, 0x1f, 0x01,
    0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
    # Huffman table AC (chrominance)
    0xff, 0xc4,
    0x00, 0xb5, 0x11,
    0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
]

class RtpPacket:
    def __init__(self, data):
        self.data = data
        rtp_info, payload_and_marker, self.sequence_number, self.timestamp, self.ssrc = struct.unpack('>BBHII', data[:12])
        self.version = (rtp_info & 0b11000000) >> 6
        self.padding = (rtp_info & 0b00100000) >> 5
        self.extension = (rtp_info & 0b00010000) >> 4
        self.csrc_count = rtp_info & 0b00001111
        self.payload_type = (payload_and_marker & 0x7F)
        self.marker = (payload_and_marker & 0b10000000) >> 7

        # self.version = (data[0] & 0b11000000) >> 6
        # self.padding = (data[0] & 0b00100000) >> 5
        # self.extension = (data[0] & 0b00010000) >> 4
        # self.csrc_count = data[0] & 0b00001111
        # self.marker = (data[1] & 0b10000000) >> 7
        # self.payload_type = data[1] & 0b01111111
        # self.sequence_number = data[2] * 256 + data[3]
        # self.timestamp = data[4] * 256 * 256 * 256 + data[5] * 256 * 256 + data[6] * 256 + data[7]
        # self.ssrc = data[8] * 256 * 256 * 256 + data[9] * 256 * 256 + data[10] * 256 + data[11]

        self.payload = data[12:]

    def __repr__(self):
        return f"RtpPacket(payload_type={self.payload_type}, marker={self.marker}, sequence_number={self.sequence_number}, timestamp={self.timestamp}, ssrc={self.ssrc})"

    def get_payload(self):
        return self.payload

    def get_payload_type(self):
        return self.payload_type

    def get_marker(self):
        return self.marker

class RtpJpegPacket(RtpPacket):
    def __init__(self, data):
        super().__init__(data)
        rtp_payload = self.get_payload()
        self.type_specific, self.frag_offset, self.frag_type, self.q, self.width, self.height = struct.unpack('>B3s BBBB', rtp_payload[:8])
        self.frag_offset = int.from_bytes(self.frag_offset, byteorder='big')
        self.q = int(self.q)
        self.width = int(self.width) * 8
        self.height = int(self.height) * 8

        self.jpeg_data = rtp_payload[8:]

        if 128 <= self.q <= 255:
            # bytes 8,9,10 are all 0
            num_quant_bytes = rtp_payload[11]
            quant_size = 64
            expected_quant_bytes = 2 * quant_size
            if num_quant_bytes != expected_quant_bytes:
                print(f"Unexpected quant bytes: {num_quant_bytes}, expected {expected_quant_bytes}")
                print(f"rtp_payload: {rtp_payload[:12]}")
            else:
                q0_offset = 12
                q1_offset = q0_offset + quant_size
                q1_end = q1_offset + quant_size
                self.q0 = rtp_payload[q0_offset:q1_offset]
                self.q1 = rtp_payload[q1_offset:q1_end]
                self.jpeg_data = rtp_payload[q1_end:]

    def __repr__(self):
        return f"RtpJpegPacket(payload_type={self.payload_type}, marker={self.marker}, sequence_number={self.sequence_number}, timestamp={self.timestamp}, ssrc={self.ssrc}, width={self.width}, height={self.height}, q={self.q}, frag_type={self.frag_type}, frag_offset={self.frag_offset})"

    def get_width(self):
        return self.width

    def get_frag_offset(self):
        return self.frag_offset

    def get_frag_type(self):
        return self.frag_type

    def get_height(self):
        return self.height

    def get_q0(self):
        return self.q0

    def get_q1(self):
        return self.q1

    def get_jpeg_data(self):
        return self.jpeg_data


class JpegHeader:
    def __init__(self, width, height, q0_quantization_table, q1_quantization_table):
        self.width = width
        self.height = height

        self.data = io.BytesIO()
        self.data.write(b'\xFF\xD8')  # Start Of Image (SOI) marker
        # JFIF APP0 marker
        jfif_app0_marker = bytearray([
            0xFF, 0xE0,  # APP0 marker
            0x00, 0x10,  # Length (16 bytes)
            0x4A, 0x46, 0x49, 0x46, 0x00,  # JFIF identifier
            0x01, 0x01,  # JFIF version 1.1
            0x01,        # Units: DPI
            0x00, 0x00,  # X density (2 bytes)
            0x00, 0x00,  # Y density (2 bytes)
            0x00, 0x00   # No thumbnail (width 0, height 0)
        ])
        self.data.write(jfif_app0_marker)

        # Quantization table (DQT) marker for luminance
        # marker(0xFFDB), size (0x0043 = 67), index (0x00)
        self.data.write(b'\xFF\xDB\x00\x43\x00')
        self.data.write(bytearray(q0_quantization_table))

        # Quantization table (DQT) marker for chrominance
        # marker(0xFFDB), size (0x0043 = 67), index (0x01)
        self.data.write(b'\xFF\xDB\x00\x43\x01')
        self.data.write(bytearray(q1_quantization_table))

        self.data.write(bytes(huffman_table))

        # Frame header (SOF0) marker
        sof0_marker = bytearray([
            0xFF, 0xC0,  # SOF0 marker
            0x00, 0x11,  # Length (17 bytes)
            0x08,        # Data precision: 8 bits
            *self.height.to_bytes(2, 'big'), # 0x01, 0xE0,  # Image height: 240
            *self.width.to_bytes(2, 'big'), # 0x01, 0xE0,  # Image width: 240
            0x03,        # Number of components: 3 (YCbCr)
            0x01, 0x21, 0x00,  # Component 1 (Y):  horizontal sampling factor = 2, vertical sampling factor = 1, quantization table ID = 0
            0x02, 0x11, 0x01,  # Component 2 (Cb): horizontal sampling factor = 1, vertical sampling factor = 1, quantization table ID = 1
            0x03, 0x11, 0x01   # Component 3 (Cr): horizontal sampling factor = 1, vertical sampling factor = 1, quantization table ID = 1
        ])
        self.data.write(sof0_marker)

        # Scan header (SOS) marker
        # marker(0xFFDA), size of SOS (0x000C), num components(0x03),
        # component specification parameters,
        # spectral selection (0x003F),
        # successive appromiation parameters (0x00)
        self.data.write(b'\xFF\xDA\x00\x0C\x03\x01\x00\x02\x11\x03\x11\x00\x3F\x00')

    def __repr__(self):
        return f"JpegHeader(width={self.width}, height={self.height})"

    def get_data(self):
        return self.data.getvalue()

class JpegFrame:
    def __init__(self, RtpJpegPacket):
        self.jpeg_header = JpegHeader(RtpJpegPacket.get_width(), RtpJpegPacket.get_height(), RtpJpegPacket.get_q0(), RtpJpegPacket.get_q1())
        self.jpeg_data = RtpJpegPacket.get_jpeg_data()

    def __repr__(self):
        return f"JpegFrame(width={self.jpeg_header.width}, height={self.jpeg_header.height}, jpeg_data={self.jpeg_data})"

    def add_packet(self, rtp_jpeg_packet):
        self.jpeg_data += rtp_jpeg_packet.get_jpeg_data()

    def get_data(self):
        data = io.BytesIO()
        data.write(self.jpeg_header.get_data())
        data.write(self.jpeg_data)
        data.write(b'\xFF\xD9')  # End Of Image (EOI) marker
        return data.getvalue()


class RtspClient:
    def __init__(self, server, port, rtsp_uri):
        self.server = server
        self.port = port
        self.cseq = 0
        self.session_id = ""
        self.rtsp_uri = rtsp_uri

    def connect(self):
        self.sock = socket.create_connection((self.server, self.port))
        self.send_request("OPTIONS", "*")

    def send_request(self, method, uri, headers=None):
        if headers is None:
            headers = {}

        request = f"{method} {uri} RTSP/1.0\r\n"
        request += f"CSeq: {self.cseq}\r\n"
        if self.session_id:
            request += f"Session: {self.session_id}\r\n"

        for key, value in headers.items():
            request += f"{key}: {value}\r\n"

        request += "User-Agent: RtspClient\r\n"
        request += "\r\n"

        self.sock.sendall(request.encode())
        response = self.sock.recv(4096)
        print("Response:", response.decode())

        self.cseq += 1

    def describe(self):
        self.send_request("DESCRIBE", self.rtsp_uri, {"Accept": "application/sdp"})

    def setup(self, transport):
        self.send_request("SETUP", self.rtsp_uri, {"Transport": transport})

    def play(self):
        self.send_request("PLAY", self.rtsp_uri)

    def pause(self):
        self.send_request("PAUSE", self.rtsp_uri)

    def teardown(self):
        self.send_request("TEARDOWN", self.rtsp_uri)

    def start_receiving_video_stream(self, rtp_port, rtcp_port):
        self.rtp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtp_socket.bind(("0.0.0.0", rtp_port))

        self.rtcp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtcp_socket.bind(("0.0.0.0", rtcp_port))

        print(f"Listening for RTP packets on port {rtp_port}")
        print(f"Listening for RTCP packets on port {rtcp_port}")

        # NOTE: right now the rtp_thread must be run in the main thread context
        #       (mac os cannot run cv2.imshow in another thread), and we are not
        #       receiving any RTCP packets, so we've simply stopped spawning
        #       those threads and are instead direclty running the rtp_thread's
        #       function

        # TODO: get threaded cv2.imshow working (or have it simply update the
        #       frame and have opencv show happen in main thread context)

        # rtp_thread = threading.Thread(target=self.handle_rtp_packet)
        # rtcp_thread = threading.Thread(target=self.handle_rtcp_packet)

        # rtp_thread.start()
        # rtcp_thread.start()

        # rtp_thread.join()
        # rtcp_thread.join()

        self.handle_rtp_packet()

    def handle_rtp_packet(self):
        # for this example we'll show the received video stream in an opencv
        # window
        window_name = 'MJPEG Stream'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        # set the window as always on top
        cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)
        jpeg_frame = None
        while True:
            # Process RTP packet in rtp_data
            rtp_data, addr = self.rtp_socket.recvfrom(8192)
            rtp_packet = RtpJpegPacket(rtp_data)

            # TODO: handle out of order packets
            #       (packets whose seq_num is not equal to the previous packet's
            #       seq_num)

            frag_offset = rtp_packet.get_frag_offset()
            if frag_offset == 0:
                # this is the first packet of a new frame, so we need to
                # create a new JpegFrame object
                jpeg_frame = JpegFrame(rtp_packet)
            elif jpeg_frame is not None:
                # this is a continuation of a previous frame, so we need to
                # add the data to the existing JpegFrame object
                jpeg_frame.add_packet(rtp_packet)
            else:
                # we don't have a JpegFrame object yet, so we can't do
                # anything with this packet
                print(f"Received a packet with frag_offset = {frag_offset} > 0, but no JpegFrame object exists yet")
                continue

            # check if this is the last packet of the frame
            # (the last packet will have the M bit set)
            marker_bit = rtp_packet.get_marker()
            if marker_bit:
                # this is the last packet of the frame, so we can decode
                # the frame and show it in the opencv window
                buf = jpeg_frame.get_data()
                # print(f"Decoding image size={len(buf)}")
                frame = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    # print(f"Decoded frame: {frame.shape}\n\n")
                    # our images are flipped vertically, fix it :)
                    # 0 = vertical, 1 = horizontal, -1 = both vertical and horiztonal
                    frame = cv2.flip(frame, 0)
                    cv2.imshow('MJPEG Stream', frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                else:
                    print("Failed to decode frame")

    def handle_rtcp_packet(self):
        while True:
            rtcp_data, addr = self.rtcp_socket.recvfrom(8192)
            print("Received rtcp packet:", rtcp_data)
            # Process RTCP packet in rtcp_data
            # ...
            # The handle_rtcp_packet function currently does nothing, but you
            # can implement it to process RTCP packets, such as sender reports
            # or receiver reports, depending on your application requirements.

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python rtsp_client.py <server> <port>")
        sys.exit(1)

    server, port = sys.argv[1], int(sys.argv[2])
    rtsp_uri = f"rtsp://{server}:{port}/mjpeg/1"
    client = RtspClient(server, port, rtsp_uri)
    client.connect()


    # Call DESCRIBE method to get SDP information
    client.describe()

    # The following lines are placeholders for RTP and RTCP ports
    # You should parse the RTSP SETUP response and set the RTP and RTCP ports accordingly
    rtp_port = 5000
    rtcp_port = 5001
    if len(sys.argv) == 4:
        rtp_port = int(sys.argv[3])
        rtcp_port = rtp_port + 1

    # Set up the transport header with the RTP and RTCP ports
    transport_header = f"RTP/AVP;unicast;client_port={rtp_port}-{rtcp_port}"
    client.setup(transport_header)

    # Start streaming
    print("Streaming:", rtsp_uri)
    client.play()

    # Start receiving video stream
    client.start_receiving_video_stream(rtp_port, rtcp_port)

    print("pause")
    client.pause()

    # Close the RTSP connection
    print("Teardown")
    client.teardown()
