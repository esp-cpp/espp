"""Offline API tests for the new RTSP multi-codec packetizer/depacketizer classes.

This script exercises the packetizer and depacketizer APIs without any network
activity, making it suitable for host-side validation of the RTSP component's
RTP layer.

Usage:
    python rtsp_api_test.py
"""

import sys
import traceback

from support_loader import espp

passed = 0
failed = 0


def test(name, fn):
    """Run a single test and track pass/fail."""
    global passed, failed
    try:
        fn()
        print(f"  [PASS] {name}")
        passed += 1
    except Exception:
        print(f"  [FAIL] {name}")
        traceback.print_exc()
        failed += 1


# ---------------------------------------------------------------------------
# MediaType enum
# ---------------------------------------------------------------------------
def test_media_type_enum():
    assert hasattr(espp, "MediaType"), "MediaType enum missing"
    assert hasattr(espp.MediaType, "audio"), "MediaType.audio missing"
    assert hasattr(espp.MediaType, "video"), "MediaType.video missing"
    assert espp.MediaType.audio != espp.MediaType.video


# ---------------------------------------------------------------------------
# RtpPayloadChunk
# ---------------------------------------------------------------------------
def test_rtp_payload_chunk():
    chunk = espp.RtpPayloadChunk()
    assert hasattr(chunk, "data"), "RtpPayloadChunk.data missing"
    assert hasattr(chunk, "marker"), "RtpPayloadChunk.marker missing"


# ---------------------------------------------------------------------------
# MJPEG Packetizer
# ---------------------------------------------------------------------------
def test_mjpeg_packetizer_construction():
    cfg = espp.MjpegPacketizer.Config()
    cfg.max_payload_size = 1400
    p = espp.MjpegPacketizer(cfg)
    assert p.get_payload_type() == 26
    assert p.get_clock_rate() == 90000


def test_mjpeg_packetizer_sdp():
    p = espp.MjpegPacketizer(espp.MjpegPacketizer.Config())
    ml = p.get_sdp_media_line()
    assert "video" in ml.lower(), f"Expected 'video' in media line: {ml}"
    assert "26" in ml, f"Expected PT 26 in media line: {ml}"
    attrs = p.get_sdp_media_attributes()
    assert "rtpmap" in attrs.lower(), f"Expected rtpmap in attrs: {attrs}"


# ---------------------------------------------------------------------------
# H264 Packetizer
# ---------------------------------------------------------------------------
def test_h264_packetizer_construction():
    cfg = espp.H264Packetizer.Config()
    cfg.payload_type = 96
    cfg.profile_level_id = "42C01E"
    cfg.packetization_mode = 1
    cfg.sps = [0x67, 0x42, 0xC0, 0x1E]
    cfg.pps = [0x68, 0xCE, 0x38, 0x80]
    p = espp.H264Packetizer(cfg)
    assert p.get_payload_type() == 96
    assert p.get_clock_rate() == 90000


def test_h264_packetizer_sdp():
    cfg = espp.H264Packetizer.Config()
    cfg.payload_type = 96
    cfg.profile_level_id = "42C01E"
    cfg.packetization_mode = 1
    cfg.sps = [0x67, 0x42, 0xC0, 0x1E]
    cfg.pps = [0x68, 0xCE, 0x38, 0x80]
    p = espp.H264Packetizer(cfg)
    ml = p.get_sdp_media_line()
    assert "96" in ml, f"Expected PT 96 in media line: {ml}"
    attrs = p.get_sdp_media_attributes()
    assert "H264" in attrs, f"Expected H264 in attrs: {attrs}"


def test_h264_packetizer_packetize():
    cfg = espp.H264Packetizer.Config()
    cfg.max_payload_size = 100
    cfg.payload_type = 96
    cfg.packetization_mode = 1
    p = espp.H264Packetizer(cfg)
    # Annex B access unit: start code + small NAL
    nal = bytes([0x00, 0x00, 0x00, 0x01, 0x65] + [0xAB] * 50)
    chunks = p.packetize(nal)
    assert len(chunks) >= 1, f"Expected at least 1 chunk, got {len(chunks)}"
    # Last chunk should have marker set
    assert chunks[-1].marker, "Last chunk should have marker bit set"


def test_h264_packetizer_fua_fragmentation():
    cfg = espp.H264Packetizer.Config()
    cfg.max_payload_size = 50
    cfg.payload_type = 96
    cfg.packetization_mode = 1
    p = espp.H264Packetizer(cfg)
    # Large NAL that should be fragmented into FU-A
    nal = bytes([0x00, 0x00, 0x00, 0x01, 0x65] + [0xCD] * 200)
    chunks = p.packetize(nal)
    assert len(chunks) > 1, f"Expected FU-A fragmentation (>1 chunk), got {len(chunks)}"
    assert chunks[-1].marker, "Last FU-A chunk should have marker"


# ---------------------------------------------------------------------------
# Generic Packetizer (audio)
# ---------------------------------------------------------------------------
def test_generic_packetizer_construction():
    cfg = espp.GenericPacketizer.Config()
    cfg.payload_type = 0
    cfg.clock_rate = 8000
    cfg.encoding_name = "PCMU"
    cfg.channels = 1
    cfg.media_type = espp.MediaType.audio
    p = espp.GenericPacketizer(cfg)
    assert p.get_payload_type() == 0
    assert p.get_clock_rate() == 8000


def test_generic_packetizer_sdp():
    cfg = espp.GenericPacketizer.Config()
    cfg.payload_type = 0
    cfg.clock_rate = 8000
    cfg.encoding_name = "PCMU"
    cfg.channels = 1
    cfg.media_type = espp.MediaType.audio
    p = espp.GenericPacketizer(cfg)
    ml = p.get_sdp_media_line()
    assert "audio" in ml.lower(), f"Expected 'audio' in media line: {ml}"
    attrs = p.get_sdp_media_attributes()
    assert "PCMU" in attrs, f"Expected PCMU in attrs: {attrs}"


def test_generic_packetizer_chunking():
    cfg = espp.GenericPacketizer.Config()
    cfg.max_payload_size = 100
    cfg.media_type = espp.MediaType.audio
    p = espp.GenericPacketizer(cfg)
    data = bytes([0x55] * 250)
    chunks = p.packetize(data)
    assert len(chunks) == 3, f"Expected 3 chunks for 250 bytes / 100 MTU, got {len(chunks)}"
    assert chunks[-1].marker, "Last chunk should have marker"
    assert not chunks[0].marker, "First chunk should not have marker"


# ---------------------------------------------------------------------------
# MJPEG Depacketizer
# ---------------------------------------------------------------------------
def test_mjpeg_depacketizer_construction():
    cfg = espp.MjpegDepacketizer.Config()
    d = espp.MjpegDepacketizer(cfg)
    # Just verify construction succeeds
    assert d is not None


# ---------------------------------------------------------------------------
# H264 Depacketizer
# ---------------------------------------------------------------------------
def test_h264_depacketizer_construction():
    cfg = espp.H264Depacketizer.Config()
    d = espp.H264Depacketizer(cfg)
    assert d is not None


# ---------------------------------------------------------------------------
# Generic Depacketizer
# ---------------------------------------------------------------------------
def test_generic_depacketizer_construction():
    cfg = espp.GenericDepacketizer.Config()
    d = espp.GenericDepacketizer(cfg)
    assert d is not None


# ---------------------------------------------------------------------------
# Config structs — verify all fields are accessible
# ---------------------------------------------------------------------------
def test_config_fields():
    # H264Packetizer::Config
    c = espp.H264Packetizer.Config()
    c.max_payload_size = 1000
    c.payload_type = 97
    c.profile_level_id = "640028"
    c.packetization_mode = 0
    c.sps = [1, 2, 3]
    c.pps = [4, 5, 6]
    assert c.max_payload_size == 1000
    assert c.payload_type == 97

    # GenericPacketizer::Config
    c2 = espp.GenericPacketizer.Config()
    c2.fmtp = "mode=30"
    c2.channels = 2
    c2.encoding_name = "opus"
    assert c2.fmtp == "mode=30"
    assert c2.channels == 2


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("=" * 60)
    print("RTSP Multi-Codec API Tests (Host-side)")
    print("=" * 60)

    print("\n--- MediaType enum ---")
    test("MediaType enum values", test_media_type_enum)

    print("\n--- RtpPayloadChunk ---")
    test("RtpPayloadChunk construction", test_rtp_payload_chunk)

    print("\n--- MJPEG Packetizer ---")
    test("Construction", test_mjpeg_packetizer_construction)
    test("SDP generation", test_mjpeg_packetizer_sdp)

    print("\n--- H264 Packetizer ---")
    test("Construction", test_h264_packetizer_construction)
    test("SDP generation", test_h264_packetizer_sdp)
    test("Packetize small NAL", test_h264_packetizer_packetize)
    test("FU-A fragmentation", test_h264_packetizer_fua_fragmentation)

    print("\n--- Generic Packetizer (Audio) ---")
    test("Construction", test_generic_packetizer_construction)
    test("SDP generation", test_generic_packetizer_sdp)
    test("MTU chunking", test_generic_packetizer_chunking)

    print("\n--- Depacketizers ---")
    test("MjpegDepacketizer construction", test_mjpeg_depacketizer_construction)
    test("H264Depacketizer construction", test_h264_depacketizer_construction)
    test("GenericDepacketizer construction", test_generic_depacketizer_construction)

    print("\n--- Config field access ---")
    test("Config struct fields", test_config_fields)

    print("\n" + "=" * 60)
    total = passed + failed
    print(f"Results: {passed}/{total} passed, {failed}/{total} failed")
    print("=" * 60)

    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
