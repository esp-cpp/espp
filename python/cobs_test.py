"""
COBS Python Test Suite - Vector API Tests

This test suite demonstrates the COBS (Consistent Overhead Byte Stuffing) component
functionality through Python bindings using the vector-based APIs.

Available tests:
- Basic encoding/decoding with vector APIs
- Streaming encoder/decoder functionality
- Move semantics testing
- Thread safety verification
- Edge cases and error handling
"""

import sys
import time
import threading

from support_loader import espp

def test_basic_encoding_decoding():
    """Test basic COBS encoding and decoding with vector APIs"""
    print("=== Basic Encoding/Decoding Test ===")
    
    try:
        # Test data with zeros
        test_data = [0x01, 0x02, 0x00, 0x03, 0x04, 0x00, 0x05, 0x06]
        
        # Test encoder
        encoder = espp.CobsStreamEncoder()
        encoder.add_packet(test_data)
        
        # Get encoded data
        encoded_data = encoder.get_encoded_data()
        
        if len(encoded_data) == 0:
            print("✗ Test 1: FAIL - No encoded data produced")
            return False
        
        # Test decoder
        decoder = espp.CobsStreamDecoder()
        decoder.add_data(encoded_data)
        
        # Extract decoded packet
        decoded_packet = decoder.extract_packet()
        
        if decoded_packet is None:
            print("✗ Test 1: FAIL - No packet decoded")
            return False
        
        # Verify data matches
        if list(decoded_packet) == test_data:
            print("✓ Test 1: PASS - Basic encoding/decoding")
            return True
        else:
            print(f"✗ Test 1: FAIL - Data mismatch. Expected: {test_data}, Got: {list(decoded_packet)}")
            return False
            
    except Exception as e:
        print(f"✗ Test 1: FAIL - Basic encoding/decoding (exception: {e})")
        return False

def test_multiple_packets():
    """Test multiple packet encoding/decoding"""
    print("=== Multiple Packets Test ===")
    
    try:
        # Test data - multiple packets
        packets = [
            [0x11, 0x22, 0x00, 0x33],
            [0x44, 0x55, 0x00, 0x66, 0x77],
            [0x88, 0x00, 0x99, 0xAA, 0xBB, 0x00, 0xCC]
        ]
        
        # Encode all packets
        encoder = espp.CobsStreamEncoder()
        for packet in packets:
            encoder.add_packet(packet)
        
        # Get all encoded data
        all_encoded = encoder.extract_data(encoder.buffer_size())
        
        if len(all_encoded) == 0:
            print("✗ Test 2: FAIL - No encoded data produced")
            return False
        
        # Decode all packets
        decoder = espp.CobsStreamDecoder()
        decoder.add_data(all_encoded)
        
        # Extract all packets
        decoded_packets = []
        while True:
            packet = decoder.extract_packet()
            if packet is None:
                break
            decoded_packets.append(list(packet))
        
        # Verify packet count and content
        if len(decoded_packets) != len(packets):
            print(f"✗ Test 2: FAIL - Packet count mismatch. Expected: {len(packets)}, Got: {len(decoded_packets)}")
            return False
        
        for i, (original, decoded) in enumerate(zip(packets, decoded_packets)):
            if original != decoded:
                print(f"✗ Test 2: FAIL - Packet {i} mismatch. Expected: {original}, Got: {decoded}")
                return False
        
        print("✓ Test 2: PASS - Multiple packets encoding/decoding")
        return True
        
    except Exception as e:
        print(f"✗ Test 2: FAIL - Multiple packets (exception: {e})")
        return False

def test_empty_packet():
    """Test empty packet handling"""
    print("=== Empty Packet Test ===")
    
    try:
        # Test empty packet - should be ignored by encoder
        empty_packet = []
        
        encoder = espp.CobsStreamEncoder()
        encoder.add_packet(empty_packet)
        
        encoded_data = encoder.get_encoded_data()
        
        # Empty packets are ignored, so no encoded data should be produced
        if len(encoded_data) == 0:
            print("✓ Test 3: PASS - Empty packet handling (ignored by encoder)")
            return True
        else:
            print(f"✗ Test 3: FAIL - Empty packet should be ignored, but produced {len(encoded_data)} bytes")
            return False
            
    except Exception as e:
        print(f"✗ Test 3: FAIL - Empty packet (exception: {e})")
        return False

def test_large_packet():
    """Test large packet encoding/decoding"""
    print("=== Large Packet Test ===")
    
    try:
        # Create large packet (1000 bytes with pattern)
        large_packet = [(i % 201) for i in range(1000)]
        
        encoder = espp.CobsStreamEncoder()
        encoder.add_packet(large_packet)
        
        encoded_data = encoder.get_encoded_data()
        
        if len(encoded_data) == 0:
            print("✗ Test 4: FAIL - Large packet produced no encoded data")
            return False
        
        # Decode large packet
        decoder = espp.CobsStreamDecoder()
        decoder.add_data(encoded_data)
        
        decoded_packet = decoder.extract_packet()
        
        if decoded_packet is None:
            print("✗ Test 4: FAIL - Large packet failed to decode")
            return False
        
        if len(decoded_packet) == len(large_packet):
            print("✓ Test 4: PASS - Large packet encoding/decoding")
            return True
        else:
            print(f"✗ Test 4: FAIL - Large packet size mismatch. Expected: {len(large_packet)}, Got: {len(decoded_packet)}")
            return False
            
    except Exception as e:
        print(f"✗ Test 4: FAIL - Large packet (exception: {e})")
        return False

def test_fragmented_data():
    """Test fragmented data handling"""
    print("=== Fragmented Data Test ===")
    
    try:
        # Create test packet
        test_packet = [0x11, 0x22, 0x00, 0x33, 0x44, 0x00, 0x55, 0x66, 0x77]
        
        # Encode packet
        encoder = espp.CobsStreamEncoder()
        encoder.add_packet(test_packet)
        encoded_data = encoder.get_encoded_data()
        
        # Simulate fragmented reception
        decoder = espp.CobsStreamDecoder()
        
        # Add data in small chunks
        chunk_size = 3
        for i in range(0, len(encoded_data), chunk_size):
            chunk = encoded_data[i:i + chunk_size]
            decoder.add_data(chunk)
        
        # Try to extract packet
        decoded_packet = decoder.extract_packet()
        
        if decoded_packet is None:
            print("✗ Test 5: FAIL - Fragmented data failed to decode")
            return False
        
        if list(decoded_packet) == test_packet:
            print("✓ Test 5: PASS - Fragmented data handling")
            return True
        else:
            print(f"✗ Test 5: FAIL - Fragmented data mismatch. Expected: {test_packet}, Got: {list(decoded_packet)}")
            return False
            
    except Exception as e:
        print(f"✗ Test 5: FAIL - Fragmented data (exception: {e})")
        return False

def test_buffer_management():
    """Test buffer management methods"""
    print("=== Buffer Management Test ===")
    
    try:
        # Test encoder buffer management
        encoder = espp.CobsStreamEncoder()
        
        # Add some data
        test_data = [0x01, 0x02, 0x00, 0x03]
        encoder.add_packet(test_data)
        
        # Check buffer size
        if encoder.buffer_size() == 0:
            print("✗ Test 6: FAIL - Encoder buffer should have data")
            return False
        
        # Clear buffer
        encoder.clear()
        
        if encoder.buffer_size() != 0:
            print("✗ Test 6: FAIL - Encoder buffer not cleared")
            return False
        
        # Test decoder buffer management
        decoder = espp.CobsStreamDecoder()
        
        # Add some data
        encoded_data = [0x01, 0x02, 0x03, 0x00]  # Simple encoded packet
        decoder.add_data(encoded_data)
        
        # Check buffer size
        if decoder.buffer_size() == 0:
            print("✗ Test 6: FAIL - Decoder buffer should have data")
            return False
        
        # Check remaining data
        remaining = decoder.remaining_data()
        if len(remaining) == 0:
            print("✗ Test 6: FAIL - Decoder should have remaining data")
            return False
        
        # Clear buffer
        decoder.clear()
        
        if decoder.buffer_size() != 0:
            print("✗ Test 6: FAIL - Decoder buffer not cleared")
            return False
        
        print("✓ Test 6: PASS - Buffer management")
        return True
        
    except Exception as e:
        print(f"✗ Test 6: FAIL - Buffer management (exception: {e})")
        return False

def test_thread_safety():
    """Test thread safety with concurrent access"""
    print("=== Thread Safety Test ===")
    
    try:
        # Test encoder thread safety
        encoder = espp.CobsStreamEncoder()
        results = []
        errors = []
        
        def encoder_worker(worker_id):
            try:
                for i in range(5):
                    # Add packet
                    test_data = [worker_id, i, 0x00, 0xFF]
                    encoder.add_packet(test_data)
                    
                    # Check buffer size
                    size = encoder.buffer_size()
                    
                    # Extract some data
                    extracted = encoder.extract_data(10)
                    
                    results.append((worker_id, i, size, len(extracted)))
                    time.sleep(0.001)
            except Exception as e:
                errors.append(f"Encoder Worker {worker_id}: {e}")
        
        # Create multiple threads
        threads = []
        for i in range(3):
            thread = threading.Thread(target=encoder_worker, args=(i,))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads
        for thread in threads:
            thread.join()
        
        if errors:
            print(f"✗ Test 7: FAIL - Encoder thread safety (errors: {errors})")
            return False
        
        if len(results) == 15:  # 3 threads * 5 iterations
            print("✓ Test 7: PASS - Thread safety")
            return True
        else:
            print(f"✗ Test 7: FAIL - Thread safety (expected 15 results, got {len(results)})")
            return False
            
    except Exception as e:
        print(f"✗ Test 7: FAIL - Thread safety (exception: {e})")
        return False

def test_method_availability():
    """Test that all expected methods are available"""
    print("=== Method Availability Test ===")
    
    try:
        encoder = espp.CobsStreamEncoder()
        decoder = espp.CobsStreamDecoder()
        
        # Test encoder methods
        encoder_methods = ['add_packet', 'get_encoded_data', 'extract_data', 'buffer_size', 'clear']
        for method in encoder_methods:
            if not hasattr(encoder, method):
                print(f"✗ Test 8: FAIL - Encoder missing {method}")
                return False
        
        # Test decoder methods
        decoder_methods = ['add_data', 'extract_packet', 'remaining_data', 'buffer_size', 'clear']
        for method in decoder_methods:
            if not hasattr(decoder, method):
                print(f"✗ Test 8: FAIL - Decoder missing {method}")
                return False
        
        print("✓ Test 8: PASS - Method availability")
        return True
        
    except Exception as e:
        print(f"✗ Test 8: FAIL - Method availability (exception: {e})")
        return False

def main():
    """Run all COBS vector API tests"""
    print("COBS Python Test Suite - Vector API Tests")
    print("=========================================")
    print("Testing COBS functionality with vector-based APIs")
    print()
    
    # Define which tests to run (can be enabled/disabled independently)
    tests = [
        ("Basic Encoding/Decoding", test_basic_encoding_decoding, True),
        ("Multiple Packets", test_multiple_packets, True),
        ("Empty Packet", test_empty_packet, True),
        ("Large Packet", test_large_packet, True),
        ("Fragmented Data", test_fragmented_data, True),
        ("Buffer Management", test_buffer_management, True),
        ("Thread Safety", test_thread_safety, True),
        ("Method Availability", test_method_availability, True),
    ]
    
    passed = 0
    total = 0
    
    for test_name, test_func, enabled in tests:
        if enabled:
            print(f"\n--- {test_name} ---")
            total += 1
            if test_func():
                passed += 1
    
    print(f"\n============================")
    print(f"Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("✓ All vector API tests completed successfully!")
        return 0
    else:
        print("✗ Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())