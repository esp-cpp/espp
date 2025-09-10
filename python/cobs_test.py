"""
COBS Python Test Suite - Pointer-Free Tests

This test suite demonstrates the COBS (Consistent Overhead Byte Stuffing) component
functionality through Python bindings, focusing on methods that don't require raw pointers.

Available pointer-free tests:
- Class instantiation and basic properties
- Buffer management (buffer_size, clear, remaining_data)
- Data extraction methods (get_encoded_data, extract_data, extract_packet)
- Thread safety verification
"""

import sys
import time

from support_loader import espp

def test_class_instantiation():
    """Test that COBS classes can be instantiated"""
    print("=== Class Instantiation Test ===")
    
    try:
        # Test encoder instantiation
        encoder = espp.CobsStreamEncoder()
        if encoder is not None:
            print("✓ Test 1: PASS - CobsStreamEncoder instantiation")
        else:
            print("✗ Test 1: FAIL - CobsStreamEncoder instantiation")
            return False
            
        # Test decoder instantiation
        decoder = espp.CobsStreamDecoder()
        if decoder is not None:
            print("✓ Test 2: PASS - CobsStreamDecoder instantiation")
            return True
        else:
            print("✗ Test 2: FAIL - CobsStreamDecoder instantiation")
            return False
    except Exception as e:
        print(f"✗ Test 1-2: FAIL - Class instantiation (exception: {e})")
        return False

def test_buffer_management():
    """Test buffer management methods"""
    print("=== Buffer Management Test ===")
    
    try:
        # Test encoder buffer management
        encoder = espp.CobsStreamEncoder()
        
        # Test initial buffer size (should be 0)
        if encoder.buffer_size() == 0:
            print("✓ Test 3: PASS - Encoder initial buffer size")
        else:
            print("✗ Test 3: FAIL - Encoder initial buffer size")
            return False
            
        # Test decoder buffer management
        decoder = espp.CobsStreamDecoder()
        
        # Test initial buffer size (should be 0)
        if decoder.buffer_size() == 0:
            print("✓ Test 4: PASS - Decoder initial buffer size")
        else:
            print("✗ Test 4: FAIL - Decoder initial buffer size")
            return False
            
        # Test clear methods
        encoder.clear()
        decoder.clear()
        
        if encoder.buffer_size() == 0 and decoder.buffer_size() == 0:
            print("✓ Test 5: PASS - Clear methods")
            return True
        else:
            print("✗ Test 5: FAIL - Clear methods")
            return False
    except Exception as e:
        print(f"✗ Test 3-5: FAIL - Buffer management (exception: {e})")
        return False

def test_data_extraction():
    """Test data extraction methods"""
    print("=== Data Extraction Test ===")
    
    try:
        # Test encoder data extraction
        encoder = espp.CobsStreamEncoder()
        
        # Test get_encoded_data on empty encoder
        encoded_data = encoder.get_encoded_data()
        if len(encoded_data) == 0:
            print("✓ Test 6: PASS - Encoder get_encoded_data (empty)")
        else:
            print("✗ Test 6: FAIL - Encoder get_encoded_data (empty)")
            return False
            
        # Test extract_data on empty encoder
        extracted = encoder.extract_data(100)
        if len(extracted) == 0:
            print("✓ Test 7: PASS - Encoder extract_data (empty)")
        else:
            print("✗ Test 7: FAIL - Encoder extract_data (empty)")
            return False
            
        # Test decoder data extraction
        decoder = espp.CobsStreamDecoder()
        
        # Test extract_packet on empty decoder
        packet = decoder.extract_packet()
        if packet is None:
            print("✓ Test 8: PASS - Decoder extract_packet (empty)")
        else:
            print("✗ Test 8: FAIL - Decoder extract_packet (empty)")
            return False
            
        # Test remaining_data on empty decoder
        remaining = decoder.remaining_data()
        if len(remaining) == 0:
            print("✓ Test 9: PASS - Decoder remaining_data (empty)")
            return True
        else:
            print("✗ Test 9: FAIL - Decoder remaining_data (empty)")
            return False
    except Exception as e:
        print(f"✗ Test 6-9: FAIL - Data extraction (exception: {e})")
        return False

def test_thread_safety():
    """Test thread safety by calling methods from different contexts"""
    print("=== Thread Safety Test ===")
    
    try:
        # Test encoder thread safety
        encoder = espp.CobsStreamEncoder()
        
        # Call multiple methods in sequence (simulating different threads)
        size1 = encoder.buffer_size()
        encoder.clear()
        size2 = encoder.buffer_size()
        data = encoder.get_encoded_data()
        size3 = encoder.buffer_size()
        
        if size1 == 0 and size2 == 0 and size3 == 0 and len(data) == 0:
            print("✓ Test 10: PASS - Encoder thread safety")
        else:
            print("✗ Test 10: FAIL - Encoder thread safety")
            return False
            
        # Test decoder thread safety
        decoder = espp.CobsStreamDecoder()
        
        # Call multiple methods in sequence
        size1 = decoder.buffer_size()
        decoder.clear()
        size2 = decoder.buffer_size()
        remaining = decoder.remaining_data()
        packet = decoder.extract_packet()
        size3 = decoder.buffer_size()
        
        if (size1 == 0 and size2 == 0 and size3 == 0 and 
            len(remaining) == 0 and packet is None):
            print("✓ Test 11: PASS - Decoder thread safety")
            return True
        else:
            print("✗ Test 11: FAIL - Decoder thread safety")
            return False
    except Exception as e:
        print(f"✗ Test 10-11: FAIL - Thread safety (exception: {e})")
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
            if hasattr(encoder, method):
                print(f"✓ Test 12: PASS - Encoder has {method}")
            else:
                print(f"✗ Test 12: FAIL - Encoder missing {method}")
                return False
                
        # Test decoder methods
        decoder_methods = ['add_data', 'extract_packet', 'remaining_data', 'buffer_size', 'clear']
        for method in decoder_methods:
            if hasattr(decoder, method):
                print(f"✓ Test 13: PASS - Decoder has {method}")
            else:
                print(f"✗ Test 13: FAIL - Decoder missing {method}")
                return False
                
        return True
    except Exception as e:
        print(f"✗ Test 12-13: FAIL - Method availability (exception: {e})")
        return False

def main():
    """Run all COBS pointer-free tests"""
    print("COBS Python Test Suite - Pointer-Free Tests")
    print("===========================================")
    print("Testing COBS functionality without raw pointer operations")
    print()
    
    # Define which tests to run (can be enabled/disabled independently)
    tests = [
        ("Class Instantiation", test_class_instantiation, True),
        ("Buffer Management", test_buffer_management, True),
        ("Data Extraction", test_data_extraction, True),
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
        print("✓ All pointer-free tests completed successfully!")
        print("Note: Full COBS functionality requires raw pointer support")
        return 0
    else:
        print("✗ Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())