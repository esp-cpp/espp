"""
COBS Demo - ESPP COBS Functionality

A demonstration of ESPP COBS functionality with native Python data types.
Shows essential features including cross-library compatibility.

Key Features:
- ESPP encoding/decoding examples
- Cross-library compatibility (cobs Python library)
- Design differences explanation
- Practical usage examples
"""

import sys
from cobs import cobs
from support_loader import espp

def simple_example():
    """Simple practical example"""
    print("\n=== Simple Example ===")
    
    # Create a message with zeros
    message = b'{"sensor": "temp", "value": 25.5}'
    print(f"Original: {message}")
    
    # Encode with ESPP
    encoder = espp.CobsStreamEncoder()
    encoder.add_packet(list(message))
    encoded = encoder.extract_data(encoder.buffer_size())
    
    print(f"Encoded:  {bytes(encoded).hex()}")
    
    # Decode with ESPP
    decoder = espp.CobsStreamDecoder()
    decoder.add_data(encoded)
    packet = decoder.extract_packet()
    
    if packet:
        decoded = bytes(packet)
        print(f"Decoded:  {decoded}")
        print(f"Match:    {decoded == message}")
    else:
        print("FAIL: No packet decoded")

def cross_library_example():
    """Example showing ESPP decoder with cobs Python library encoder"""
    print("\n=== Cross-Library Example ===")
    
    # Create a message with zeros
    message = b'Hello\x00World\x00Test'
    print(f"Original: {message}")
    
    # Encode with cobs Python library
    ref_encoded = cobs.encode(message)
    print(f"cobs encoded: {ref_encoded.hex()}")
    
    # Decode with ESPP (add delimiter since ESPP expects it)
    decoder = espp.CobsStreamDecoder()
    decoder.add_data(list(ref_encoded) + [0x00])  # Add missing delimiter
    
    packet = decoder.extract_packet()
    if packet:
        decoded = bytes(packet)
        print(f"ESPP decoded: {decoded}")
        print(f"Match: {decoded == message}")
    else:
        print("FAIL: No packet decoded")

def main():
    """Run the COBS demo"""
    print("COBS Demo - ESPP COBS Functionality")
    print("=" * 40)

    """Explain key design differences"""
    print("\n=== Design Differences ===")
    print("ESPP COBS vs Other Libraries:")
    print("- ESPP adds 0x00 delimiters automatically")
    print("- Other libraries may not add delimiters")
    print("- ESPP ignores empty packets for performance")
    print("- Other libraries may encode empty packets as a single 0x01 byte")
    
    
    # Simple example
    simple_example()
    
    # Cross-library example
    cross_library_example()
    
    print("\n" + "=" * 40)
    print("SUCCESS: COBS demo completed!")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
