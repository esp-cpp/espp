# St25dv I2C Dynamic Near Field Communications (NFC) Tag Component

The `St25dv` provides a utility class for NFC using the ST25DV Dynamic NFC /
RFID tag. The tag includes 4Kb, 16Kb, or 64Kb EEPROM along with support for fast
transfer mode - which enables bi-directional communications with other NFC/RFID
enabled devices such as phones. This chip stores in its EEPROM NFC T5T tag
records - which include a CC header (that this class maintains) followed by
serialized NDEF records.

## Example

The [example](./example) shows how to configure a `espp::St25dv` class to
communicate with an ST25DV reprogrammable NFC tag.

