/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Modifications Copyright (c) 2026 ATDev
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE

This file is part of embeddedRTPS.

Author: i11 - Embedded Software, RWTH Aachen University
*/
#include "rtps/discovery/TopicData.hpp"
#include "logger.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include <cstdio>
#include <cstring>

using rtps::TopicData;
using rtps::TopicDataCompressed;
using rtps::SMElement::ParameterId;

namespace {
espp::Logger s_topic_data_logger({.tag = "RtpsTopicData", .level = espp::Logger::Verbosity::WARN});
}

bool TopicData::isDisposedFlagSet() const { return statusInfoValid && ((statusInfo & 0b1)); }

bool TopicData::isUnregisteredFlagSet() const {
  return statusInfoValid && ((statusInfo & (0b1 << 1)) != 0);
}

bool TopicData::matchesTopicOf(const TopicData &other) {
  return strcmp(this->topicName, other.topicName) == 0 &&
         strcmp(this->typeName, other.typeName) == 0;
}

bool TopicData::readFromBuffer(std::span<const uint8_t> data) {
  CdrReader buffer(asBytes(data.data(), data.size()));

  // Reset valid flags, as the respective parameters are optional
  statusInfoValid = false;
  entityIdFromKeyHashValid = false;

  while (buffer.remaining() >= 4) {
    const auto pidRaw = buffer.read<uint16_t>();
    const auto lengthRaw = buffer.read<uint16_t>();
    if (!pidRaw || !lengthRaw) {
      s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
      return false;
    }
    const auto pid = static_cast<ParameterId>(*pidRaw);
    const uint16_t length = *lengthRaw;
    FullLengthLocator uLoc;

    if (buffer.remaining() < length) {
      return false;
    }

    switch (pid) {
    case ParameterId::PID_ENDPOINT_GUID: {
      if (!readBytes(buffer, endpointGuid.prefix.id.data(), endpointGuid.prefix.id.size()) ||
          !readBytes(buffer, endpointGuid.entityId.entityKey.data(),
                     endpointGuid.entityId.entityKey.size())) {
        s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
        return false;
      }
      const auto kind = buffer.read<uint8_t>();
      if (!kind) {
        s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
        return false;
      }
      endpointGuid.entityId.entityKind = static_cast<EntityKind_t>(*kind);
      break;
    }
    case ParameterId::PID_RELIABILITY: {
      const auto kind = buffer.read<uint32_t>();
      if (!kind) {
        s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
        return false;
      }
      reliabilityKind = static_cast<ReliabilityKind_t>(*kind);
      skipBytes(buffer, 8);
      // TODO Skip 8 bytes. don't know what they are yet
      break;
    }
    case ParameterId::PID_SENTINEL:
      return true;
    case ParameterId::PID_TOPIC_NAME: {
      const auto topicNameLength = buffer.read<uint32_t>();
      if (!topicNameLength) {
        s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
        return false;
      }
      if (*topicNameLength > Config::MAX_TOPICNAME_LENGTH) {
        s_topic_data_logger.warn("Topic name length {} exceeds maximum allowed length {}",
                                 *topicNameLength, Config::MAX_TOPICNAME_LENGTH);
        return false;
      }
      if (!readBytes(buffer, reinterpret_cast<uint8_t *>(topicName), *topicNameLength)) {
        s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
        return false;
      }
      break;
    }
    case ParameterId::PID_TYPE_NAME: {
      const auto typeNameLength = buffer.read<uint32_t>();
      if (!typeNameLength) {
        s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
        return false;
      }
      if (*typeNameLength > Config::MAX_TYPENAME_LENGTH) {
        s_topic_data_logger.warn("Type name length {} exceeds maximum allowed length {}",
                                 *typeNameLength, Config::MAX_TYPENAME_LENGTH);
        return false;
      }
      if (!readBytes(buffer, reinterpret_cast<uint8_t *>(typeName), *typeNameLength)) {
        s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
        return false;
      }
      break;
    }
    case ParameterId::PID_UNICAST_LOCATOR:
      uLoc.readFromBuffer(buffer);
      // Accept valid UDPv4 locators even if subnet detection is temporarily
      // unavailable (e.g. early startup) to avoid keeping placeholder defaults.
      if (uLoc.kind == LocatorKind_t::LOCATOR_KIND_UDPv4) {
        unicastLocator = uLoc;
        const auto a0 = static_cast<unsigned int>(uLoc.address[12]);
        const auto a1 = static_cast<unsigned int>(uLoc.address[13]);
        const auto a2 = static_cast<unsigned int>(uLoc.address[14]);
        const auto a3 = static_cast<unsigned int>(uLoc.address[15]);
        const auto port = static_cast<unsigned long>(uLoc.port);
        s_topic_data_logger.warn("Received unicast locator: {}.{}.{}.{}:{}", a0, a1, a2, a3, port);
      } else {
        // print warning and the invalid locator for debugging
        s_topic_data_logger.warn("Warning: Received invalid unicast locator with kind {}",
                                 static_cast<int>(uLoc.kind));
      }
      break;
    case ParameterId::PID_MULTICAST_LOCATOR:
      multicastLocator.readFromBuffer(buffer);
      break;
    case ParameterId::PID_STATUS_INFO: {
      if (length == 4) {
        skipBytes(buffer, 3); // skip first 3 bytes of status info as they are
                              // reserved parameters
        const auto status = buffer.read<uint8_t>();
        if (!status) {
          s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
          return false;
        }
        statusInfo = *status;
        statusInfoValid = true;
      } else { // Ignore Status Info
        skipBytes(buffer, length);
      }
    } break;
    case ParameterId::PID_KEY_HASH: // only use case so far is deleting remote
                                    // endpoints
    {
      if (length == 16) {
        if (!readBytes(buffer, endpointGuid.prefix.id.data(), endpointGuid.prefix.id.size()) ||
            !readBytes(buffer, this->entityIdFromKeyHash.entityKey.data(),
                       this->entityIdFromKeyHash.entityKey.size())) {
          s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
          return false;
        }
        const auto kind = buffer.read<uint8_t>();
        if (!kind) {
          s_topic_data_logger.error("FAILED TO DESERIALIZE TOPIC DATA");
          return false;
        }
        this->entityIdFromKeyHash.entityKind = static_cast<EntityKind_t>(*kind);
        entityIdFromKeyHashValid = true;
      } else { // Ignore value
        skipBytes(buffer, length);
      }
    } break;
    default:
      skipBytes(buffer, length);
    }

    // Parameter-list elements are 4-byte aligned
    alignTo4(buffer);
  }
  return buffer.remaining() == 0;
}

bool TopicData::serializeInto(CdrWriter &writer) const {
  const uint16_t guidSize = sizeof(GuidPrefix_t::id) + 4;

#if SUPPRESS_UNICAST
  if (multicastLocator.kind != LocatorKind_t::LOCATOR_KIND_UDPv4) {
#endif
    writer.write<uint16_t>(ParameterId::PID_UNICAST_LOCATOR);
    writer.write<uint16_t>(sizeof(FullLengthLocator));
    writeBytes(writer, reinterpret_cast<const uint8_t *>(&unicastLocator),
               sizeof(FullLengthLocator));
#if SUPPRESS_UNICAST
  }
#endif

  if (multicastLocator.kind == LocatorKind_t::LOCATOR_KIND_UDPv4) {
    writer.write<uint16_t>(ParameterId::PID_MULTICAST_LOCATOR);
    writer.write<uint16_t>(sizeof(FullLengthLocator));
    writeBytes(writer, reinterpret_cast<const uint8_t *>(&multicastLocator),
               sizeof(FullLengthLocator));
  }

  // It's a 32 bit instead of 16 because it seems like the field is padded.
  const auto lenTopicName = static_cast<uint32_t>(strlen(topicName) + 1); // + \0
  uint16_t topicAlignment = 0;
  if (lenTopicName % 4 != 0) {
    topicAlignment = static_cast<uint8_t>(4 - (lenTopicName % 4));
  }
  const auto totalLengthTopicNameField =
      static_cast<uint16_t>(sizeof(lenTopicName) + lenTopicName + topicAlignment);
  writer.write<uint16_t>(ParameterId::PID_TOPIC_NAME);
  writer.write<uint16_t>(totalLengthTopicNameField);
  writer.write<uint32_t>(lenTopicName);
  writeBytes(writer, reinterpret_cast<const uint8_t *>(topicName), lenTopicName);
  writer.align(4);

  // It's a 32 bit instead of 16 because it seems like the field is padded.
  const auto lenTypeName = static_cast<uint32_t>(strlen(typeName) + 1); // + \0
  uint16_t typeAlignment = 0;
  if (lenTypeName % 4 != 0) {
    typeAlignment = static_cast<uint8_t>(4 - (lenTypeName % 4));
  }
  const auto totalLengthTypeNameField =
      static_cast<uint16_t>(sizeof(lenTypeName) + lenTypeName + typeAlignment);

  writer.write<uint16_t>(ParameterId::PID_TYPE_NAME);
  writer.write<uint16_t>(totalLengthTypeNameField);
  writer.write<uint32_t>(lenTypeName);
  writeBytes(writer, reinterpret_cast<const uint8_t *>(typeName), lenTypeName);
  writer.align(4);

  writer.write<uint16_t>(ParameterId::PID_KEY_HASH);
  writer.write<uint16_t>(guidSize);
  writeBytes(writer, endpointGuid.prefix.id.data(), endpointGuid.prefix.id.size());
  writeBytes(writer, endpointGuid.entityId.entityKey.data(),
             endpointGuid.entityId.entityKey.size());
  writer.write<uint8_t>(static_cast<uint8_t>(endpointGuid.entityId.entityKind));

  writer.write<uint16_t>(ParameterId::PID_ENDPOINT_GUID);
  writer.write<uint16_t>(guidSize);
  writeBytes(writer, endpointGuid.prefix.id.data(), endpointGuid.prefix.id.size());
  writeBytes(writer, endpointGuid.entityId.entityKey.data(),
             endpointGuid.entityId.entityKey.size());
  writer.write<uint8_t>(static_cast<uint8_t>(endpointGuid.entityId.entityKind));

  const uint8_t unidentifiedOffset = 8;
  writer.write<uint16_t>(ParameterId::PID_RELIABILITY);
  writer.write<uint16_t>(sizeof(ReliabilityKind_t) + unidentifiedOffset);
  writer.write<uint32_t>(static_cast<uint32_t>(reliabilityKind));
  writer.write<uint32_t>(0); // unidentified additional value
  writer.write<uint32_t>(0); // unidentified additional value

  writer.write<uint16_t>(ParameterId::PID_DURABILITY);
  writer.write<uint16_t>(sizeof(DurabilityKind_t));
  writer.write<uint32_t>(static_cast<uint32_t>(durabilityKind));

  writer.write<uint16_t>(ParameterId::PID_SENTINEL);
  writer.write<uint16_t>(0);

  return writer.ok();
}

bool TopicDataCompressed::matchesTopicOf(const TopicData &other) const {
  return (hashCharArray(other.topicName, sizeof(other.topicName)) == topicHash &&
          hashCharArray(other.typeName, sizeof(other.typeName)) == typeHash);
}
