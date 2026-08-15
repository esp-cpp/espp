#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/rtps/common/Locator.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>

namespace {

using namespace eprosima::fastdds::dds;

static constexpr uint32_t kDomainId = 0;
static constexpr uint16_t kSpdpMulticastPort = 7400;
static constexpr char kTypeName[] = "std_msgs::msg::String";

class RawStringPubSubType : public TopicDataType {
public:
  RawStringPubSubType() {
    set_name(kTypeName);
    max_serialized_type_size = 256;
    is_compute_key_provided = false;
  }

  ~RawStringPubSubType() override = default;

  bool serialize(const void *const data, eprosima::fastdds::rtps::SerializedPayload_t &payload,
                 DataRepresentationId_t /*rep*/) override {
    const std::string *str = static_cast<const std::string *>(data);
    const uint32_t slen = static_cast<uint32_t>(str->size()) + 1u;
    const uint32_t needed = 4u + 4u + slen; // encap header + CDR length prefix + string + null
    if (needed > payload.max_size) {
      return false;
    }
    // CDR_LE encapsulation header
    payload.data[0] = 0x00;
    payload.data[1] = 0x01;
    payload.data[2] = 0x00;
    payload.data[3] = 0x00;
    // CDR string: 4-byte LE length followed by string bytes and null terminator
    payload.data[4] = static_cast<uint8_t>(slen & 0xFF);
    payload.data[5] = static_cast<uint8_t>((slen >> 8) & 0xFF);
    payload.data[6] = static_cast<uint8_t>((slen >> 16) & 0xFF);
    payload.data[7] = static_cast<uint8_t>((slen >> 24) & 0xFF);
    std::memcpy(payload.data + 8, str->c_str(), slen);
    payload.length = needed;
    payload.encapsulation = CDR_LE;
    return true;
  }

  bool deserialize(eprosima::fastdds::rtps::SerializedPayload_t &payload, void *data) override {
    // payload.data includes the 4-byte CDR encapsulation header followed by the CDR string body
    if (payload.length < 9u) {
      return false;
    }
    auto *str = static_cast<std::string *>(data);
    // Skip encapsulation header, read CDR 4-byte LE string length
    const uint32_t slen = static_cast<uint32_t>(payload.data[4]) |
                          (static_cast<uint32_t>(payload.data[5]) << 8) |
                          (static_cast<uint32_t>(payload.data[6]) << 16) |
                          (static_cast<uint32_t>(payload.data[7]) << 24);
    if (slen == 0 || 8u + slen > payload.length) {
      return false;
    }
    str->assign(reinterpret_cast<const char *>(payload.data + 8), slen - 1);
    return true;
  }

  uint32_t calculate_serialized_size(const void *const data,
                                     DataRepresentationId_t /*rep*/) override {
    const std::string *str = static_cast<const std::string *>(data);
    return 4u + 4u + static_cast<uint32_t>(str->size()) + 1u;
  }

  bool compute_key(eprosima::fastdds::rtps::SerializedPayload_t & /*payload*/,
                   eprosima::fastdds::rtps::InstanceHandle_t & /*handle*/,
                   bool /*force_md5*/) override {
    return false;
  }

  bool compute_key(const void *const /*data*/,
                   eprosima::fastdds::rtps::InstanceHandle_t & /*handle*/,
                   bool /*force_md5*/) override {
    return false;
  }

  void *create_data() override { return new std::string(); }

  void delete_data(void *data) override { delete static_cast<std::string *>(data); }

  void register_type_object_representation() override {}

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
  bool is_bounded() const override { return false; }
#endif
#ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
  bool is_plain(DataRepresentationId_t) const override { return false; }
#endif
};

class StringListener : public DataReaderListener {
public:
  explicit StringListener(std::string label)
      : label_(std::move(label)) {}

  void on_data_available(DataReader *reader) override {
    std::string sample;
    SampleInfo info;
    while (reader->take_next_sample(&sample, &info) == RETCODE_OK) {
      if (info.valid_data) {
        std::cout << "[rx " << label_ << "] " << sample << std::endl;
      }
    }
  }

private:
  std::string label_;
};

std::atomic<bool> g_running{true};

void signal_handler(int) { g_running.store(false); }

} // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::string interface_ip = "192.168.4.2";
  int period_ms = 2000;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--interface-ip" && i + 1 < argc) {
      interface_ip = argv[++i];
    } else if (arg == "--period-ms" && i + 1 < argc) {
      period_ms = std::stoi(argv[++i]);
    } else {
      std::cerr << "Usage: " << argv[0] << " [--interface-ip <ip>] [--period-ms <ms>]" << std::endl;
      return 1;
    }
  }

  constexpr const char *pub_topic = "pc_to_mcu";
  constexpr const char *sub_topic = "mcu_to_pc";
  std::cout << "Publish: " << pub_topic << "\nSubscribe: " << sub_topic << std::endl;

  auto *factory = DomainParticipantFactory::get_instance();
  DomainParticipantQos qos;
  factory->get_default_participant_qos(qos);

  if (!interface_ip.empty()) {
    auto transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    transport->interfaceWhiteList.push_back(interface_ip);
    transport->sendBufferSize = 65536;
    transport->receiveBufferSize = 65536;
    qos.transport().use_builtin_transports = false;
    qos.transport().user_transports.push_back(transport);
  }

  eprosima::fastdds::rtps::Locator_t peer_mcast;
  peer_mcast.kind = LOCATOR_KIND_UDPv4;
  eprosima::fastdds::rtps::IPLocator::setIPv4(peer_mcast, "239.255.0.1");
  peer_mcast.port = kSpdpMulticastPort;
  qos.wire_protocol().builtin.initialPeersList.push_back(peer_mcast);

  auto *participant = factory->create_participant(kDomainId, qos);
  if (participant == nullptr) {
    std::cerr << "Failed to create participant" << std::endl;
    return 1;
  }

  TypeSupport type_support(new RawStringPubSubType());
  if (type_support.register_type(participant) != RETCODE_OK) {
    std::cerr << "Failed to register type" << std::endl;
    factory->delete_participant(participant);
    return 1;
  }

  auto *topic_pub =
      participant->create_topic(pub_topic, type_support.get_type_name(), TOPIC_QOS_DEFAULT);
  auto *topic_sub =
      participant->create_topic(sub_topic, type_support.get_type_name(), TOPIC_QOS_DEFAULT);
  if (topic_pub == nullptr || topic_sub == nullptr) {
    std::cerr << "Failed to create topics" << std::endl;
    factory->delete_participant(participant);
    return 1;
  }

  auto *publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
  auto *subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
  if (publisher == nullptr || subscriber == nullptr) {
    std::cerr << "Failed to create publisher/subscriber" << std::endl;
    participant->delete_topic(topic_pub);
    participant->delete_topic(topic_sub);
    factory->delete_participant(participant);
    return 1;
  }

  StringListener listener(sub_topic);
  auto *reader = subscriber->create_datareader(topic_sub, DATAREADER_QOS_DEFAULT, &listener);
  auto *writer = publisher->create_datawriter(topic_pub, DATAWRITER_QOS_DEFAULT);
  if (reader == nullptr || writer == nullptr) {
    std::cerr << "Failed to create reader/writer" << std::endl;
    participant->delete_subscriber(subscriber);
    participant->delete_publisher(publisher);
    participant->delete_topic(topic_pub);
    participant->delete_topic(topic_sub);
    factory->delete_participant(participant);
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  uint32_t counter = 0;
  while (g_running.load()) {
    std::string msg = "pc " + std::to_string(counter++);
    if (writer->write(&msg) == RETCODE_OK) {
      std::cout << "[tx] " << msg << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  subscriber->delete_datareader(reader);
  publisher->delete_datawriter(writer);
  participant->delete_subscriber(subscriber);
  participant->delete_publisher(publisher);
  participant->delete_topic(topic_pub);
  participant->delete_topic(topic_sub);
  factory->delete_participant(participant);
  return 0;
}
