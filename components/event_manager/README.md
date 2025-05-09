# Event Manager Component

[![Badge](https://components.espressif.com/components/espp/event_manager/badge.svg)](https://components.espressif.com/components/espp/event_manager)

The `EventManager` class enables anonymous publish/subscribe interactions
between different software components and contexts in the system. It provides a
singleton class which manages the registry of publishers and subscribers in the
system and provides loose coupling between them. To subscribe to events/data, a
component must register itself with the manager by calling `add_subscriber()` -
which will register a callback function associated with that component for the
event/topic provided. All callback functions for a given topic/event are called
from the same thread/context - a thread that is started and managed by the
EventManager. As noted in a few places, it is recommended to use a
(de-)serialization library such as espp::serialization / alpaca for transforming
data structures to/from `std::vector<uint8_t>` for publishing/subscribing.

## Example

The [example](./example) shows some basic usage for the `EventManager` which
allows asynchronous communications between multiple tasks following the
anonymous pub/sub interaction pattern - supporting one to one, one to many, many
to one, and many to many data and event dissemination.
