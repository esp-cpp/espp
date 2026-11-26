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

## Threads and callbacks

Each topic's callbacks run one after another on that topic's thread, in the
order the subscribers were added, and data published on a topic is delivered
in the order it was published. One slow callback delays every other subscriber
of the same topic (but not other topics).

Every function is thread safe and can be called from any thread, including from
inside a subscriber callback: a callback can publish, subscribe, or remove its
own subscription. When `remove_subscriber()` returns, that callback is not
running and will not be called again, so an object can call it from its
destructor and then be destroyed safely. To guarantee that,
`remove_subscriber()` (and `clear()`) wait for the callback they remove if it is
running on another thread, so don't call them while holding a lock that the
callback takes. They can't wait in two cases, which would deadlock: when called
from that callback itself, and when that callback is waiting in turn for the
caller (callbacks of two topics removing each other's subscribers).

The wait runs at the topic task's priority, which is 0 unless the
`task_config` passed to `add_subscriber()` sets one, so a high priority task
removing a subscriber can be held up by a low priority topic. `add_subscriber()`
returns false while `clear()` runs. Data still queued for a topic when its last
subscriber leaves is dropped.

## Example

The [example](./example) shows some basic usage for the `EventManager` which
allows asynchronous communications between multiple tasks following the
anonymous pub/sub interaction pattern - supporting one to one, one to many, many
to one, and many to many data and event dissemination.
