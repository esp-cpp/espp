#include "gfps.hpp"

// Returns true if right earbud is active
bool nearby_platform_GetEarbudRightStatus() {
  // TODO: implement
  return false;
}

// Returns true if left earbud is active
bool nearby_platform_GetEarbudLeftStatus() {
  // TODO: implement
  return false;
}

// Returns one of NEARBY_PLATFORM_CONNECTION_STATE_* values
// Call |nearby_platform_AudioCallbacks::on_state_change| when this state
// changes.
unsigned int nearby_platform_GetAudioConnectionState() {
  // TODO: implement
  return NEARBY_PLATFORM_CONNECTION_STATE_NO_CONNECTION;
}

// Returns true if the device is on head (or in ear).
// Call |nearby_platform_AudioCallbacks::on_state_change| when on-head state
// changes.
bool nearby_platform_OnHead() {
  // TODO: implement
  return false;
}

// Returns true if the device can accept another audio connection without
// dropping any of the existing connections.
// Call |nearby_platform_AudioCallbacks::on_state_change| when this state
// changes.
bool nearby_platform_CanAcceptConnection() {
  // TODO: implement
  return false;
}

// When the device is in focus mode, connection switching is not allowed
// Call |nearby_platform_AudioCallbacks::on_state_change| when this state
// changes.
bool nearby_platform_InFocusMode() {
  // TODO: implement
  return false;
}

// Returns true if the current connection is auto-recconnected, meaning it is
// not connected by the user. For multi-point connections, returns true if any
// of the existing connections is auto-reconnected.
// Call |nearby_platform_AudioCallbacks::on_state_change| when this state
// changes.
bool nearby_platform_AutoReconnected() {
  // TODO: implement
  return false;
}

// Sets a bit in the |bitmap| for every connected peer. The bit stays cleared
// for bonded but not connected peers. The order change is acceptable if it is
// unavoidable, e.g. when users factory reset the headset or when the bonded
// device count reaches the upper limit.
// |length| is the |bitmap| length on input in bytes and used space on output.
// For example, if there are 5 bonded devices, then |length| should be set to 1.
// Call |nearby_platform_AudioCallbacks::on_state_change| when this state
// changes.
void nearby_platform_GetConnectionBitmap(uint8_t *, size_t *) {
  // TODO: implement
}

// Returns true is SASS state in On
bool nearby_platform_IsSassOn() {
  // TODO: implement
  return false;
}

// Returns true if the device supports multipoint and it can be switched between
// on and off
bool nearby_platform_IsMultipointConfigurable() {
  // TODO: implement
  return false;
}

// Returns true is multipoint in On
bool nearby_platform_IsMultipointOn() {
  // TODO: implement
  return false;
}

// Returns true if the device supports OHD (even if it's turned off at the
// moment)
bool nearby_platform_IsOnHeadDetectionSupported() {
  // TODO: implement
  return false;
}

// Returns true if OHD is supported and enabled
bool nearby_platform_IsOnHeadDetectionEnabled() {
  // TODO: implement
  return false;
}

// Enables or disables multipoint
// When |enable| is false, the device should keep the connection with
// |peer_address| and disconnect other connections (if any)
nearby_platform_status nearby_platform_SetMultipoint(uint64_t peer_address, bool enable) {
  // TODO: implement
  return kNearbyStatusOK;
}

// Sets multipoint switching preference flags
nearby_platform_status nearby_platform_SetSwitchingPreference(uint8_t flags) {
  // TODO: implement
  return kNearbyStatusOK;
}

// Gets switching preference flags
uint8_t nearby_platform_GetSwitchingPreference() {
  // TODO: implement
  return 0;
}

// Switches active audio source (to a connected device). If the flags indicate a
// switch to |peer_address| device but |peer_address| is already the active
// device, then return kNearbyStatusRedundantAction.
// If the flags indicate a switch to another device, then
// |preferred_audio_source| is the address of the next, preferred audio source.
// |preferred_audio_source| is 0 if Nearby SDK cannot figure out what the next
// audio source should be. It may happen if they are no other connected Seekers.
nearby_platform_status nearby_platform_SwitchActiveAudioSource(uint64_t peer_address, uint8_t flags,
                                                               uint64_t preferred_audio_source) {
  // TODO: implement
  return kNearbyStatusOK;
}

// Switches back to a disconnected audio source.
// |peer_address| is the address of the seeker who sent this command, *not* the
// address of the audio source that the device should switch to. The device
// should connect to the previous, currently disconnected, audio source and
// disconnect from |peer_address|. Ideally, the device should disconnect from
// |peer_address| after returning from this function. This would give Nearby SDK
// a chance to send an ACK message to the seeker.
nearby_platform_status nearby_platform_SwitchBackAudioSource(uint64_t peer_address, uint8_t flags) {
  // TODO: implement
  return kNearbyStatusOK;
}

// Notifies the platform if the connection was initiated by SASS. SASS Providers
// may need to know if the connection switching is triggered by SASS to have
// different reactions, e.g. disable earcons for SASS events. The Seeker sends a
// message to notify the Provider that this connection was a SASS initiated
// connection.
nearby_platform_status nearby_platform_NotifySassInitiatedConnection(uint64_t peer_address,
                                                                     uint8_t flags) {
  // TODO: implement
  return kNearbyStatusOK;
}

// Sets drop connection target
// On multipoint headphones, if the preferred connection to be dropped is not
// the least recently used one, SASS Seekers can tell the Provider which device
// to be dropped by using the message below.
nearby_platform_status nearby_platform_SetDropConnectionTarget(uint64_t peer_address,
                                                               uint8_t flags) {
  // TODO: implement
  return kNearbyStatusOK;
}

// Returns the BT address of the active audio source
// Call |nearby_platform_AudioCallbacks::on_state_change| when active audio
// source changes.
uint64_t nearby_platform_GetActiveAudioSource() {
  // TODO: implement
  return 0;
}

// Initializes Audio module
nearby_platform_status
nearby_platform_AudioInit(const nearby_platform_AudioCallbacks *audio_interface) {
  // TODO: implement
  return kNearbyStatusOK;
}
