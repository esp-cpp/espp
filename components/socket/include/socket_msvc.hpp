#ifdef _MSC_VER
extern "C" {
// if we don't define NOMINMAX, windows.h will define min and max as macros
// which will conflict with std::min and std::max
#define NOMINMAX                                                                                   \
// /* See http://stackoverflow.com/questions/12765743/getaddrinfo-on-win32 */
// #ifndef _WIN32_WINNT
// #define _WIN32_WINNT 0x0501 /* Windows XP. */
// #endif
#define WIN32_LEAN_AND_MEAN
#include <WS2tcpip.h>
#include <WinSock2.h>
#include <Windows.h>
#include <io.h>
}
#endif
