#ifdef _WIN32
#include <iostream>
#include <string>
#include <WinSock2.h>
#include <WS2tcpip.h>
typedef SSIZE_T ssize_t;

#define socket_recv(socket, buffer, length) recv(socket, buffer, length, 0)
#define socket_send(socket, buffer, length) send(socket, buffer, length, 0)
#define socket_close closesocket

#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#define socket_recv(socket, buffer, length) read(socket, buffer, length)
#define socket_send(socket, buffer, length) write(socket, buffer, length)
#define socket_close close

#endif

#ifdef _WIN32

void LogWSAErrorStr(int error)
{
    LPWSTR err = NULL;

    DWORD len = FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        error,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPWSTR)&err, 0, NULL);

    std::wcerr << "\tError " << error << ": " << std::wstring(err) << std::endl;

    LocalFree(err);
}


#endif // _WIN32