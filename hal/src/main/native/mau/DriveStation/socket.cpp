#include "socket.hpp"

#ifndef OS_WIN
	#include <errno.h>
	#include <fcntl.h>
#endif

using namespace Toast::Net;
using namespace std;

int Socket::socket_init() {
    #ifdef OS_WIN
        WSADATA wsa_data;
        return WSAStartup(MAKEWORD(1, 1), &wsa_data);
    #else
        return 0;
    #endif
}

Socket::SOCKET Socket::socket_create() {
    Socket::SOCKET s;
    s = socket(AF_INET, SOCK_STREAM, 0);
    return s;
}

#ifndef OS_WIN
Socket::SOCKET Socket::socket_unix_create() {
	Socket::SOCKET s;
	s = socket(AF_UNIX, SOCK_STREAM, 0);
	return s;
}
#endif

Socket::SOCKET Socket::socket_udp_create() {
    Socket::SOCKET s;
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    return s;
}

Socket::SocketAddress::SocketAddress(string host, int port) {
	inaddr_struct.sin_addr.s_addr = inet_addr((char *)host.c_str());
	inaddr_struct.sin_family = AF_INET;
	inaddr_struct.sin_port = htons(port);
	inaddr_length = sizeof(inaddr_struct);
}

string Socket::SocketAddress::host() {
	return string(inet_ntoa(inaddr_struct.sin_addr));
}

int Socket::SocketAddress::port() {
	return ntohs(inaddr_struct.sin_port);
}

void Socket::SocketAddress::set_host(string host) {
	inaddr_struct.sin_addr.s_addr = inet_addr((char *)host.c_str());
	inaddr_struct.sin_family = AF_INET;
	inaddr_length = sizeof(inaddr_struct);
}

void Socket::SocketAddress::set_port(int port) {
	inaddr_struct.sin_port = htons(port);
	inaddr_length = sizeof(inaddr_struct);
}

Socket::SocketAddress Socket::socket_address(std::string host, int port) {
	struct sockaddr_in addr;
	addr.sin_addr.s_addr = inet_addr((char *)host.c_str());
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	return SocketAddress(addr);
}

int Socket::socket_connect(Socket::SOCKET s, string host, int port) {
    struct sockaddr_in host_addr;
    host_addr.sin_addr.s_addr = inet_addr((char *)host.c_str());
    host_addr.sin_family = AF_INET;
    host_addr.sin_port = htons(port);
    return connect(s, (struct sockaddr *)&host_addr, sizeof(host_addr));
}

int Socket::socket_nonblock(Socket::SOCKET s) {
#ifdef OS_WIN
	ULONG NonBlock = 1;
	return ioctlsocket(s, FIONBIO, &NonBlock);
#else
	int opts;
	opts = fcntl(s, F_GETFL);
	if (opts < 0) return opts;
	opts = (opts | O_NONBLOCK);
	return fcntl(s, F_SETFL, opts) < 0;
#endif
}

int Socket::socket_bind(Socket::SOCKET s, int port) {
	struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    
    return ::bind(s, (struct sockaddr *)&addr, sizeof(addr));
}

void Socket::socket_listen(Socket::SOCKET s, int backfill) {
    listen(s, backfill);
}

Socket::SOCKET Socket::socket_accept(SOCKET s, Socket::SocketAddress *addr) {
	#ifdef OS_WIN
		int *ptr = addr->raw_address_len_ptr();
	#else
		unsigned int *ptr = (unsigned int *)addr->raw_address_len_ptr();
	#endif
    return accept(s, (struct sockaddr *)addr->raw_address(), ptr);
}

int Socket::socket_quit() {
  #ifdef OS_WIN
    return WSACleanup();
  #else
    return 0;
  #endif
}

int Socket::socket_last_error() {
#ifdef OS_WIN
	return WSAGetLastError();
#else
	return errno;
#endif
}

int Socket::socket_close(Socket::SOCKET sock) {
  int status = 0;

  #ifdef OS_WIN
    status = shutdown(sock, SD_BOTH);
    if (status == 0) { status = closesocket(sock); }
  #else
    status = shutdown(sock, SHUT_RDWR);
    if (status == 0) { status = close(sock); }
  #endif

  return status;
}

string Socket::hostname_to_ip(string hostname) {
    struct hostent *he;
    struct in_addr **addr_list;
    int i;
         
    if ((he = gethostbyname((char *)hostname.c_str())) == NULL) {
        return string();
    }
 
    addr_list = (struct in_addr **) he->h_addr_list;
     
    for(i = 0; addr_list[i] != NULL; i++) {
        return string(inet_ntoa(*addr_list[i]));
    }
     
    return string();
}

int Socket::ClientSocket::connect() {
    #ifdef SO_NOSIGPIPE
        const int set = 1;
        setsockopt(_socket, SOL_SOCKET, SO_NOSIGPIPE, &set, sizeof(set));
    #endif
    return Socket::socket_connect(_socket, Socket::hostname_to_ip(host), port);
}

int Socket::ClientSocket::close() {
    return Socket::socket_close(_socket);
}

int Socket::ClientSocket::send(const char *buffer, size_t length, int flags) {
    return ::send(_socket, buffer, length, flags);
}

int Socket::ClientSocket::send(const char *buffer, size_t length) {
    #ifdef MSG_NOSIGNAL
        return Socket::ClientSocket::send(buffer, length, MSG_NOSIGNAL);
    #else
        return Socket::ClientSocket::send(buffer, length, 0);
    #endif
}

int Socket::ClientSocket::send(string message) {
    return Socket::ClientSocket::send(message.c_str(), message.length());
}

int Socket::ClientSocket::read(char *buf, size_t length) {
    #ifdef OS_WIN
        return ::recv(_socket, buf, length, 0);
    #else
        return ::read(_socket, buf, length);
    #endif
}

int Socket::ServerSocket::open() {

    int ret = Socket::socket_bind(_socket, port);
    if (ret != 0) {
        return ret;
    }
    Socket::socket_listen(_socket);
    return 0;
}

int Socket::ServerSocket::SetReuseAddress() {
    int enable = 1;
    return setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
}

int Socket::ServerSocket::close() {
    return Socket::socket_close(_socket);
}

Socket::ClientSocket Socket::ServerSocket::accept() {
	Socket::SocketAddress addr;
	Socket::SOCKET sid = Socket::socket_accept(_socket, &addr);

	ClientSocket socket(sid);
	socket.host = addr.host();
	socket.port = addr.port();
	return socket;
}

#ifndef OS_WIN
int Socket::UnixDomainClientSocket::connect() {
	struct sockaddr_un addr;
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	*addr.sun_path = '\0';
	memcpy(addr.sun_path+1, _path.c_str(), _path.length());

	return ::connect(_socket, (struct sockaddr*)&addr, sizeof(addr));
}

int Socket::UnixDomainClientSocket::close() {
	return Socket::socket_close(_socket);
}

int Socket::UnixDomainClientSocket::send(const char *buffer, size_t length, int flags) {
	return ::send(_socket, buffer, length, flags);
}

int Socket::UnixDomainClientSocket::send(const char *buffer, size_t length) {
#ifdef MSG_NOSIGNAL
	return Socket::UnixDomainClientSocket::send(buffer, length, MSG_NOSIGNAL);
#else
	return Socket::UnixDomainClientSocket::send(buffer, length, 0);
#endif
}

int Socket::UnixDomainClientSocket::send(string message) {
	return Socket::UnixDomainClientSocket::send(message.c_str(), message.length());
}

int Socket::UnixDomainClientSocket::read(char *buf, size_t length) {
	return ::read(_socket, buf, length);
}

int Socket::UnixDomainServerSocket::open() {
	struct sockaddr_un addr;
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	*addr.sun_path = '\0';
	memcpy(addr.sun_path+1, _path.c_str(), _path.length());

	int ret = ::bind(_socket, (struct sockaddr*)&addr, sizeof(addr));
	if (ret != 0) {
		return ret;
	}
	Socket::socket_listen(_socket);
	return 0;
}

int Socket::UnixDomainServerSocket::close() {
	::close(_socket);
	return 0;
}

Socket::UnixDomainClientSocket Socket::UnixDomainServerSocket::accept() {
	Socket::SOCKET sid = ::accept(_socket, NULL, NULL);

	Socket::UnixDomainClientSocket socket(sid);
	return socket;
}

#endif

int Socket::SelectiveServerSocket::prepare() {
	//int ret = Socket::socket_nonblock(_socket);
	int ret = 0;
	if (ret != 0) {
		return ret;
	}
	_highsock = _socket;
	memset((char *)_connectionlist, 0, sizeof(Socket::SOCKET) * _maxsize);
	return 0;
}

int Socket::SelectiveServerSocket::close() {
	return Socket::socket_close(_socket);
}

#define SOCKET_ACCEPT_TIMEOUT_MS 10

int Socket::SelectiveServerSocket::accept() {
	FD_ZERO(&_socks);
	FD_SET(_socket, &_socks);
	_highsock = _socket;
	for (int i = 0; i < _maxsize; i++) {
		if (_connectionlist[i] != 0) {
			FD_SET(_connectionlist[i], &_socks);
			if (_connectionlist[i] > _highsock)
				_highsock = _connectionlist[i];
		}
	}

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = SOCKET_ACCEPT_TIMEOUT_MS * 1000;

	int readsocks = select(_highsock + 1, &_socks, (fd_set *)0, (fd_set *)0, &tv);
	if (readsocks < 0) {
		return -1;
	} else if (readsocks == 0) {
		return 0;
	} else {
		if (FD_ISSET(_socket, &_socks)) {
			// New Connection
			Socket::SOCKET connection = ::accept(_socket, NULL, NULL);
			if (connection < 0) return connection;
			Socket::socket_nonblock(connection);
			for (int i = 0; (i < _maxsize) && (connection >= 0); i++) {
				if (_connectionlist[i] == 0) {
					// Connection Accepted
					_connectionlist[i] = connection;
					connection = -1;
				}
			}
			if (connection != -1) {
				// Overflow, server busy
				Socket::socket_close(connection);
			}
		}
		for (int i = 0; i < _maxsize; i++) {
			if (FD_ISSET(_connectionlist[i], &_socks)) {
				// Data
				if (_cb) {
					_cb(i, _connectionlist[i]);
				}
			}
		}
	}
	return 0;
}

void Socket::SelectiveServerSocket::on_data(std::function<void(int client_id, Socket::ClientSocket sock)> callback) {
	_cb = callback;
}

void Socket::SelectiveServerSocket::send_to_all_connected_clients(char *buffer, size_t length) {
	for (int i = 0; i < _maxsize; i++) {
		if (_connectionlist[i] != 0) {
			int ret = ::send(_connectionlist[i], buffer, length, MSG_NOSIGNAL);
			if (ret == -1) {
				if (errno == EPIPE) {
					// Client connection is now closed
					Socket::socket_close(_connectionlist[i]);
					_connectionlist[i] = 0;
				}
			}
		}
	}
}

// Remove any no-longer connected clients from the connectionlist
void Socket::SelectiveServerSocket::prune_disconnected_clients() {
	for (int i = 0; i < _maxsize; i++) {
		if (_connectionlist[i] != 0) {
			char buffer[32];
			if (recv(_connectionlist[i], buffer, sizeof(buffer), MSG_PEEK | MSG_DONTWAIT) == 0 ) {
				Socket::socket_close(_connectionlist[i]);
				_connectionlist[i] = 0;
			}
		}
	}
}

int Socket::SelectiveServerSocket::get_num_connected_clients() {
	int count = 0;
	for (int i = 0; i < _maxsize; i++) {
		if (_connectionlist[i] != 0) {
			count++;
		}
	}
	return count;
}

void Socket::SelectiveServerSocket::close_connected_clients() {
	for (int i = 0; i < _maxsize; i++) {
		if (_connectionlist[i] != 0) {
			Socket::socket_close(_connectionlist[i]);
			_connectionlist[i] = 0;
		}
	}
}

int Socket::DatagramSocket::bind() {
	return Socket::socket_bind(_socket, port);
}

int Socket::DatagramSocket::read(char *buf, size_t length, Socket::SocketAddress *addr) {
	#ifdef OS_WIN
		int *ptr = addr->raw_address_len_ptr();
	#else
		unsigned int *ptr = (unsigned int *)addr->raw_address_len_ptr();
	#endif
	return ::recvfrom(_socket, buf, length, 0, (struct sockaddr *)addr->raw_address(), ptr);
}

int Socket::DatagramSocket::read_with_timeout(char *buf, size_t length, Socket::SocketAddress *addr, uint32_t wait_milliseconds) {
	fd_set set;
	struct timeval timeout;
	FD_ZERO(&set);
	FD_SET(_socket, &set);
	
	timeout.tv_sec = wait_milliseconds / 1000;
	timeout.tv_usec = (wait_milliseconds % 1000) * 1000;

	int rv = select(_socket+1, &set, NULL, NULL, &timeout);
	if (rv == -1) {
		perror("Socket::DatagramSocket::read_with_timeout - select()"); // error
		return -2;
	} else if (rv == 0) {
		return -1; // timeout
	}

	unsigned int *ptr = (unsigned int *)addr->raw_address_len_ptr();	
	return ::recvfrom(_socket, buf, length, 0, (struct sockaddr *)addr->raw_address(), ptr);
}

int Socket::DatagramSocket::send(const char *buf, size_t length, Socket::SocketAddress *addr) {
	return sendto(_socket, buf, length, 0, (struct sockaddr *)addr->raw_address(), addr->raw_address_length());
}

int Socket::DatagramSocket::close() {
	return Socket::socket_close(_socket);
}
