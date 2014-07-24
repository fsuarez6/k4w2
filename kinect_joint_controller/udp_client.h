#ifndef UDPCLIENT_H_
#define UDPCLIENT_H_

//includes for socket use
#include <winsock2.h>
#include <windows.h>

#include <iostream>
#include <string.h>

struct Skeleton
{
	int hp;
	int mp;
	int str;
};

class UdpClient {
public:
	UdpClient(const char *ip_address, int port_no);
	virtual ~UdpClient();
	int create();
	int close_socket();
	int send(const char *buffer);
	int send(float *data);
	int get_sent_size();

private:
	unsigned int serverlen_; // size of server structure
	WSADATA wsa_;
	SOCKET sockfd_; // socket file descriptor
	int sent_size_; // size of the sent buffer
	int port_; // port to write
	struct sockaddr_in server_; // address to write
};
#endif