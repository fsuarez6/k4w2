#include "udp_client.h"

#pragma comment(lib, "ws2_32.lib")

UdpClient::UdpClient(const char *ip_address, int port_no) {
	serverlen_ = sizeof(server_); //size of address structures (used in creation of memory for ip address and sending)
	// Construct the server sockaddr_in structure
	memset(&server_, 0, serverlen_); // Clear struct
	server_.sin_family = AF_INET; // Internet/IP
	server_.sin_port = htons(port_no); // server port
	server_.sin_addr.s_addr = inet_addr(ip_address);
	sent_size_ = 0;
}

UdpClient::~UdpClient() {
}

int UdpClient::create() {
	// Initialize Winsock
	if (WSAStartup(MAKEWORD(2, 2), &wsa_) != 0)
		return -666;
	// Timeout. Just 1 milisecond	
	struct timeval t_out;
	t_out.tv_sec = 0;
	t_out.tv_usec = 1000;
	// Create the UDP socket
	sockfd_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sockfd_ == INVALID_SOCKET)
		return sockfd_;
	// Set timeout
	if (setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<char*>(&t_out), sizeof(t_out)) < 0)
		return -667;
	return sockfd_;
}

int UdpClient::send(const char *buffer) {
	sent_size_ = strlen(buffer) + 1;
	return sendto(sockfd_, buffer, sent_size_, 0, (struct sockaddr*) &server_,
			serverlen_);
}

int UdpClient::send(float *data) {
	char const * buffer = reinterpret_cast<char const *>(data);
	return send(buffer);
}

int UdpClient::close_socket() {
	WSACleanup();
	return closesocket(sockfd_);
}

int UdpClient::get_sent_size() {
	return sent_size_;
}
