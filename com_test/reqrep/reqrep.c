//
// Copyright 2018 Staysail Systems, Inc. <info@staysail.tech>
// Copyright 2021 Capitar IT Group BV <info@capitar.com>
//
// This software is supplied under the terms of the MIT License, a
// copy of which should be located in the distribution where this
// file was obtained (LICENSE.txt).  A copy of the license may also be
// found online at https://opensource.org/licenses/MIT.
//

//
// This is just a simple REQ/REP demonstration application.  It is derived
// from the legacy nanomsg demonstration program of the same name, written
// by Tim Dysinger, but updated for nng.  I've also updated it to pass simpler
// binary data rather than strings over the network.
//
// The program implements a simple RPC style service, which just returns
// the date in UNIX time (seconds since 1970).
//
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <fstream>

#include <nng/nng.h>
#include <nng/protocol/reqrep0/rep.h>
#include <nng/protocol/reqrep0/req.h>
#include <nng/transport/zerotier/zerotier.h>
#include <nng/supplemental/util/platform.h>

using namespace std;
#define CLIENT "client"
#define SERVER "server"
#define DATECMD 1
fstream writer;
#define PUT64(ptr, u)                                        \
	do {                                                 \
		(ptr)[0] = (uint8_t)(((uint64_t)(u)) >> 56); \
		(ptr)[1] = (uint8_t)(((uint64_t)(u)) >> 48); \
		(ptr)[2] = (uint8_t)(((uint64_t)(u)) >> 40); \
		(ptr)[3] = (uint8_t)(((uint64_t)(u)) >> 32); \
		(ptr)[4] = (uint8_t)(((uint64_t)(u)) >> 24); \
		(ptr)[5] = (uint8_t)(((uint64_t)(u)) >> 16); \
		(ptr)[6] = (uint8_t)(((uint64_t)(u)) >> 8);  \
		(ptr)[7] = (uint8_t)((uint64_t)(u));         \
	} while (0)

#define GET64(ptr, v)                                 \
	v = (((uint64_t)((uint8_t)(ptr)[0])) << 56) + \
	    (((uint64_t)((uint8_t)(ptr)[1])) << 48) + \
	    (((uint64_t)((uint8_t)(ptr)[2])) << 40) + \
	    (((uint64_t)((uint8_t)(ptr)[3])) << 32) + \
	    (((uint64_t)((uint8_t)(ptr)[4])) << 24) + \
	    (((uint64_t)((uint8_t)(ptr)[5])) << 16) + \
	    (((uint64_t)((uint8_t)(ptr)[6])) << 8) +  \
	    (((uint64_t)(uint8_t)(ptr)[7]))

#define PUT32(ptr, u)                                        \
	do {                                                 \
		(ptr)[0] = (uint8_t)(((uint32_t)(u)) >> 24); \
		(ptr)[1] = (uint8_t)(((uint32_t)(u)) >> 16); \
		(ptr)[2] = (uint8_t)(((uint32_t)(u)) >> 8); \
		(ptr)[3] = (uint8_t)(((uint32_t)(u))); \
	} while (0)

#define GET32(ptr, v)                                 \
	v = (((uint32_t)((uint8_t)(ptr)[0])) << 24) + \
	    (((uint32_t)((uint8_t)(ptr)[1])) << 16) + \
	    (((uint32_t)((uint8_t)(ptr)[2])) << 8) + \
	    (((uint32_t)((uint8_t)(ptr)[3])))


void putMsg(char* result,uint64_t timestamp, uint32_t size){
PUT64(result,timestamp);
char* size_place = result+sizeof(timestamp);
PUT32(size_place,size);
return;
}

void getMsg(char*msg, uint64_t& timestamp, uint32_t& size){
GET64(msg,timestamp);
char* size_place = msg+sizeof(uint64_t);
GET32(size_place,size);
//size = (uint32_t)(msg[sizeof(timestamp)]);
return;
}
void
fatal(const char *func, int rv)
{
	fprintf(stderr, "%s: %s\n", func, nng_strerror(rv));
	exit(1);
}

void
showdate(time_t now)
{
	struct tm *info = localtime(&now);
	printf("%s", asctime(info));
}

int
server(const char *url)
{
	nng_socket sock;
	nng_listener listener;
	int        rv;

	if ((rv = nng_rep0_open(&sock)) != 0) {
		fatal("nng_rep0_open", rv);
	}

	if ((rv = nng_listener_create(&listener, sock, url)) != 0) {
		fatal("nng_listener_create", rv);
	}

	if (strncmp(url, "zt://", 5) == 0) {
		printf("ZeroTier transport will store its keys in current working directory.\n");
		printf("The server and client instances must run in separate directories.\n");
		nng_listener_setopt_string(listener, NNG_OPT_ZT_HOME, ".");
		nng_listener_setopt_ms(listener, NNG_OPT_RECONNMINT, 1);
		nng_listener_setopt_ms(listener, NNG_OPT_RECONNMAXT, 1000);
		nng_setopt_ms(sock, NNG_OPT_REQ_RESENDTIME, 2000);
		nng_setopt_ms(sock, NNG_OPT_RECVMAXSZ, 0);
		nng_listener_setopt_ms(listener, NNG_OPT_ZT_PING_TIME, 10000);
		nng_listener_setopt_ms(listener, NNG_OPT_ZT_CONN_TIME, 1000);
	} else {
		nng_setopt_ms(sock, NNG_OPT_REQ_RESENDTIME, 2000);
	}
	nng_listener_start(listener, 0);

	for (;;) {
		char *   buf = NULL;
		size_t   sz;
		uint64_t val;
		if ((rv = nng_recv(sock, &buf, &sz, NNG_FLAG_ALLOC)) != 0) {
			fatal("nng_recv", rv);
		}
		uint64_t time_get;
		uint32_t size_get;
		getMsg(buf,time_get,size_get);
		if (sz == size_get) {
			//printf("SERVER: RECEIVED DATE REQUEST\n");
			rv = nng_send(sock, buf, sz, NNG_FLAG_ALLOC);
			if (rv != 0) {
				fatal("nng_send", rv);
			}
			continue;
		}
		// Unrecognized command, so toss the buffer.
		nng_free(buf, sz);
	}
}

int
client(const char *url,uint32_t msgSize,int countMax)
{
	nng_socket sock;
	nng_dialer dialer;
	int        rv;
	size_t     sz;
	char *     buf = NULL;
	char* msg_send = new char[msgSize+sizeof(uint64_t)+sizeof(uint32_t)];
	int        sleep = 0;
	int count = 0;


	if ((rv = nng_req0_open(&sock)) != 0) {
		fatal("nng_socket", rv);
	}

	if ((rv = nng_dialer_create(&dialer, sock, url)) != 0) {
		fatal("nng_dialer_create", rv);
	}

	if (strncmp(url, "zt://", 5) == 0) {
		printf("ZeroTier transport will store its keys in current working directory\n");
		printf("The server and client instances must run in separate directories.\n");
		nng_dialer_setopt_string(dialer, NNG_OPT_ZT_HOME, ".");
		nng_dialer_setopt_ms(dialer, NNG_OPT_RECONNMINT, 1);
		nng_dialer_setopt_ms(dialer, NNG_OPT_RECONNMAXT, 1000);
		nng_setopt_ms(sock, NNG_OPT_REQ_RESENDTIME, 2000);
		nng_setopt_ms(sock, NNG_OPT_RECVMAXSZ, 0);
		nng_dialer_setopt_ms(dialer, NNG_OPT_ZT_PING_TIME, 10000);
		nng_dialer_setopt_ms(dialer, NNG_OPT_ZT_CONN_TIME, 1000);
	} else {
		nng_setopt_ms(sock, NNG_OPT_REQ_RESENDTIME, 2000);
	}

	nng_dialer_start(dialer, NNG_FLAG_NONBLOCK);

	int send_size = sizeof(uint64_t)+sizeof(uint32_t)+msgSize;
	while (count<=countMax) {
		count++;

		uint64_t time_send =std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
	putMsg(msg_send,time_send,send_size);
		if ((rv = nng_send(sock, msg_send,send_size , 0))!=0){
			fatal("nng_send", rv);
				}
		if ((rv = nng_recv(sock, &buf, &sz, NNG_FLAG_ALLOC)) != 0) {
			fatal("nng_recv", rv);

		}

		uint64_t time_recv =std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
		if (sz ==send_size ) {
			uint64_t time_get;
			uint32_t size_get;
			//GET64(buf, time_get);
			getMsg(buf,time_get,size_get);
			//cout<<"time_send:"<<time_send<<endl;
			//cout<<"time_recv:"<<time_recv<<endl;
			writer<<time_recv-time_send<<endl;
		} else {
			printf("CLIENT: GOT WRONG SIZE!\n");
		}
	}
	writer.close();

	// This assumes that buf is ASCIIZ (zero terminated).
		delete[] msg_send;
	nng_free(buf, sz);
	nng_close(sock);
	return (0);
}

int
main(const int argc, const char **argv)
{
	if ((argc > 3) && (strcmp(CLIENT, argv[1]) == 0)){
		writer.open("./log.txt",ios::trunc|std::ios::out);
		return (client(argv[2],atoi(argv[3]),atoi(argv[4])));
	}

	if ((argc > 1) && (strcmp(SERVER, argv[1]) == 0))
		return (server(argv[2]));

	fprintf(stderr, "Usage: reqrep %s <URL> <Msg Size> <Count Max>\n", CLIENT);
	fprintf(stderr, "Usage: reqrep %s <URL>\n", SERVER);
	return (1);
}
