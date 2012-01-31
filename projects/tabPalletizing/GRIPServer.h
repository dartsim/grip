#ifndef NWSERVER_H_
#define NWSERVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>

static struct sigaction sa;

static void sigchld_handler(int) {
	while (waitpid(-1, NULL, WNOHANG) > 0)
		;
}

static void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

class GRIPServer {

	int rv, yes;
	int sockfd, new_fd;
	struct addrinfo hints, *servinfo, *p;
	struct sockaddr_storage their_addr;
	socklen_t sin_size;
	char s[INET6_ADDRSTRLEN];

public:

	GRIPServer() {
		yes = 1;
		memset(&hints, 0, sizeof(hints));
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_flags = AI_PASSIVE;
	}

	int setup() {
		if ((rv = getaddrinfo(NULL, "3490", &hints, &servinfo)) != 0) {
			fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
			return 1;
		}

		for (p = servinfo; p != NULL; p = p->ai_next) {
			if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol))
					== -1) {
				perror("server: socket");
				continue;
			}

			if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int))
					== -1) {
				perror("setsockopt");
				exit(1);
			}

			if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
				close(sockfd);
				perror("server: bind");
				continue;
			}

			break;
		}

		if (p == NULL) {
			fprintf(stderr, "server: failed to bind\n");
			return 2;
		}

		freeaddrinfo(servinfo);

		if (listen(sockfd, 10) == -1) {
			perror("listen");
			exit(1);
		}

		sa.sa_handler = sigchld_handler; // reap all dead processes
		sigemptyset(&sa.sa_mask);
		sa.sa_flags = SA_RESTART;
		if (sigaction(SIGCHLD, &sa, NULL) == -1) {
			perror("sigaction");
			exit(1);
		}

		// printf("server: setup done...\n");

		return 0;
	}

	void acceptMode() {
		// printf("server: waiting for new connections...\n");
		sin_size = sizeof their_addr;
		new_fd = accept(sockfd, (struct sockaddr *) &their_addr, &sin_size);
		if (new_fd == -1) {
			perror("accept");
		} else {
			inet_ntop(their_addr.ss_family,
					get_in_addr((struct sockaddr *) &their_addr), s, sizeof s);
			// printf("server: got connection from %s\n", s);
		}

		std::string reply = "HTTP/1.1 200 OK\n"
			"Content-Type: text/html\n"
			"Accept-Ranges: bytes\n"
			"Connection: close\n"
			"\nWelcome to the GRIP Server!";

		if (send(new_fd, reply.c_str(), reply.size(), 0) == -1)
			perror("send");
		close(new_fd);
		return;
	}

	~GRIPServer() {
		close(sockfd);
	}
};

#endif /* NWSERVER_H_ */
