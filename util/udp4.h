
/*
 * interface for UDP sockets
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


/*
 * convenience record for combination of socket information
 */
typedef struct udp_socket_s
{
   int sock;
   struct sockaddr_in sin;
   int timeout;
}
udp_socket_t;


udp_socket_t *udp_socket_create(char *udp_host, int udp_port, int udp_ttl, int bind);

int udp_socket_send(udp_socket_t *udp_socket, void *data, unsigned int len);

int udp_socket_recv(udp_socket_t *udp_socket, void *data, unsigned int len, struct sockaddr_in *from);

void udp_socket_close(udp_socket_t *udp_socket);

