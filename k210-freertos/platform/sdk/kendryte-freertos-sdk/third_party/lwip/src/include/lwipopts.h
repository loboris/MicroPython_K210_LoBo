
#if !defined LWIP_HDR_LWIPOPTS_H
#define LWIP_HDR_LWIPOPTS_H

/*
#define PPP_THREAD_NAME             "ppp"
#define PPP_THREAD_STACKSIZE        200
#define PPP_THREAD_PRIO             2

#define LWIP_DEBUG                  1
#define PPP_DEBUG                   LWIP_DBG_ON
*/

#define MEM_LIBC_MALLOC                 1
#define MEM_ALIGNMENT                   8
#define LWIP_COMPAT_SOCKETS             0

#define LWIP_ICMP                       1

#define TCPIP_THREAD_STACKSIZE          10240
#define TCPIP_MBOX_SIZE                 40
#define DEFAULT_RAW_RECVMBOX_SIZE       12
#define DEFAULT_UDP_RECVMBOX_SIZE       1600
#define DEFAULT_TCP_RECVMBOX_SIZE       1600
#define DEFAULT_ACCEPTMBOX_SIZE         8000

#endif
