/*
 * lwftp.h : a lightweight FTP client using raw API of LWIP
 *
 * Copyright (c) 2014 GEZEDO
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * Author: Laurent GONZALEZ <lwip@gezedo.com>
 *
 */

#ifndef LWFTP_H
#define LWFTP_H

#include "lwip/opt.h"
#include "lwip/ip.h"

#ifdef __cplusplus
extern "C" {
#endif

enum lwftp_results {
  LWFTP_RESULT_OK=0,
  LWFTP_RESULT_INPROGRESS,
  LWFTP_RESULT_LOGGED,
  LWFTP_RESULT_ERR_UNKNOWN,   /** Unknown error */
  LWFTP_RESULT_ERR_ARGUMENT,  /** Wrong argument */
  LWFTP_RESULT_ERR_MEMORY,    /** Out of memory */
  LWFTP_RESULT_ERR_CONNECT,   /** Connection to server failed */
  LWFTP_RESULT_ERR_HOSTNAME,  /** Failed to resolve server hostname */
  LWFTP_RESULT_ERR_CLOSED,    /** Connection unexpectedly closed by remote server */
  LWFTP_RESULT_ERR_TIMEOUT,   /** Connection timed out (server didn't respond in time) */
  LWFTP_RESULT_ERR_SRVR_RESP, /** Server responded with an unknown response code */
  LWFTP_RESULT_ERR_INTERNAL,  /** Internal network stack error */
  LWFTP_RESULT_ERR_LOCAL,     /** Local storage error */
  LWFTP_RESULT_ERR_FILENAME   /** Remote host could not find file */
};

/** LWFTP control connection state */
typedef enum  {
  LWFTP_CLOSED=0,
  LWFTP_CONNECTED,
  LWFTP_USER_SENT,
  LWFTP_PASS_SENT,
  LWFTP_LOGGED,
  LWFTP_TYPE_SENT,
  LWFTP_PASV_SENT,
  LWFTP_RETR_SENT,
  LWFTP_STOR_SENT,
  LWFTP_XFERING,
  LWFTP_DATAEND,
  LWFTP_QUIT,
  LWFTP_QUIT_SENT,
  LWFTP_CLOSING,
} lwftp_state_t;

/** LWFTP session structure */
typedef struct {
  // User interface
  ip_addr_t     server_ip;
  u16_t         server_port;
  const char    *remote_path;
  const char    *user;
  const char    *pass;
  void          *handle;
  uint          (*data_source)(void*, const char**, uint);
  uint          (*data_sink)(void*, const char*, uint);
  void          (*done_fn)(void*, int);
  uint          timeout;
  // Internal data
  lwftp_state_t   control_state;
  lwftp_state_t   target_state;
  lwftp_state_t   data_state;
  struct tcp_pcb  *control_pcb;
  struct tcp_pcb  *data_pcb;
} lwftp_session_t;

// LWFTP API
err_t lwftp_connect(lwftp_session_t *s);
err_t lwftp_store(lwftp_session_t *s);
err_t lwftp_retrieve(lwftp_session_t *s);
void  lwftp_close(lwftp_session_t *s);

#ifdef __cplusplus
}
#endif

#endif // LWFTP_H
