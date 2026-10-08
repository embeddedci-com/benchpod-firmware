/**
 * @file
 * altcp_bp_proxy: an HTTP CONNECT layer for the pod's cloud link (cloud-hardening.md section 3).
 *
 * Adapted from lwIP's apps/http/altcp_proxyconnect.c (license below), with what a corporate proxy
 * needs and the original lacks: the CONNECT line carries the server's host NAME (proxies filter on
 * it, and the pod may not be able to resolve outside names), an optional Proxy-Authorization:
 * Basic header, and the reply must be "HTTP/1.x 200" or the connection fails. Layered as
 * tls(bp_proxy(tcp)): TLS starts its handshake only once the tunnel is up.
 */

/*
 * Copyright (c) 2018 Simon Goldschmidt
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
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Simon Goldschmidt <goldsimon@gmx.de>
 *
 */

#include "altcp_bp_proxy.h"

#if LWIP_ALTCP /* don't build if not configured for use in lwipopts.h */

#include "lwip/altcp.h"
#include "lwip/priv/altcp_priv.h"

#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"

#include "lwip/mem.h"
#include "lwip/init.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** This string is passed in the HTTP header as "User-Agent: " */
#ifndef ALTCP_BP_PROXY_CLIENT_AGENT
#define ALTCP_BP_PROXY_CLIENT_AGENT "BenchPod"
#endif

#define ALTCP_BP_PROXY_FLAGS_CONNECT_STARTED  0x01
#define ALTCP_BP_PROXY_FLAGS_HANDSHAKE_DONE   0x02

typedef struct altcp_bp_proxy_state_s
{
  ip_addr_t outer_addr;
  u16_t outer_port;
  struct altcp_bp_proxy_config *conf;
  u8_t flags;
  char reply[16];        /* start of the proxy's status line, for the 200 check */
  u8_t reply_len;
  u8_t end_match;        /* how much of "\r\n\r\n" the reply has matched so far */
} altcp_bp_proxy_state_t;

/* Variable prototype, the actual declaration is at the end of this file
   since it contains pointers to static functions declared here */
extern const struct altcp_functions altcp_bp_proxy_functions;

/* memory management functions: */

static altcp_bp_proxy_state_t *
altcp_bp_proxy_state_alloc(void)
{
  altcp_bp_proxy_state_t *ret = (altcp_bp_proxy_state_t *)mem_calloc(1, sizeof(altcp_bp_proxy_state_t));
  return ret;
}

static void
altcp_bp_proxy_state_free(altcp_bp_proxy_state_t *state)
{
  LWIP_ASSERT("state != NULL", state != NULL);
  mem_free(state);
}

/* helper functions */

/* Create and send the CONNECT request: the target by host name, optional Basic auth. */
static err_t
altcp_bp_proxy_send_request(struct altcp_pcb *conn)
{
  altcp_bp_proxy_state_t *state = (altcp_bp_proxy_state_t *)conn->state;
  if (!state || !state->conf || !state->conf->target_host) {
    return ERR_VAL;
  }
  const struct altcp_bp_proxy_config *c = state->conf;
  char *buffer = (char *)mem_malloc(512);
  if (buffer == NULL) {
    return ERR_MEM;
  }
  int len = snprintf(buffer, 512,
                     "CONNECT %s:%u HTTP/1.1\r\n"
                     "Host: %s:%u\r\n"
                     "User-Agent: " ALTCP_BP_PROXY_CLIENT_AGENT "\r\n"
                     "%s%s%s"
                     "Proxy-Connection: keep-alive\r\n"
                     "\r\n",
                     c->target_host, (unsigned)state->outer_port, c->target_host,
                     (unsigned)state->outer_port,
                     c->auth_b64 ? "Proxy-Authorization: Basic " : "",
                     c->auth_b64 ? c->auth_b64 : "", c->auth_b64 ? "\r\n" : "");
  err_t err = ERR_VAL;
  if (len > 0 && len < 512) {
    err = altcp_write(conn->inner_conn, buffer, (u16_t)len, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
      err = altcp_output(conn->inner_conn);
    }
  }
  mem_free(buffer);
  return err;
}

/* Fail the connection during setup (the proxy refused, or closed before its reply). Called
 * from our lower_recv, i.e. from inside tcp_input via altcp_tcp_recv; returns ERR_ABRT, which
 * altcp_tcp_recv hands back to tcp_input ("pcb aborted, do not touch it").
 *
 * The stack is app -> tls (T) -> this layer (conn) -> altcp_tcp (inner_conn) -> tcp_pcb, and
 * every layer must be freed exactly once, with nobody left pointing at a freed one:
 *
 *  1. Remove our callbacks from inner_conn, then abort it. tcp_abort() calls altcp_tcp_err,
 *     which frees inner_conn (its err callback is now NULL, so it does not call us back), and
 *     tcp_abandon frees the tcp_pcb (tcp_input knows from ERR_ABRT not to touch it).
 *  2. Clear conn->inner_conn: from here on nothing may reach the freed TCP layer through us.
 *  3. Tell the layer above the way an error from below does (altcp_bp_proxy_lower_err).
 *     conn->err is altcp_mbedtls_lower_err: it sets T->inner_conn = NULL (so T never touches us
 *     again), calls the application's err callback (cl_err_cb: drops its pcb pointer and marks
 *     the link failed, where last_status / closed_early give the reason), and frees T.
 *  4. Free this layer (altcp_bp_proxy_dealloc frees our state). Nobody points at it now.
 *
 * Forwarding the FIN up instead (conn->recv(arg, conn, NULL)) would also work in principle:
 * altcp_mbedtls_lower_recv, before its handshake is done, calls err(ERR_ABRT) and then
 * altcp_close(T), which closes us and tcp_close()s the pcb from inside its own recv callback.
 * But that calls the application's err callback and then still uses T, and a failing tcp_close
 * (ERR_MEM) leaves the pcbs half-closed with the application already told they are gone. The
 * abort path is one sequence for every setup failure, and it is the one bench-tested on 407. */
static err_t
altcp_bp_proxy_fail_setup(struct altcp_pcb *conn, struct altcp_pcb *inner_conn)
{
  altcp_arg(inner_conn, NULL);
  altcp_recv(inner_conn, NULL);
  altcp_sent(inner_conn, NULL);
  altcp_err(inner_conn, NULL);
  altcp_poll(inner_conn, NULL, 0);
  altcp_abort(inner_conn);
  conn->inner_conn = NULL;
  if (conn->err) {
    conn->err(conn->arg, ERR_ABRT);
  }
  altcp_free(conn);
  return ERR_ABRT;
}

/* callback functions from inner/lower connection: */

/** Connected callback from lower connection (i.e. TCP).
 * Not really implemented/tested yet...
 */
static err_t
altcp_bp_proxy_lower_connected(void *arg, struct altcp_pcb *inner_conn, err_t err)
{
  struct altcp_pcb *conn = (struct altcp_pcb *)arg;
  if (conn && conn->state) {
    LWIP_ASSERT("pcb mismatch", conn->inner_conn == inner_conn);
    LWIP_UNUSED_ARG(inner_conn); /* for LWIP_NOASSERT */
    /* upper connected is called when handshake is done */
    if (err != ERR_OK) {
      if (conn->connected) {
        if (conn->connected(conn->arg, conn, err) == ERR_ABRT) {
          return ERR_ABRT;
        }
        return ERR_OK;
      }
    }
    /* send proxy connect request here */
    return altcp_bp_proxy_send_request(conn);
  }
  return ERR_VAL;
}

/** Recv callback from lower connection (i.e. TCP)
 * This one mainly differs between connection setup (wait for proxy OK string)
 * and application phase (data is passed on to the application).
 */
static err_t
altcp_bp_proxy_lower_recv(void *arg, struct altcp_pcb *inner_conn, struct pbuf *p, err_t err)
{
  altcp_bp_proxy_state_t *state;
  struct altcp_pcb *conn = (struct altcp_pcb *)arg;

  LWIP_ASSERT("no err expected", err == ERR_OK);
  LWIP_UNUSED_ARG(err);

  if (!conn) {
    /* no connection given as arg? should not happen, but prevent pbuf/conn leaks */
    if (p != NULL) {
      pbuf_free(p);
    }
    altcp_close(inner_conn);
    return ERR_CLSD;
  }
  state = (altcp_bp_proxy_state_t *)conn->state;
  LWIP_ASSERT("pcb mismatch", conn->inner_conn == inner_conn);
  if (!state) {
    /* already closed */
    if (p != NULL) {
      pbuf_free(p);
    }
    altcp_close(inner_conn);
    return ERR_CLSD;
  }
  if (state->flags & ALTCP_BP_PROXY_FLAGS_HANDSHAKE_DONE) {
    /* application phase, just pass this through */
    if (conn->recv) {
      return conn->recv(conn->arg, conn, p, err);
    }
    pbuf_free(p);
    return ERR_OK;
  } else {
    /* setup phase */
    /* handle NULL pbuf (inner connection closed) */
    if (p == NULL) {
      /* The proxy closed before its reply (FIN in the setup phase). Closing only ourselves here
         (what lwIP's altcp_proxyconnect does) frees this layer while TLS above still has it as
         its inner_conn: the application's later altcp_close/abort of the TLS pcb then runs
         altcp_mbedtls_close on freed memory. Fail the whole stack instead (fail_setup). */
      state->conf->last_status = 0;
      state->conf->closed_early = 1;
      return altcp_bp_proxy_fail_setup(conn, inner_conn);
    } else {
      /* Collect the status line and wait for the end of the reply headers. The proxy sends
         nothing after them until TLS speaks (the client talks first), so no data is lost. */
      struct pbuf *q;
      for (q = p; q != NULL && state->end_match < 4; q = q->next) {
        const u8_t *b = (const u8_t *)q->payload;
        for (u16_t i = 0; i < q->len && state->end_match < 4; i++) {
          static const char end[] = "\r\n\r\n";
          if (state->reply_len < sizeof(state->reply) - 1) {
            state->reply[state->reply_len++] = (char)b[i];
          }
          state->end_match = (b[i] == (u8_t)end[state->end_match]) ? (u8_t)(state->end_match + 1)
                           : (b[i] == '\r' ? 1u : 0u);
        }
      }
      altcp_recved(inner_conn, p->tot_len);
      pbuf_free(p);
      if (state->end_match == 4) {
        state->reply[state->reply_len] = '\0';
        /* "HTTP/1.0 200" or "HTTP/1.1 200": anything else (407 auth, 403, 502) fails. */
        if (strncmp(state->reply, "HTTP/1.", 7) != 0 || strncmp(state->reply + 8, " 200", 4) != 0) {
          state->conf->last_status = (u16_t)atoi(state->reply + 9);
          /* Aborting only ourselves would leave TLS pointing at freed memory: seen as a
             HardFault in altcp_mbedtls_lower_recv on the bench. */
          return altcp_bp_proxy_fail_setup(conn, inner_conn);
        }
        state->conf->last_status = 200;
        state->flags |= ALTCP_BP_PROXY_FLAGS_HANDSHAKE_DONE;
        if (conn->connected) {
          return conn->connected(conn->arg, conn, ERR_OK);
        }
      }
      return ERR_OK;
    }
  }
}

/** Sent callback from lower connection (i.e. TCP)
 * This only informs the upper layer to try to send more, not about
 * the number of ACKed bytes.
 */
static err_t
altcp_bp_proxy_lower_sent(void *arg, struct altcp_pcb *inner_conn, u16_t len)
{
  struct altcp_pcb *conn = (struct altcp_pcb *)arg;
  LWIP_UNUSED_ARG(len);
  if (conn) {
    altcp_bp_proxy_state_t *state = (altcp_bp_proxy_state_t *)conn->state;
    LWIP_ASSERT("pcb mismatch", conn->inner_conn == inner_conn);
    LWIP_UNUSED_ARG(inner_conn); /* for LWIP_NOASSERT */
    if (!state || !(state->flags & ALTCP_BP_PROXY_FLAGS_HANDSHAKE_DONE)) {
      /* @todo: do something here? */
      return ERR_OK;
    }
    /* pass this on to upper sent */
    if (conn->sent) {
      return conn->sent(conn->arg, conn, len);
    }
  }
  return ERR_OK;
}

/** Poll callback from lower connection (i.e. TCP)
 * Just pass this on to the application.
 * @todo: retry sending?
 */
static err_t
altcp_bp_proxy_lower_poll(void *arg, struct altcp_pcb *inner_conn)
{
  struct altcp_pcb *conn = (struct altcp_pcb *)arg;
  if (conn) {
    LWIP_ASSERT("pcb mismatch", conn->inner_conn == inner_conn);
    LWIP_UNUSED_ARG(inner_conn); /* for LWIP_NOASSERT */
    if (conn->poll) {
      return conn->poll(conn->arg, conn);
    }
  }
  return ERR_OK;
}

static void
altcp_bp_proxy_lower_err(void *arg, err_t err)
{
  struct altcp_pcb *conn = (struct altcp_pcb *)arg;
  if (conn) {
    conn->inner_conn = NULL; /* already freed */
    if (conn->err) {
      conn->err(conn->arg, err);
    }
    altcp_free(conn);
  }
}


/* setup functions */

static void
altcp_bp_proxy_setup_callbacks(struct altcp_pcb *conn, struct altcp_pcb *inner_conn)
{
  altcp_arg(inner_conn, conn);
  altcp_recv(inner_conn, altcp_bp_proxy_lower_recv);
  altcp_sent(inner_conn, altcp_bp_proxy_lower_sent);
  altcp_err(inner_conn, altcp_bp_proxy_lower_err);
  /* tcp_poll is set when interval is set by application */
  /* listen is set totally different :-) */
}

static err_t
altcp_bp_proxy_setup(struct altcp_bp_proxy_config *config, struct altcp_pcb *conn, struct altcp_pcb *inner_conn)
{
  altcp_bp_proxy_state_t *state;
  if (!config) {
    return ERR_ARG;
  }
  LWIP_ASSERT("invalid inner_conn", conn != inner_conn);

  /* allocate proxyconnect context */
  state = altcp_bp_proxy_state_alloc();
  if (state == NULL) {
    return ERR_MEM;
  }
  state->flags = 0;
  state->conf = config;
  altcp_bp_proxy_setup_callbacks(conn, inner_conn);
  conn->inner_conn = inner_conn;
  conn->fns = &altcp_bp_proxy_functions;
  conn->state = state;
  return ERR_OK;
}

/** Allocate a new altcp layer connecting through a proxy.
 * This function gets the inner pcb passed.
 *
 * @param config struct altcp_bp_proxy_config that contains the proxy settings
 * @param inner_pcb pcb that makes the connection to the proxy (i.e. tcp pcb)
 */
struct altcp_pcb *
altcp_bp_proxy_new(struct altcp_bp_proxy_config *config, struct altcp_pcb *inner_pcb)
{
  struct altcp_pcb *ret;
  if (inner_pcb == NULL) {
    return NULL;
  }
  ret = altcp_alloc();
  if (ret != NULL) {
    if (altcp_bp_proxy_setup(config, ret, inner_pcb) != ERR_OK) {
      altcp_free(ret);
      return NULL;
    }
  }
  return ret;
}

/** Allocate a new altcp layer connecting through a proxy.
 * This function allocates the inner pcb as tcp pcb, resulting in a direct tcp
 * connection to the proxy.
 *
 * @param config struct altcp_bp_proxy_config that contains the proxy settings
 * @param ip_type IP type of the connection (@ref lwip_ip_addr_type)
 */
struct altcp_pcb *
altcp_bp_proxy_new_tcp(struct altcp_bp_proxy_config *config, u8_t ip_type)
{
  struct altcp_pcb *inner_pcb, *ret;

  /* inner pcb is tcp */
  inner_pcb = altcp_tcp_new_ip_type(ip_type);
  if (inner_pcb == NULL) {
    return NULL;
  }
  ret = altcp_bp_proxy_new(config, inner_pcb);
  if (ret == NULL) {
    altcp_close(inner_pcb);
  }
  return ret;
}



/* "virtual" functions */
static void
altcp_bp_proxy_set_poll(struct altcp_pcb *conn, u8_t interval)
{
  if (conn != NULL) {
    altcp_poll(conn->inner_conn, altcp_bp_proxy_lower_poll, interval);
  }
}

static void
altcp_bp_proxy_recved(struct altcp_pcb *conn, u16_t len)
{
  altcp_bp_proxy_state_t *state;
  if (conn == NULL) {
    return;
  }
  state = (altcp_bp_proxy_state_t *)conn->state;
  if (state == NULL) {
    return;
  }
  if (!(state->flags & ALTCP_BP_PROXY_FLAGS_HANDSHAKE_DONE)) {
    return;
  }
  altcp_recved(conn->inner_conn, len);
}

static err_t
altcp_bp_proxy_connect(struct altcp_pcb *conn, const ip_addr_t *ipaddr, u16_t port, altcp_connected_fn connected)
{
  altcp_bp_proxy_state_t *state;

  if ((conn == NULL) || (ipaddr == NULL)) {
    return ERR_VAL;
  }
  state = (altcp_bp_proxy_state_t *)conn->state;
  if (state == NULL) {
    return ERR_VAL;
  }
  if (state->flags & ALTCP_BP_PROXY_FLAGS_CONNECT_STARTED) {
    return ERR_VAL;
  }
  state->flags |= ALTCP_BP_PROXY_FLAGS_CONNECT_STARTED;

  conn->connected = connected;
  /* connect to our proxy instead, but store the requested address and port */
  ip_addr_copy(state->outer_addr, *ipaddr);
  state->outer_port = port;

  return altcp_connect(conn->inner_conn, &state->conf->proxy_addr, state->conf->proxy_port, altcp_bp_proxy_lower_connected);
}

static struct altcp_pcb *
altcp_bp_proxy_listen(struct altcp_pcb *conn, u8_t backlog, err_t *err)
{
  LWIP_UNUSED_ARG(conn);
  LWIP_UNUSED_ARG(backlog);
  LWIP_UNUSED_ARG(err);
  /* listen not supported! */
  return NULL;
}

/* Aborting the TCP pcb below calls its err callback, which is altcp_bp_proxy_lower_err: that
   frees this layer. Freeing it again afterwards (what lwIP's altcp_proxyconnect_abort does) is a
   double free, reached from cloud_client's altcp_abort of the proxy pcb when the TLS config
   cannot be built, and from an abort of the TLS pcb above. Take the setup-failure path instead:
   detach, abort the TCP pcb, tell the layer above (TLS frees itself there), free this layer once. */
static void
altcp_bp_proxy_abort(struct altcp_pcb *conn)
{
  if (conn != NULL) {
    if (conn->inner_conn != NULL) {
      (void)altcp_bp_proxy_fail_setup(conn, conn->inner_conn);
      return;
    }
    altcp_free(conn);
  }
}

static err_t
altcp_bp_proxy_close(struct altcp_pcb *conn)
{
  if (conn == NULL) {
    return ERR_VAL;
  }
  if (conn->inner_conn != NULL) {
    err_t err = altcp_close(conn->inner_conn);
    if (err != ERR_OK) {
      /* closing inner conn failed, return the error */
      return err;
    }
  }
  /* no inner conn or closing it succeeded, deallocate myself */
  altcp_free(conn);
  return ERR_OK;
}

static err_t
altcp_bp_proxy_write(struct altcp_pcb *conn, const void *dataptr, u16_t len, u8_t apiflags)
{
  altcp_bp_proxy_state_t *state;

  LWIP_UNUSED_ARG(apiflags);

  if (conn == NULL) {
    return ERR_VAL;
  }

  state = (altcp_bp_proxy_state_t *)conn->state;
  if (state == NULL) {
    /* @todo: which error? */
    return ERR_CLSD;
  }
  if (!(state->flags & ALTCP_BP_PROXY_FLAGS_HANDSHAKE_DONE)) {
    /* @todo: which error? */
    return ERR_VAL;
  }
  return altcp_write(conn->inner_conn, dataptr, len, apiflags);
}

static void
altcp_bp_proxy_dealloc(struct altcp_pcb *conn)
{
  /* clean up and free tls state */
  if (conn) {
    altcp_bp_proxy_state_t *state = (altcp_bp_proxy_state_t *)conn->state;
    if (state) {
      altcp_bp_proxy_state_free(state);
      conn->state = NULL;
    }
  }
}
const struct altcp_functions altcp_bp_proxy_functions = {
  altcp_bp_proxy_set_poll,
  altcp_bp_proxy_recved,
  altcp_default_bind,
  altcp_bp_proxy_connect,
  altcp_bp_proxy_listen,
  altcp_bp_proxy_abort,
  altcp_bp_proxy_close,
  altcp_default_shutdown,
  altcp_bp_proxy_write,
  altcp_default_output,
  altcp_default_mss,
  altcp_default_sndbuf,
  altcp_default_sndqueuelen,
  altcp_default_nagle_disable,
  altcp_default_nagle_enable,
  altcp_default_nagle_disabled,
  altcp_default_setprio,
  altcp_bp_proxy_dealloc,
  altcp_default_get_tcp_addrinfo,
  altcp_default_get_ip,
  altcp_default_get_port
#if LWIP_TCP_KEEPALIVE
  , altcp_default_keepalive_disable
  , altcp_default_keepalive_enable
#endif
#ifdef LWIP_DEBUG
  , altcp_default_dbg_get_tcp_state
#endif
};

#endif /* LWIP_ALTCP */
