#ifndef BP_LIMITS_H
#define BP_LIMITS_H

/*
 * bp_limits — the coupled buffer sizes for the cloud/command protocol in ONE
 * place, with _Static_asserts pinning the relationships that used to be tracked
 * by hand-written comments spread across cloud_client.c and command_handler.c.
 *
 * The chain that has to stay consistent:
 *   raw tunnel bytes (BP_TUNNEL_CHUNK)
 *     -> base64url  (grows 4/3)
 *       -> wrapped in a tunnel.data JSON envelope (BP_TUNNEL_OUT_FRAME_MAX)
 *         -> framed as a masked WS frame (+ BP_WS_HEADER_MAX) in cl_ws_send's
 *            scratch (BP_WS_FRAME_MAX)
 * and likewise a command reply (BP_CLOUD_REPLY_MAX) wrapped in a command.response
 * envelope (BP_CLOUD_CMD_FRAME_MAX) then WS-framed.  Bump any one of these and the
 * asserts below fail the build if a partner buffer was not bumped to match.
 */
#include "b64url.h"        /* B64URL_ENCODED_LEN */
#include "cloud_config.h"  /* CLOUD_DEVICE_ID_MAX */

/* Identifiers carried in frames. */
#define BP_TUNNEL_ID_MAX        64u                    /* tunnel_id string cap        */
#define BP_DEVICE_ID_MAX        CLOUD_DEVICE_ID_MAX    /* device_id string cap        */

/* Raw payload bytes per device->server tunnel.data frame (chunked in
   cloud_client_tunnel_out so the encoded frame fits BP_WS_FRAME_MAX). */
#define BP_TUNNEL_CHUNK         768u

/* WebSocket masked-frame header worst case (2 hdr + 8 ext-len + 4 mask, rounded
   up).  Our payloads never exceed 65535 B so 14 is a safe ceiling. */
#define BP_WS_HEADER_MAX        14u

/* cl_ws_send scratch: holds the WS header + the largest payload we ever send. */
#define BP_WS_FRAME_MAX         1750u

/* Per-frame scratch: the text-frame copy + the inbound base64 scratch.  One full
   server->device frame must fit here (BP_CLOUD_RX_MAX >= BP_TUNNEL_OUT_FRAME_MAX). */
#define BP_CLOUD_RX_MAX         2048u

/* Inbound (decrypted) byte ACCUMULATOR (s_rx).  Larger than one frame so a burst of
   server->device frames (a chunked DAC replay upload) can queue up faster than the
   net task drains them without overflowing the buffer and resetting the whole cloud
   link ("backoff: rx overflow").  cl_recv_cb eager-acks (no TCP window backpressure),
   so this depth + the server's upload rate-limit are what bound the burst.  Holds
   ~12 max-size frames. */
#define BP_CLOUD_RX_ACCUM       16384u

/* A single-reply command's captured reply (command_handler cloud_cap_buf) and the
   matching reply[] the cloud client grafts into the response envelope.  1536, not 1024: the
   14-entry `la_pins` / `gpio` read reply reaches ~1324 B in its worst case (test_la_pins.c;
   the 12-entry reply was 1135 B). */
#define BP_CLOUD_REPLY_MAX      1536u

/* The inbound command JSON extracted from a command.request envelope. Larger than a plain
   control command so a compact closed-loop curve LUT (base64url) rides in one command. Must
   stay <= HW_WORK_DATA_MAX (the worker queue payload that carries it to the hw task). */
#define BP_CLOUD_CMD_IN_MAX     1280u

/* Fixed JSON overhead (minus the id fields, counted separately) of the frames we
   build, with generous slack.  Used only by the asserts. */
#define BP_TUNNEL_ENVELOPE_OVH  64u    /* {"type":"tunnel.data","tunnel_id":"","data_b64":""} */
#define BP_CMDRESP_ENVELOPE_OVH 96u    /* {"type":"command.response","request_id":"",...}      */

/* Assembled outbound frame buffers. */
#define BP_TUNNEL_OUT_FRAME_MAX 1300u  /* tunnel.data envelope + id + base64(chunk) */
#define BP_CLOUD_CMD_FRAME_MAX  1736u  /* command.response envelope + ids + reply    */

/* ---- relationships (fail the build if a coupled size drifts) -------------- */

/* An encoded tunnel chunk + its envelope + the tunnel id must fit the assembled
   tunnel.data buffer... */
_Static_assert(B64URL_ENCODED_LEN(BP_TUNNEL_CHUNK) + BP_TUNNEL_ENVELOPE_OVH +
                   BP_TUNNEL_ID_MAX <= BP_TUNNEL_OUT_FRAME_MAX,
               "tunnel.data frame buffer too small for one base64 chunk + envelope");

/* ...and that assembled frame plus the WS header must fit cl_ws_send's scratch. */
_Static_assert(BP_TUNNEL_OUT_FRAME_MAX + BP_WS_HEADER_MAX <= BP_WS_FRAME_MAX,
               "WS scratch too small for a tunnel.data frame");

/* A captured command reply + the response envelope + both ids must fit the
   command.response buffer... */
_Static_assert(BP_CLOUD_REPLY_MAX + BP_CMDRESP_ENVELOPE_OVH + BP_TUNNEL_ID_MAX +
                   BP_DEVICE_ID_MAX <= BP_CLOUD_CMD_FRAME_MAX,
               "command.response buffer too small for reply + envelope");

/* ...and that must WS-frame into the shared scratch too. */
_Static_assert(BP_CLOUD_CMD_FRAME_MAX + BP_WS_HEADER_MAX <= BP_WS_FRAME_MAX,
               "WS scratch too small for a command.response frame");

/* The per-frame scratch must hold at least the biggest frame we would ever send
   back to ourselves (a full tunnel.data), or a legitimate frame could never be
   parsed; the accumulator must in turn hold at least one such frame. */
_Static_assert(BP_CLOUD_RX_MAX >= BP_TUNNEL_OUT_FRAME_MAX,
               "inbound rx buffer smaller than an outbound tunnel.data frame");
_Static_assert(BP_CLOUD_RX_ACCUM >= BP_CLOUD_RX_MAX,
               "inbound accumulator smaller than the per-frame scratch");

#endif /* BP_LIMITS_H */
