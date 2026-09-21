#pragma once

/* SCPI (IEEE-488.2) command interface served over the same TCP socket as the
   JSON protocol.  command_handler routes a connection here when its first
   non-whitespace byte is not '{'.  Lines are newline-terminated; responses are
   newline-terminated, with bulk sample data returned as ASCII CSV.  Maps onto
   the same underlying actions as the JSON commands (signal_engine, gpio_control,
   target_power, wifi_manager). */

/* Dispatch one complete, NUL-terminated SCPI command line (no trailing CR/LF)
   received on conn_id.  Sends any response via at_send_data(conn_id, ...). */
void scpi_dispatch_line(int conn_id, const char *line);

/* Notify the SCPI layer that a TCP connection has closed.  SCPI state is
   device-wide (one instrument) and captures are synchronous, so there is no
   per-connection state to release; provided for symmetry with the JSON path. */
void scpi_conn_closed(int conn_id);
