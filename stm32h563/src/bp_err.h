#ifndef BP_ERR_H
#define BP_ERR_H

/*
 * bp_err — a shared vocabulary for the error conditions the command handlers
 * report, so the JSON reply strings are consistent instead of hand-typed (and
 * subtly divergent) at every call site.
 *
 * This intentionally does NOT change the return conventions of the instrument
 * drivers (signal_engine / i2c_bus / can_bus), which use their own function-local
 * negative codes and are hardware-verified — rewriting those in place would be
 * risky churn for little gain.  Instead each handler maps its driver's rc to one
 * of these abstract errors and passes bp_err_str() to send_error(), which keeps
 * the outward-facing error text uniform and greppable.  New code should map onto
 * this enum from the start.
 */

typedef enum {
    BP_OK = 0,
    BP_ERR_BUSY,          /* a shared resource is already in use             */
    BP_ERR_INVALID_ARG,   /* a parameter was missing / out of range          */
    BP_ERR_UNSUPPORTED,   /* not available on this board / gateware           */
    BP_ERR_HW_FAULT,      /* the hardware operation failed                    */
    BP_ERR_TIMEOUT,       /* the operation did not complete in time           */
    BP_ERR_NOT_READY,     /* a prerequisite (e.g. LA voltage) is unmet        */
    BP_ERR_TOO_LARGE,     /* a reply / payload did not fit its buffer         */
} bp_err_t;

static inline const char *bp_err_str(bp_err_t e) {
    switch (e) {
        case BP_OK:              return "ok";
        case BP_ERR_BUSY:        return "busy";
        case BP_ERR_INVALID_ARG: return "invalid argument";
        case BP_ERR_UNSUPPORTED: return "unsupported on this board";
        case BP_ERR_HW_FAULT:    return "hardware fault";
        case BP_ERR_TIMEOUT:     return "timeout";
        case BP_ERR_NOT_READY:   return "not ready";
        case BP_ERR_TOO_LARGE:   return "reply too large";
        default:                 return "error";
    }
}

#endif /* BP_ERR_H */
