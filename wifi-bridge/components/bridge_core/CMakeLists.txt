# Pure C core (protocol codec, forwarding rules, helpers): no ESP-IDF APIs, host-tested in test/.
idf_component_register(SRCS "bwp.c" "l2fwd.c" "bridge_util.c" "hmac_sha256.c" "pairing.c" "bwp_crypto.c" "fwsig.c"
                       INCLUDE_DIRS "include"
                       REQUIRES monocypher)
