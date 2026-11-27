/* test_supply.c — host test for supply.c: the four supply names, the legacy efuse key, both keys. */
#include "supply.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

static supply_req_t res(const char *s, const char *e, bool agree, supply_t *out) {
    return supply_resolve(s, e, agree, out);
}

int main(void) {
    supply_t v = SUPPLY_1V8;

    /* Names, both ways. */
    CHECK(!strcmp(supply_name(SUPPLY_5V), "5v") && !strcmp(supply_name(SUPPLY_EXTERNAL), "external"));
    CHECK(!strcmp(supply_name(SUPPLY_3V3), "3v3") && !strcmp(supply_name(SUPPLY_1V8), "1v8"));
    CHECK(!strcmp(supply_name((supply_t)9), "?"));
    CHECK(supply_from_name("5v") == SUPPLY_5V && supply_from_name("external") == SUPPLY_EXTERNAL);
    CHECK(supply_from_name("3v3") == SUPPLY_3V3 && supply_from_name("1v8") == SUPPLY_1V8);
    CHECK(supply_from_name("internal") == -1 && supply_from_name("j1_3v3") == -1 && supply_from_name("5V") == -1);
    CHECK(supply_from_name("") == -1 && supply_from_name(NULL) == -1);

    /* eFuse numbers and the J1 test. */
    CHECK(supply_efuse(SUPPLY_5V) == 1 && supply_efuse(SUPPLY_EXTERNAL) == 2);
    CHECK(supply_efuse(SUPPLY_3V3) == 0 && supply_efuse(SUPPLY_1V8) == 0);
    CHECK(supply_from_efuse(1) == SUPPLY_5V && supply_from_efuse(2) == SUPPLY_EXTERNAL);
    CHECK(supply_from_efuse(0) == -1 && supply_from_efuse(3) == -1);
    CHECK(!supply_is_j1(SUPPLY_5V) && !supply_is_j1(SUPPLY_EXTERNAL) && supply_is_j1(SUPPLY_3V3) && supply_is_j1(SUPPLY_1V8));

    /* On the board: 3v3 and 1v8 only with the feature. */
    CHECK(supply_on_board(SUPPLY_5V, false) && supply_on_board(SUPPLY_EXTERNAL, false));
    CHECK(!supply_on_board(SUPPLY_3V3, false) && !supply_on_board(SUPPLY_1V8, false));
    CHECK(supply_on_board(SUPPLY_3V3, true) && supply_on_board(SUPPLY_1V8, true));

    /* supply alone. */
    CHECK(res("5v", NULL, false, &v) == SUPPLY_REQ_OK && v == SUPPLY_5V);
    CHECK(res("external", NULL, false, &v) == SUPPLY_REQ_OK && v == SUPPLY_EXTERNAL);
    CHECK(res("3v3", NULL, false, &v) == SUPPLY_REQ_OK && v == SUPPLY_3V3);
    CHECK(res("1v8", NULL, false, &v) == SUPPLY_REQ_OK && v == SUPPLY_1V8);

    /* The legacy efuse alone. */
    CHECK(res(NULL, "1", false, &v) == SUPPLY_REQ_OK && v == SUPPLY_5V);
    CHECK(res(NULL, "2", false, &v) == SUPPLY_REQ_OK && v == SUPPLY_EXTERNAL);

    /* Neither key. */
    CHECK(res(NULL, NULL, false, &v) == SUPPLY_REQ_ABSENT);

    /* Unknown values. */
    CHECK(res("12v", NULL, false, &v) == SUPPLY_REQ_BAD_SUPPLY);
    CHECK(res("j1_3v3", NULL, false, &v) == SUPPLY_REQ_BAD_SUPPLY);
    CHECK(res("", NULL, false, &v) == SUPPLY_REQ_BAD_SUPPLY);
    CHECK(res(NULL, "0", false, &v) == SUPPLY_REQ_BAD_EFUSE);
    CHECK(res(NULL, "3", false, &v) == SUPPLY_REQ_BAD_EFUSE);
    CHECK(res(NULL, "x", false, &v) == SUPPLY_REQ_BAD_EFUSE);
    CHECK(res(NULL, "1x", false, &v) == SUPPLY_REQ_BAD_EFUSE);
    CHECK(res(NULL, "", false, &v) == SUPPLY_REQ_BAD_EFUSE);

    /* Both keys: a command refuses them even when they agree; a profile takes them when they agree. */
    CHECK(res("5v", "1", false, &v) == SUPPLY_REQ_BOTH);
    CHECK(res("3v3", "1", false, &v) == SUPPLY_REQ_BOTH);
    v = SUPPLY_1V8;
    CHECK(res("5v", "1", true, &v) == SUPPLY_REQ_OK && v == SUPPLY_5V);
    CHECK(res("external", "2", true, &v) == SUPPLY_REQ_OK && v == SUPPLY_EXTERNAL);
    CHECK(res("external", "1", true, &v) == SUPPLY_REQ_CONFLICT);
    CHECK(res("3v3", "1", true, &v) == SUPPLY_REQ_CONFLICT);
    CHECK(res("3v3", "9", true, &v) == SUPPLY_REQ_BAD_EFUSE);
    CHECK(res("nope", "1", true, &v) == SUPPLY_REQ_BAD_SUPPLY);
    CHECK(res("5v", NULL, false, NULL) == SUPPLY_REQ_OK);   /* out may be NULL */

    if (failures) { printf("test_supply: %d FAILED\n", failures); return 1; }
    printf("test_supply: all passed\n");
    return 0;
}
