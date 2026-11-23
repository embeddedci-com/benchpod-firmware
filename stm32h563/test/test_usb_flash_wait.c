/* Host test for usb_flash_wait.h: when usb_flash reads the drive back after a flash. */
#include "usb_flash_wait.h"

#include <stdio.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

int main(void)
{
    usb_flash_wait_t w;

    /* file: the drive re-enumerates (mount count 4 -> 5) and settles. */
    usb_flash_wait_start(&w, true, 1000, 4);
    CHECK(usb_flash_wait_poll(&w, 1000, 4, true) == USB_FLASH_WAIT_PENDING);   /* still the old drive */
    CHECK(usb_flash_wait_poll(&w, 3000, 4, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 3100, 4, false) == USB_FLASH_WAIT_PENDING);  /* gone */
    CHECK(usb_flash_wait_poll(&w, 4000, 5, true) == USB_FLASH_WAIT_PENDING);   /* back, settling */
    CHECK(usb_flash_wait_poll(&w, 4400, 5, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 4500, 5, true) == USB_FLASH_WAIT_READY);

    /* file: it came back before the first poll. */
    usb_flash_wait_start(&w, true, 1000, 4);
    CHECK(usb_flash_wait_poll(&w, 1100, 5, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 1600, 5, true) == USB_FLASH_WAIT_READY);

    /* file: a drive that blinks out while settling starts the settle time again. */
    usb_flash_wait_start(&w, true, 0, 1);
    CHECK(usb_flash_wait_poll(&w, 100, 2, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 400, 2, false) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 450, 3, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 900, 3, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 950, 3, true) == USB_FLASH_WAIT_READY);

    /* file: no drive comes back. */
    usb_flash_wait_start(&w, true, 0, 1);
    CHECK(usb_flash_wait_poll(&w, 9999, 1, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 10000, 1, false) == USB_FLASH_WAIT_TIMEOUT);

    /* uf2: the drive stays (the loader keeps BOOTSEL): ready after 1.5 s. */
    usb_flash_wait_start(&w, false, 0, 7);
    CHECK(usb_flash_wait_poll(&w, 0, 7, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 1499, 7, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 1500, 7, true) == USB_FLASH_WAIT_READY);

    /* uf2: the board starts its program and the drive goes: timeout (skipped). */
    usb_flash_wait_start(&w, false, 0, 7);
    CHECK(usb_flash_wait_poll(&w, 200, 7, false) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 4999, 7, false) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 5000, 7, false) == USB_FLASH_WAIT_TIMEOUT);

    /* uf2: the drive goes and returns. */
    usb_flash_wait_start(&w, false, 0, 7);
    CHECK(usb_flash_wait_poll(&w, 100, 7, false) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 2000, 8, true) == USB_FLASH_WAIT_PENDING);
    CHECK(usb_flash_wait_poll(&w, 2500, 8, true) == USB_FLASH_WAIT_READY);

    printf("test_usb_flash_wait: %s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
