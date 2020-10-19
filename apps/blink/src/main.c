#include "sysinit/sysinit.h"
#include "os/os.h"

/**
 * Depending on the type of package, there are different
 * compilation rules for this directory.  This comment applies
 * to packages of type "app."  For other types of packages,
 * please view the documentation at http://mynewt.apache.org/.
 *
 * Put source files in this directory.  All files that have a *.c
 * ending are recursively compiled in the src/ directory and its
 * descendants.  The exception here is the arch/ directory, which
 * is ignored in the default compilation.
 *
 * The arch/<your-arch>/ directories are manually added and
 * recursively compiled for all files that end with either *.c
 * or *.a.  Any directories in arch/ that don't match the
 * architecture being compiled are not compiled.
 *
 * Architecture is set by the BSP/MCU combination.
 */


#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#define EXT_OUTPUT          MCU_GPIO_PORTC(5)
#define BLINK_PERIOD        5000

static struct os_callout blink_work;

void blink_extio(struct os_event *work){

    hal_gpio_toggle(EXT_OUTPUT);
    os_callout_reset(&blink_work, BLINK_PERIOD);
}


int
main(int argc, char **argv)
{
    /* Perform some extra setup if we're running in the simulator. */
#ifdef ARCH_sim
    mcu_sim_parse_args(argc, argv);
#endif

    /* Initialize all packages. */
    sysinit();

    hal_gpio_init_out(EXT_OUTPUT, 0);

    os_callout_init(&blink_work, os_eventq_dflt_get(), blink_extio, NULL);
    os_callout_reset(&blink_work, BLINK_PERIOD);

    /* As the last thing, process events from default event queue. */
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
