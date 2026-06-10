/*
 * crash_capture.c — Captures crash/lockup context into retained (.noinit) RAM,
 * then reports it on next boot.
 *
 * Three mechanisms:
 *
 * 1. __wrap_z_arm_nmi (naked asm) — WDT timeout fires NMI.  The naked stub
 *    reads EXC_RETURN to determine which stack (MSP/PSP) holds the
 *    interrupted context, then tail-calls crash_nmi_c() with a pointer to
 *    the exception frame.  This gives the EXACT PC of the stuck code.
 *
 * 2. k_sys_fatal_error_handler override — captures PC/LR/CFSR from the
 *    Zephyr exception stack frame on hard faults, bus faults, etc.
 *
 * 3. crash_capture_init (SYS_INIT) — on boot, reads RESETREAS and checks
 *    .noinit for a crash record.  Logs both.  Then arms the HW WDT (8 s)
 *    and starts a workqueue feeder (every 3 s).
 *
 * After reboot, decode the logged PC with:
 *   arm-none-eabi-addr2line -e build/right/zephyr/zephyr.elf 0x<PC>
 *
 * Linker requirement: -Wl,--wrap=z_arm_nmi  (added in CMakeLists.txt)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/fatal.h>
#include <zephyr/arch/cpu.h>
#include <hal/nrf_power.h>

LOG_MODULE_REGISTER(crash, LOG_LEVEL_ERR);

/* ------------------------------------------------------------------ */
/* Retained crash record (.noinit — survives soft/WDT reset)          */
/* ------------------------------------------------------------------ */

#define CRASH_MAGIC 0xDEAD1234U

struct crash_data {
	uint32_t magic;
	uint32_t pc;      /* program counter at crash / lockup              */
	uint32_t lr;      /* link register (caller)                         */
	uint32_t psr;     /* xPSR — bits [8:0] = ISR number of interrupted  */
	uint32_t cfsr;    /* Configurable Fault Status Register             */
	uint32_t hfsr;    /* HardFault Status Register                      */
	uint32_t icsr;    /* Interrupt Control / State Register              */
	uint32_t source;  /* 0 = fault  |  1 = WDT NMI                     */
};

static volatile struct crash_data __attribute__((section(".noinit")))
    crash_info;

/* ------------------------------------------------------------------ */
/* WDT feeding — runs on the system workqueue                         */
/* ------------------------------------------------------------------ */

static const struct device *wdt_dev;
static int wdt_ch = -1;

static void wdt_feed_work_fn(struct k_work *w)
{
	ARG_UNUSED(w);
	if (wdt_dev && wdt_ch >= 0) {
		wdt_feed(wdt_dev, wdt_ch);
	}
	k_work_reschedule((struct k_work_delayable *)
			  CONTAINER_OF(w, struct k_work_delayable, work),
			  K_SECONDS(3));
}

static K_WORK_DELAYABLE_DEFINE(wdt_feed_dw, wdt_feed_work_fn);

/* ------------------------------------------------------------------ */
/* NMI handler — WDT timeout captures the interrupted PC              */
/* ------------------------------------------------------------------ */

/*
 * C handler: receives a pointer to the hardware exception frame that
 * the NMI interrupted.  Frame layout (Cortex-M4):
 *   [0]=R0  [1]=R1  [2]=R2  [3]=R3  [4]=R12  [5]=LR  [6]=PC  [7]=xPSR
 */
void __used crash_nmi_c(uint32_t *frame)
{
	crash_info.magic  = CRASH_MAGIC;
	crash_info.pc     = frame[6];
	crash_info.lr     = frame[5];
	crash_info.psr    = frame[7];
	crash_info.cfsr   = SCB->CFSR;
	crash_info.hfsr   = SCB->HFSR;
	crash_info.icsr   = SCB->ICSR;
	crash_info.source = 1;

	/* Spin; the WDT hardware will reset the chip in ~61 µs. */
	for (;;) {
		__NOP();
	}
}

/*
 * Naked NMI entry — linked via  -Wl,--wrap=z_arm_nmi  so the vector
 * table calls us instead of Zephyr's default z_arm_nmi().
 *
 * At NMI entry LR holds EXC_RETURN.  Bit 2 tells us which stack
 * contains the interrupted context's exception frame:
 *   0 → MSP  (interrupted handler / ISR)
 *   1 → PSP  (interrupted thread)
 */
__attribute__((naked))
void __wrap_z_arm_nmi(void)
{
	__asm volatile(
		"tst   lr, #4            \n"
		"ite   eq                \n"
		"mrseq r0, msp           \n"
		"mrsne r0, psp           \n"
		"b     crash_nmi_c       \n"
	);
}

/* ------------------------------------------------------------------ */
/* Fatal error handler — faults (HardFault, BusFault, …)              */
/* ------------------------------------------------------------------ */

void k_sys_fatal_error_handler(unsigned int reason,
			       const struct arch_esf *esf)
{
	ARG_UNUSED(reason);

	crash_info.magic  = CRASH_MAGIC;
	crash_info.source = 0;
	crash_info.cfsr   = SCB->CFSR;
	crash_info.hfsr   = SCB->HFSR;
	crash_info.icsr   = SCB->ICSR;

	if (esf) {
		crash_info.pc  = esf->basic.pc;
		crash_info.lr  = esf->basic.lr;
		crash_info.psr = esf->basic.xpsr;
	} else {
		crash_info.pc  = 0xFFFFFFFFU;
		crash_info.lr  = 0xFFFFFFFFU;
		crash_info.psr = 0;
	}

	NVIC_SystemReset();
}

/* ------------------------------------------------------------------ */
/* Deferred crash report — waits for USB CDC to be ready              */
/* ------------------------------------------------------------------ */

/* Saved at SYS_INIT time so the report can be emitted later */
static uint32_t saved_resetreas;
static struct crash_data saved_crash;
static bool have_crash;

static void crash_report_work_fn(struct k_work *w)
{
	ARG_UNUSED(w);

	/* Emit reset reason + crash data after USB serial is up */
	LOG_ERR("BOOT: resetreas=0x%08x%s%s%s%s%s%s", saved_resetreas,
		(saved_resetreas & 0x01)      ? " PIN"    : "",
		(saved_resetreas & 0x02)      ? " DOG"    : "",
		(saved_resetreas & 0x04)      ? " SREQ"   : "",
		(saved_resetreas & 0x08)      ? " LOCKUP" : "",
		(saved_resetreas & 0x10000)   ? " OFF"    : "",
		(saved_resetreas & 0x100000)  ? " VBUS"   : "");

	if (have_crash) {
		const char *src = saved_crash.source ? "WDT" : "FAULT";

		LOG_ERR("CRASH[%s]: PC=0x%08x LR=0x%08x PSR=0x%08x",
			src, saved_crash.pc, saved_crash.lr, saved_crash.psr);
		LOG_ERR("CRASH: CFSR=0x%08x HFSR=0x%08x ICSR=0x%08x",
			saved_crash.cfsr, saved_crash.hfsr, saved_crash.icsr);

		uint32_t isr = saved_crash.psr & 0x1FFU;
		if (isr == 0) {
			LOG_ERR("CRASH: context=Thread");
		} else {
			LOG_ERR("CRASH: context=ISR #%u", isr);
		}
	}

	LOG_ERR("WDT: armed 8s, feed 3s");
}

static K_WORK_DELAYABLE_DEFINE(crash_report_dw, crash_report_work_fn);

/* ------------------------------------------------------------------ */
/* Boot-time: save crash data + arm WDT (report deferred 35 s)        */
/* ------------------------------------------------------------------ */

static int crash_capture_init(void)
{
	/* ---- 1. Snapshot reset reason (clear immediately) ---- */
	saved_resetreas = nrf_power_resetreas_get(NRF_POWER);
	nrf_power_resetreas_clear(NRF_POWER, saved_resetreas);

	/* ---- 2. Snapshot previous-crash record ---- */
	if (crash_info.magic == CRASH_MAGIC) {
		saved_crash = *(const struct crash_data *)&crash_info;
		have_crash = true;
		crash_info.magic = 0; /* clear sentinel */
	}

	/* ---- 3. Schedule deferred report (35 s — USB CDC is up by then) */
	k_work_reschedule(&crash_report_dw, K_SECONDS(35));

	/* ---- 4. Arm hardware watchdog (8 s timeout) ---- */
	wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));
	if (!device_is_ready(wdt_dev)) {
		LOG_ERR("WDT: device not ready");
		wdt_dev = NULL;
		return 0;
	}

	struct wdt_timeout_cfg cfg = {
		.window = { .min = 0, .max = 8000 },
		.callback = NULL,
	};

	wdt_ch = wdt_install_timeout(wdt_dev, &cfg);
	if (wdt_ch < 0) {
		LOG_ERR("WDT: install failed %d", wdt_ch);
		return 0;
	}

	int err = wdt_setup(wdt_dev, WDT_OPT_PAUSE_IN_SLEEP);
	if (err) {
		LOG_ERR("WDT: setup failed %d", err);
		return 0;
	}

	/* First feed + start periodic feeder */
	wdt_feed(wdt_dev, wdt_ch);
	k_work_reschedule(&wdt_feed_dw, K_SECONDS(3));

	LOG_ERR("WDT: armed 8s, feed 3s");
	return 0;
}

SYS_INIT(crash_capture_init, APPLICATION, 99);
