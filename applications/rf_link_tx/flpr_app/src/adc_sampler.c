#include "adc_sampler.h"
#include "lp_trace.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdbool.h>

#include "proto.h"

/* ---------- IIM-42352 register definitions ---------- */
#define IIM42352_REG_INT_CONFIG     0x14u
#define IIM42352_REG_FIFO_CONFIG    0x16u
#define IIM42352_REG_ACCEL_DATA_X1  0x1Fu
#define IIM42352_REG_INT_STATUS     0x2Du
#define IIM42352_REG_FIFO_COUNTH    0x2Eu
#define IIM42352_REG_FIFO_COUNTL    0x2Fu
#define IIM42352_REG_FIFO_DATA      0x30u
#define IIM42352_REG_SIGNAL_PATH_RESET 0x4Bu
#define IIM42352_REG_INTF_CONFIG0   0x4Cu
#define IIM42352_REG_PWR_MGMT0      0x4Eu
#define IIM42352_REG_ACCEL_CONFIG0  0x50u
#define IIM42352_REG_FIFO_CONFIG1   0x5Fu
#define IIM42352_REG_FIFO_CONFIG2   0x60u
#define IIM42352_REG_FIFO_CONFIG3   0x61u
#define IIM42352_REG_INT_SOURCE0    0x65u
#define IIM42352_REG_WHO_AM_I       0x75u

#define IIM42352_WHO_AM_I_VAL     0x6D
#define IIM42352_SPI_READ         0x80

/* ---------- FIFO constants ---------- */
#define FIFO_WM_PACKETS    128u   /* half of 2048-byte FIFO */
#define FIFO_PACKET_SIZE   8u    /* 1 header + 6 accel + 1 temp = 8 bytes */
/* FIFO_WM valid range: 1 <= WM <= 4095; do NOT set to 0 (datasheet requirement) */
#define FIFO_WM_BYTES      (FIFO_WM_PACKETS * FIFO_PACKET_SIZE)  /* 128 */
#define FIFO_MAX_PACKETS   32u
#define FIFO_MAX_BYTES     (FIFO_MAX_PACKETS * FIFO_PACKET_SIZE)  /* 256 */

/* SPI operation flags: Mode 3 (CPOL=1, CPHA=1), 8-bit, MSB first */
#define SPI_OP (SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB)

/* Get SPI device spec from device tree */
static const struct spi_dt_spec mems_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(mems_sensor), SPI_OP, 0);

/* ---------- INT1 GPIO: P0.02 on gpio0, falling edge ---------- */
#define INT1_PORT  DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define INT1_PIN   2

static struct gpio_callback gpio_cb;

/* ---------- Ring buffer: 4 frames × 96 samples = 384 int16_t ---------- */
#define RING_BUF_SIZE 384

static int16_t ring_buf[RING_BUF_SIZE];
static uint32_t ring_head;  /* write position */
static uint32_t ring_tail;  /* read position */
static uint32_t ring_count; /* valid sample count */

/* ---------- Semaphore ---------- */
static struct k_sem data_sem;

/* ---------- Work queue ---------- */
static struct k_work fifo_work;

/* ---------- Sequence counter ---------- */
static uint16_t seq;

/* ================================================================
 * SPI register access helpers
 * ================================================================ */

/**
 * Read a single register from IIM42352.
 * Returns 0 on success, negative errno on failure.
 */
static int iim42352_read_reg(uint8_t reg, uint8_t *val)
{
	uint8_t tx_buf[2] = { reg | IIM42352_SPI_READ, 0x00 };
	uint8_t rx_buf[2] = { 0 };

	const struct spi_buf tx = { .buf = tx_buf, .len = 2 };
	const struct spi_buf rx = { .buf = rx_buf, .len = 2 };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_transceive_dt(&mems_spi, &tx_set, &rx_set);
	if (ret == 0) {
		*val = rx_buf[1];
	}
	return ret;
}

/**
 * Write a single register to IIM42352.
 */
static int iim42352_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t tx_buf[2] = { reg & 0x7F, val };

	const struct spi_buf tx = { .buf = tx_buf, .len = 2 };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	return spi_write_dt(&mems_spi, &tx_set);
}

/**
 * Burst-read up to 'len' bytes from FIFO_DATA register.
 * SPI transaction:
 *   tx: [FIFO_DATA | READ] [len dummy 0x00 bytes]
 *   rx: [dummy]            [len data bytes]
 * The first rx byte is dummy and skipped; the remaining 'len' bytes
 * are copied into 'buf'.
 * Returns 0 on success, negative errno on failure.
 */
static int iim42352_fifo_burst_read(uint8_t *buf, uint16_t len)
{
	static uint8_t tx_buf[1 + FIFO_MAX_BYTES];
	static uint8_t rx_buf[1 + FIFO_MAX_BYTES];

	if (len == 0u || len > FIFO_MAX_BYTES) {
		return -EINVAL;
	}

	tx_buf[0] = IIM42352_REG_FIFO_DATA | IIM42352_SPI_READ;
	for (uint16_t i = 1u; i <= len; i++) {
		tx_buf[i] = 0x00;
	}

	const struct spi_buf tx = { .buf = tx_buf, .len = (size_t)(len + 1u) };
	const struct spi_buf rx = { .buf = rx_buf, .len = (size_t)(len + 1u) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_transceive_dt(&mems_spi, &tx_set, &rx_set);
	if (ret == 0) {
		memcpy(buf, &rx_buf[1], len);
	}
	return ret;
}

/* ================================================================
 * Ring buffer helpers
 * ================================================================ */

static void ring_put(int16_t sample)
{
	if (ring_count < RING_BUF_SIZE) {
		ring_buf[ring_head] = sample;
		ring_head = (ring_head + 1u) % RING_BUF_SIZE;
		ring_count++;
	} else {
		/* Overwrite oldest: advance tail */
		ring_buf[ring_head] = sample;
		ring_head = (ring_head + 1u) % RING_BUF_SIZE;
		ring_tail = (ring_tail + 1u) % RING_BUF_SIZE;
	}
}

static int16_t ring_get(void)
{
	if (ring_count == 0) {
		return 0;
	}
	int16_t val = ring_buf[ring_tail];
	ring_tail = (ring_tail + 1u) % RING_BUF_SIZE;
	ring_count--;
	return val;
}

/* ================================================================
 * FIFO work handler – runs in system work queue context
 * Triggered by FIFO watermark interrupt; drains all complete packets.
 * ================================================================ */

static void fifo_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int ret;

	/* Read INT_STATUS to clear interrupt flag */
	uint8_t int_status = 0;
	ret = iim42352_read_reg(IIM42352_REG_INT_STATUS, &int_status);
	if (ret != 0) {
		rf_link_lp_trace_note_fatal(0x20);
		return;
	}
	/* Always proceed to read FIFO regardless of interrupt source.
	 * If FIFO has data, we should read it. FIFO_COUNT will be 0
	 * if there's nothing to read, which is handled below. */

	/* 2. Read FIFO byte count (high then low) */
	uint8_t cnt_h = 0;
	uint8_t cnt_l = 0;
	ret = iim42352_read_reg(IIM42352_REG_FIFO_COUNTH, &cnt_h);
	if (ret != 0) {
		rf_link_lp_trace_note_fatal(0x21);
		return;
	}
	ret = iim42352_read_reg(IIM42352_REG_FIFO_COUNTL, &cnt_l);
	if (ret != 0) {
		rf_link_lp_trace_note_fatal(0x22);
		return;
	}
	uint16_t fifo_bytes = ((uint16_t)cnt_h << 8) | cnt_l;

	/* Sanity check: byte count must be packet-aligned and within FIFO depth */
	if (fifo_bytes > 2048u) {
		rf_link_lp_trace_note_fatal(0x25);
		return;
	}

	if ((fifo_bytes % FIFO_PACKET_SIZE) != 0u) {
		/* Misaligned FIFO data, flush by reading and discarding */
		return;
	}

	/* 3. Calculate complete packets, cap to buffer size */
	uint16_t packets = fifo_bytes / FIFO_PACKET_SIZE;
	if (packets == 0u) {
		return;
	}
	if (packets > FIFO_MAX_PACKETS) {
		packets = FIFO_MAX_PACKETS;
	}
	uint16_t read_bytes = packets * FIFO_PACKET_SIZE;

	/* 4. Burst read FIFO data */
	static uint8_t fifo_data[FIFO_MAX_BYTES];
	ret = iim42352_fifo_burst_read(fifo_data, read_bytes);
	if (ret != 0) {
		rf_link_lp_trace_note_fatal(0x24);
		return;
	}

	/* 5. Parse each 8-byte packet */
	for (uint16_t i = 0; i < packets; i++) {
		uint8_t *pkt = &fifo_data[i * FIFO_PACKET_SIZE];

		/* Header check: bit6 = 1 means accel data valid (Packet1) */
		if ((pkt[0] & 0x40u) == 0u) {
			continue; /* skip invalid/empty packet */
		}

		/* Extract 3-axis accel data (MSB first) */
		int16_t accel_x = (int16_t)((uint16_t)pkt[1] << 8 | pkt[2]);
		int16_t accel_y = (int16_t)((uint16_t)pkt[3] << 8 | pkt[4]);
		int16_t accel_z = (int16_t)((uint16_t)pkt[5] << 8 | pkt[6]);
		/* pkt[7] = temperature byte, ignored */

		ring_put(accel_x);
		ring_put(accel_y);
		ring_put(accel_z);
		k_sem_give(&data_sem);
	}
}

/* ================================================================
 * GPIO ISR callback
 * ================================================================ */

static void gpio_isr_callback(const struct device *port,
			       struct gpio_callback *cb,
			       uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_work_submit(&fifo_work);
}

/* ================================================================
 * Public API
 * ================================================================ */

void adc_sampler_init(void)
{
	seq = 0;
	ring_head = 0;
	ring_tail = 0;
	ring_count = 0;

	k_sem_init(&data_sem, 0, 128);
	k_work_init(&fifo_work, fifo_work_handler);

	/* ---- Check SPI bus ready ---- */
	if (!spi_is_ready_dt(&mems_spi)) {
		rf_link_lp_trace_note_fatal(0x03);
		return;
	}

	/* ---- Read WHO_AM_I ---- */
	uint8_t who_am_i = 0;
	if (iim42352_read_reg(IIM42352_REG_WHO_AM_I, &who_am_i) != 0) {
		rf_link_lp_trace_note_fatal(0x04);
		return;
	}
	if (who_am_i != IIM42352_WHO_AM_I_VAL) {
		rf_link_lp_trace_note_fatal(0x05 | ((uint32_t)who_am_i << 8));
		return;
	}

	/* ---- 1. Configure INT1: push-pull, active-low, pulsed ---- */
	if (iim42352_write_reg(IIM42352_REG_INT_CONFIG, 0x02) != 0) {
		rf_link_lp_trace_note_fatal(0x11);
		return;
	}

	/* ---- 2. Flush FIFO via SIGNAL_PATH_RESET ---- */
	/* bit1 = FIFO_FLUSH: write 1 to flush, auto-clears */
	if (iim42352_write_reg(IIM42352_REG_SIGNAL_PATH_RESET, 0x02) != 0) {
		rf_link_lp_trace_note_fatal(0x38);
		return;
	}
	k_sleep(K_MSEC(1));  /* brief wait for flush to complete */

	/* ---- 3. INTF_CONFIG0: ensure FIFO counts in bytes ---- */
	/* Read-modify-write: clear bit6 (FIFO_COUNT_REC = 0 -> byte mode) */
	uint8_t intf_cfg = 0;
	if (iim42352_read_reg(IIM42352_REG_INTF_CONFIG0, &intf_cfg) != 0) {
		rf_link_lp_trace_note_fatal(0x30);
		return;
	}
	intf_cfg &= ~(1u << 6);
	if (iim42352_write_reg(IIM42352_REG_INTF_CONFIG0, intf_cfg) != 0) {
		rf_link_lp_trace_note_fatal(0x31);
		return;
	}

	/* ---- 4. FIFO_CONFIG: STREAM mode ---- */
	/* bits[7:6] = 01 -> continuous fill, new data overwrites oldest */
	if (iim42352_write_reg(IIM42352_REG_FIFO_CONFIG, 0x40) != 0) {
		rf_link_lp_trace_note_fatal(0x35);
		return;
	}

	/* ---- 5. FIFO_CONFIG1: enable Packet1 (accel data into FIFO) ---- */
	/* bit[1:0] = 01 = Packet1; bit4(HIRES)=0, bit3(TMST)=0, bit2(TEMP)=0 */
	if (iim42352_write_reg(IIM42352_REG_FIFO_CONFIG1, 0x01) != 0) {
		rf_link_lp_trace_note_fatal(0x32);
		return;
	}

	/* ---- 6. FIFO watermark: 128 bytes = 16 packets x 8 bytes ---- */
	if (iim42352_write_reg(IIM42352_REG_FIFO_CONFIG2,
			       (uint8_t)(FIFO_WM_BYTES & 0xFFu)) != 0) {
		rf_link_lp_trace_note_fatal(0x33);
		return;
	}
	if (iim42352_write_reg(IIM42352_REG_FIFO_CONFIG3,
			       (uint8_t)((FIFO_WM_BYTES >> 8) & 0x0Fu)) != 0) {
		rf_link_lp_trace_note_fatal(0x34);
		return;
	}

	/* ---- 7. Accel config: 4 kHz ODR, ±16 g ---- */
	if (iim42352_write_reg(IIM42352_REG_ACCEL_CONFIG0, 0x04) != 0) {
		rf_link_lp_trace_note_fatal(0x13);
		return;
	}

	/* ---- 8. Power on: accel low-noise mode ---- */
	/* PWR_MGMT0 MUST come after all FIFO configuration (datasheet 13.6) */
	if (iim42352_write_reg(IIM42352_REG_PWR_MGMT0, 0x03) != 0) {
		rf_link_lp_trace_note_fatal(0x09);
		return;
	}

	/* Wait for sensor to stabilise after power-on */
	k_sleep(K_MSEC(30));

	/* ---- 9. INT_SOURCE0: route FIFO watermark interrupt to INT1 ---- */
	/* bit[2] = FIFO_THS_INT1_EN — enable after sensor is running */
	if (iim42352_write_reg(IIM42352_REG_INT_SOURCE0, 0x04) != 0) {
		rf_link_lp_trace_note_fatal(0x12);
		return;
	}

	/* Verify INT_SOURCE0 was written correctly */
	uint8_t int_src_verify = 0;
	if (iim42352_read_reg(IIM42352_REG_INT_SOURCE0, &int_src_verify) != 0) {
		rf_link_lp_trace_note_fatal(0x36);
		return;
	}
	if (int_src_verify != 0x04u) {
		rf_link_lp_trace_note_fatal(0x37);
		return;
	}

	/* ---- 10. Clear any pending interrupts before enabling GPIO ---- */
	{
		uint8_t dummy_status = 0;
		iim42352_read_reg(IIM42352_REG_INT_STATUS, &dummy_status);
	}

	/* ---- Configure INT1 GPIO interrupt (P0.02, falling edge) ---- */
	if (!device_is_ready(INT1_PORT)) {
		rf_link_lp_trace_note_fatal(0x14);
		return;
	}

	if (gpio_pin_configure(INT1_PORT, INT1_PIN, GPIO_INPUT) != 0) {
		rf_link_lp_trace_note_fatal(0x15);
		return;
	}

	if (gpio_pin_interrupt_configure(INT1_PORT, INT1_PIN,
					 GPIO_INT_EDGE_FALLING) != 0) {
		rf_link_lp_trace_note_fatal(0x16);
		return;
	}

	gpio_init_callback(&gpio_cb, gpio_isr_callback, BIT(INT1_PIN));
	if (gpio_add_callback(INT1_PORT, &gpio_cb) != 0) {
		rf_link_lp_trace_note_fatal(0x17);
		return;
	}

	/* ---- Diagnostic: direct accel register read ---- */
	{
		uint8_t tx_diag[7];
		uint8_t rx_diag[7];
		tx_diag[0] = 0x1Fu | 0x80u;  /* ACCEL_DATA_X1 | READ */
		for (int i = 1; i < 7; i++) tx_diag[i] = 0x00u;

		const struct spi_buf tx_d = { .buf = tx_diag, .len = 7 };
		const struct spi_buf rx_d = { .buf = rx_diag, .len = 7 };
		const struct spi_buf_set tx_ds = { .buffers = &tx_d, .count = 1 };
		const struct spi_buf_set rx_ds = { .buffers = &rx_d, .count = 1 };

		if (spi_transceive_dt(&mems_spi, &tx_ds, &rx_ds) == 0) {
			/* Report accel X high byte via trace (non-zero = sensor working) */
			rf_link_lp_trace_note_loop(0xA000u | ((uint16_t)rx_diag[1] << 4) | (rx_diag[2] >> 4));
		}
	}

	/* ---- Diagnostic: check FIFO is filling ---- */
	{
		k_sleep(K_MSEC(50));  /* Wait 50ms for FIFO to accumulate data */

		uint8_t cnt_h = 0, cnt_l = 0;
		iim42352_read_reg(IIM42352_REG_FIFO_COUNTH, &cnt_h);
		iim42352_read_reg(IIM42352_REG_FIFO_COUNTL, &cnt_l);
		uint16_t diag_fifo_bytes = ((uint16_t)cnt_h << 8) | cnt_l;

		/* Report FIFO byte count via trace */
		/* 0xB000 | fifo_bytes: if 0 = FIFO not filling (problem!) */
		/* if > 0 = FIFO is working */
		rf_link_lp_trace_note_loop(0xB000u | (diag_fifo_bytes & 0xFFFu));
	}

	/* ---- Diagnostic: read first FIFO packet to check header ---- */
	{
		uint8_t cnt_h2 = 0, cnt_l2 = 0;
		iim42352_read_reg(IIM42352_REG_FIFO_COUNTH, &cnt_h2);
		iim42352_read_reg(IIM42352_REG_FIFO_COUNTL, &cnt_l2);
		uint16_t fifo_avail = ((uint16_t)cnt_h2 << 8) | cnt_l2;

		if (fifo_avail >= FIFO_PACKET_SIZE) {
			uint8_t test_pkt[8] = {0};
			iim42352_fifo_burst_read(test_pkt, FIFO_PACKET_SIZE);

			/* Report: 0xC0HH where HH = header byte value */
			/* Expected: 0xC040 if header = 0x40 (Packet1 with accel) */
			rf_link_lp_trace_note_loop(0xC000u | test_pkt[0]);

			/* Report: 0xD0XX where XX = first accel byte (should be non-zero) */
			rf_link_lp_trace_note_loop(0xD000u | ((uint16_t)test_pkt[1] << 4) | (test_pkt[2] >> 4));
		} else {
			/* FIFO still empty after 50ms - report this */
			rf_link_lp_trace_note_loop(0xE000u);
		}
	}
}

void adc_sampler_fill_frame(struct rf_frame *frame)
{
	if (frame == NULL) {
		return;
	}

	frame->magic = RF_LINK_MAGIC;
	frame->seq = seq++;
	frame->flags = 0;
	frame->timestamp_ms = k_uptime_get_32();

	/* Wait for 32 groups (32 × 3 = 96 samples) via semaphore.
	 * Each FIFO watermark interrupt produces ~16 groups (packets).
	 * Timeout 16 ms per take: if data is not ready in time,
	 * use whatever we have and zero-fill the rest.
	 */
	uint32_t groups_needed = RF_LINK_FRAME_SAMPLE_COUNT / 3u; /* 32 */
	uint32_t groups_got = 0;

	for (uint32_t i = 0; i < groups_needed; i++) {
		if (k_sem_take(&data_sem, K_MSEC(16)) == 0) {
			groups_got++;
		} else {
			/* Timeout: stop waiting, use what we have */
			break;
		}
	}

	/* Read available samples from ring buffer (up to groups_got × 3) */
	uint32_t samples_available = groups_got * 3u;
	if (samples_available > ring_count) {
		samples_available = ring_count;
	}

	for (uint32_t i = 0; i < samples_available; i++) {
		frame->samples[i] = ring_get();
	}

	/* Polling fallback: DISABLED for testing
	if (samples_available == 0) {
		uint8_t tx_buf[7] = { IIM42352_REG_ACCEL_DATA_X1 | IIM42352_SPI_READ };
		uint8_t rx_buf[7] = { 0 };
		const struct spi_buf tx = { .buf = tx_buf, .len = 7 };
		const struct spi_buf rx = { .buf = rx_buf, .len = 7 };
		const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
		const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

		if (spi_transceive_dt(&mems_spi, &tx_set, &rx_set) == 0) {
			frame->samples[0] = (int16_t)((uint16_t)rx_buf[1] << 8 | rx_buf[2]);
			frame->samples[1] = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[4]);
			frame->samples[2] = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[6]);
			samples_available = 3;
		}
	}
	*/

	/* Zero-fill remaining slots */
	for (uint32_t i = samples_available; i < RF_LINK_FRAME_SAMPLE_COUNT; i++) {
		frame->samples[i] = 0;
	}

	frame->sample_count = (uint16_t)samples_available;
}
