/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2024  Henry Wynberg <henrywynberg@gmail.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

static bool samc_flash_erase(target_flash_s *f, target_addr_t addr, size_t len);
static bool samc_flash_write(target_flash_s *f, target_addr_t dest, const void *src, size_t len);
/* NB: This is not marked static on purpose as it's used by samx5x.c. */
bool samc_mass_erase(target_s *t);

static bool samc_cmd_lock_flash(target_s *t, int argc, const char **argv);
static bool samc_cmd_unlock_flash(target_s *t, int argc, const char **argv);
static bool samc_cmd_unlock_bootprot(target_s *t, int argc, const char **argv);
static bool samc_cmd_lock_bootprot(target_s *t, int argc, const char **argv);
static bool samc_cmd_read_userrow(target_s *t, int argc, const char **argv);
static bool samc_cmd_serial(target_s *t, int argc, const char **argv);
static bool samc_cmd_mbist(target_s *t, int argc, const char **argv);
static bool samc_cmd_ssb(target_s *t, int argc, const char **argv);

const command_s samc_cmd_list[] = {
	{"lock_flash", samc_cmd_lock_flash, "Locks flash against spurious commands"},
	{"unlock_flash", samc_cmd_unlock_flash, "Unlocks flash"},
	{"lock_bootprot", samc_cmd_lock_bootprot, "Lock the boot protections to maximum"},
	{"unlock_bootprot", samc_cmd_unlock_bootprot, "Unlock the boot protections to minimum"},
	{"user_row", samc_cmd_read_userrow, "Prints user row from flash"},
	{"serial", samc_cmd_serial, "Prints serial number"},
	{"mbist", samc_cmd_mbist, "Runs the built-in memory test"},
	{"set_security_bit", samc_cmd_ssb, "Sets the Security Bit"},
	{NULL, NULL, NULL},
};

/* Non-Volatile Memory Controller (NVMC) Parameters */
#define SAMC_ROW_SIZE  256U
#define SAMC_PAGE_SIZE 64U

/* -------------------------------------------------------------------------- */
/* Non-Volatile Memory Controller (NVMC) Registers */
/* -------------------------------------------------------------------------- */

#define SAMC_NVMC         0x41004000U
#define SAMC_NVMC_CTRLA   (SAMC_NVMC + 0x00U)
#define SAMC_NVMC_CTRLB   (SAMC_NVMC + 0x04U)
#define SAMC_NVMC_PARAM   (SAMC_NVMC + 0x08U)
#define SAMC_NVMC_INTFLAG (SAMC_NVMC + 0x14U)
#define SAMC_NVMC_STATUS  (SAMC_NVMC + 0x18U)
#define SAMC_NVMC_ADDRESS (SAMC_NVMC + 0x1cU)

/* Control A Register (CTRLA) */
#define SAMC_CTRLA_CMD_KEY             0xa500U
#define SAMC_CTRLA_CMD_ERASEROW        0x0002U
#define SAMC_CTRLA_CMD_WRITEPAGE       0x0004U
#define SAMC_CTRLA_CMD_ERASEAUXROW     0x0005U
#define SAMC_CTRLA_CMD_WRITEAUXPAGE    0x0006U
#define SAMC_CTRLA_CMD_LOCK            0x0040U
#define SAMC_CTRLA_CMD_UNLOCK          0x0041U
#define SAMC_CTRLA_CMD_PAGEBUFFERCLEAR 0x0044U
#define SAMC_CTRLA_CMD_SSB             0x0045U
#define SAMC_CTRLA_CMD_INVALL          0x0046U

/* Interrupt Flag Register (INTFLAG) */
#define SAMC_NVMC_READY (1U << 0U)

/* Non-Volatile Memory Calibration and Auxiliary Registers */
#define SAMC_NVM_USER_ROW_LOW  0x00804000U
#define SAMC_NVM_USER_ROW_HIGH 0x00804004U
#define SAMC_NVM_CALIBRATION   0x00806020U
#define SAMC_NVM_SERIAL(n)     (0x0080a00cU + (0x30U * (((n) + 3U) / 4U)) + ((n)*4U))

/* -------------------------------------------------------------------------- */
/* Device Service Unit (DSU) Registers */
/* -------------------------------------------------------------------------- */

#define SAMC_DSU            0x41002000U
#define SAMC_DSU_EXT_ACCESS (SAMC_DSU + 0x100U)
#define SAMC_DSU_CTRLSTAT   (SAMC_DSU_EXT_ACCESS + 0x0U)
#define SAMC_DSU_ADDRESS    (SAMC_DSU_EXT_ACCESS + 0x4U)
#define SAMC_DSU_LENGTH     (SAMC_DSU_EXT_ACCESS + 0x8U)
#define SAMC_DSU_DID        (SAMC_DSU_EXT_ACCESS + 0x018U)
#define SAMC_DSU_PID        (SAMC_DSU + 0x1000U)
#define SAMC_DSU_CID        (SAMC_DSU + 0x1010U)

/* Control and Status Register (CTRLSTAT) */
#define SAMC_CTRL_CHIP_ERASE (1U << 4U)
#define SAMC_CTRL_MBIST      (1U << 3U)
#define SAMC_CTRL_CRC        (1U << 2U)
#define SAMC_STATUSA_PERR    (1U << 12U)
#define SAMC_STATUSA_FAIL    (1U << 11U)
#define SAMC_STATUSA_BERR    (1U << 10U)
#define SAMC_STATUSA_CRSTEXT (1U << 9U)
#define SAMC_STATUSA_DONE    (1U << 8U)
#define SAMC_STATUSB_PROT    (1U << 16U)

/* Device Identification Register (DID) */
#define SAMC_DID_MASK          0xff380000U
#define SAMC_DID_CONST_VALUE   0x11000000U
#define SAMC_DID_DEVSEL_MASK   0xffU
#define SAMC_DID_DEVSEL_POS    0U
#define SAMC_DID_REVISION_MASK 0x0fU
#define SAMC_DID_REVISION_POS  8U
#define SAMC_DID_SERIES_MASK   0x1fU
#define SAMC_DID_SERIES_POS    16U
#define SAMC_DID_FAMILY_MASK   0x3fU
#define SAMC_DID_FAMILY_POS    23U

/* Peripheral ID */
#define SAMC_PID_MASK        0x00ffffffU
#define SAMC_PID_CONST_VALUE 0x0009fcD0U

/* Component ID */
#define SAMC_CID_VALUE 0xb105100dU

/* Family parts */
typedef struct samc_part {
	uint8_t devsel;
	char pin;
	uint8_t mem;
	uint8_t variant;
} samc_part_s;


static const samc_part_s samc_20_parts[] = {
	{0x00, 'J', 18, 'A'}, /* ATSAMC20J18 */
	{0x01, 'J', 17, 'A'}, /* ATSAMC20J17 */
	{0x02, 'J', 16, 'A'}, /* ATSAMC20J16 */
	{0x03, 'J', 15, 'A'}, /* ATSAMC20J15 */
	{0x05, 'G', 18, 'A'}, /* ATSAMC20G18 */
	{0x06, 'G', 17, 'A'}, /* ATSAMC20G17 */
	{0x07, 'G', 16, 'A'}, /* ATSAMC20G16 */
	{0x08, 'G', 15, 'A'}, /* ATSAMC20G15 */
	{0x0A, 'E', 18, 'A'}, /* ATSAMC20E18 */
	{0x0B, 'E', 17, 'A'}, /* ATSAMC20E17 */
	{0x0C, 'E', 16, 'A'}, /* ATSAMC20E16 */
	{0x0D, 'E', 15, 'A'}, /* ATSAMC20E15 */
	{0xff, 0, 0, 0},      /* Sentinel entry */
};


static const samc_part_s samc_21_parts[] = {
	{0x00, 'J', 18, 'A'}, /* SAMC21J18A */
	{0x01, 'J', 17, 'A'}, /* SAMC21J17A */
	{0x02, 'J', 16, 'A'}, /* SAMC21J16A */
	{0x03, 'J', 15, 'A'}, /* SAMC21J15A */
	{0x05, 'G', 18, 'A'}, /* SAMC21G18A */
	{0x06, 'G', 17, 'A'}, /* SAMC21G17A */
	{0x07, 'G', 16, 'A'}, /* SAMC21G16A */
	{0x08, 'G', 15, 'A'}, /* SAMC21G15A */
	{0x0a, 'E', 18, 'A'}, /* SAMC21E18A */
	{0x0b, 'E', 17, 'A'}, /* SAMC21E17A */
	{0x0c, 'E', 16, 'A'}, /* SAMC21E16A */
	{0x0d, 'E', 15, 'A'}, /* SAMC21E15A */
	{0x20, 'N', 18, 'A'}, /* SAMC21N18A */
	{0x21, 'N', 17, 'A'}, /* SAMC21N17A */
	{0xff, 0, 0, 0},      /* Sentinel entry */
};


/*
 * Overloads the default cortexm reset function with a version that
 * removes the target from extended reset where required.
 */
void samc_reset(target_s *t)
{
	/*
	 * nRST is not asserted here as it appears to reset the adiv5
	 * logic, meaning that subsequent adiv5_* calls PLATFORM_FATAL_ERROR.
	 *
	 * This is ok as normally you can just connect the debugger and go,
	 * but if that's not possible (protection or SWCLK being used for
	 * something else) then having SWCLK low on reset should get you
	 * debug access (cold-plugging). TODO: Confirm this
	 *
	 * See the SAM D20 datasheet §12.6 Debug Operation for more details.
	 *
	 * jtagtap_nrst(true);
	 * jtagtap_nrst(false);
	 *
	 * XXX: Should this actually call cortexm_reset()?
	 */

	/* Read DHCSR here to clear S_RESET_ST bit before reset */
	target_mem_read32(t, CORTEXM_DHCSR);

	/*
	 * Request System Reset from NVIC: nRST doesn't work correctly
	 * This could be VECTRESET: 0x05fa0001 (reset only core)
	 *          or SYSRESETREQ: 0x05fa0004 (system reset)
	 */
	target_mem_write32(t, CORTEXM_AIRCR, CORTEXM_AIRCR_VECTKEY | CORTEXM_AIRCR_SYSRESETREQ);

	/* Exit extended reset */
	if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) & SAMC_STATUSA_CRSTEXT)
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_CRSTEXT);

	/* Poll for release from reset */
	while (target_mem_read32(t, CORTEXM_DHCSR) & CORTEXM_DHCSR_S_RESET_ST)
		continue;

	/* Reset DFSR flags */
	target_mem_write32(t, CORTEXM_DFSR, CORTEXM_DFSR_RESETALL);

	/* Clear any target errors */
	target_check_error(t);
}

/*
 * Overloads the default cortexm detached function with a version that
 * removes the target from extended reset where required.
 *
 * Only required for SAM D20 _Revision B_ Silicon
 */
static void samc20_revB_detach(target_s *t)
{
	cortexm_detach(t);

	/* Exit extended reset */
	if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) & SAMC_STATUSA_CRSTEXT)
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_CRSTEXT);
}

/*
 * Overloads the default cortexm halt_resume function with a version
 * that removes the target from extended reset where required.
 *
 * Only required for SAM D20 _Revision B_ Silicon
 */
static void samc20_revB_halt_resume(target_s *t, bool step)
{
	cortexm_halt_resume(t, step);

	/* Exit extended reset */
	if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) & SAMC_STATUSA_CRSTEXT)
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_CRSTEXT);
}

/*
 * Release the target from extended reset before running the normal cortexm_attach routine.
 * This prevents tripping up over errata ref 9905
 *
 * Only required for SAM D11 silicon.
 */
static bool samc11_attach(target_s *t)
{
	/* Exit extended reset */
	if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) & SAMC_STATUSA_CRSTEXT)
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_CRSTEXT);

	return cortexm_attach(t);
}

/*
 * Overload the default cortexm attach for when the samc is protected.
 *
 * If the samc is protected then the default cortexm attach will
 * fail as the S_HALT bit in the DHCSR will never go high. This
 * function allows users to attach on a temporary basis so they can
 * rescue the device.
 */
bool samc_protected_attach(target_s *t)
{
	tc_printf(t, "Attached in protected mode, please issue 'monitor erase_mass' to regain chip access\n");
	/* Patch back in the normal cortexm attach for next time */
	t->attach = cortexm_attach;

	/* Allow attach this time */
	return true;
}

/*
 * Use the DSU Device Identification Register to populate a struct
 * describing the SAM D device.
 */
typedef struct samc_descr {
	char family;
	uint8_t series;
	char revision;
	char pin;
	uint32_t ram_size;
	uint32_t flash_size;
	uint8_t mem;
	char variant;
	char package[3];
} samc_descr_s;

samc_descr_s samc_parse_device_id(uint32_t did)
{
	samc_descr_s samc = {0};
	const samc_part_s *parts = samc_20_parts;
	samc.ram_size = 0x8000;
	samc.flash_size = 0x40000;

	/* Family */
	const uint8_t family = (did >> SAMC_DID_FAMILY_POS) & SAMC_DID_FAMILY_MASK;

	if (family == 2)
	{
		samc.family = 'C';
	}

	/* Series */
	const uint8_t series = (did >> SAMC_DID_SERIES_POS) & SAMC_DID_SERIES_MASK;
	switch (series) {
	case 0:
		parts = samc_20_parts;
		samc.series = 20;
		break;
	case 1:
		parts = samc_21_parts;
		samc.series = 21;
		break;
	default:
		samc.series = 0;
		break;
	}

	/* Revision */
	const uint8_t revision = (did >> SAMC_DID_REVISION_POS) & SAMC_DID_REVISION_MASK;
	samc.revision = (char)('A' + revision);

	const uint8_t devsel = (did >> SAMC_DID_DEVSEL_POS) & SAMC_DID_DEVSEL_MASK;
	
	switch (samc.series) {
	case 20U: /* SAM C20 */
		switch (devsel / 5U) {
		case 0U:
			samc.pin = 'J';
			break;
		case 1U:
			samc.pin = 'G';
			break;
		case 2U:
			samc.pin = 'E';
			break;
		default:
			samc.pin = 'u';
			break;
		}
		samc.mem = 18U - (devsel % 5U);
		samc.variant = 'A';
		break;
	case 21U: /* SAM C21 */
		for (size_t i = 0; parts[i].devsel != 0xffU; ++i) {
			if (parts[i].devsel == devsel) {
				samc.pin = parts[i].pin;
				samc.mem = parts[i].mem;
				samc.variant = parts[i].variant;
				break;
			}
		}
		break;
	}
	return samc;
}

static void samc_add_flash(target_s *t, uint32_t addr, size_t length)
{
	target_flash_s *f = calloc(1, sizeof(*f));
	if (!f) { /* calloc failed: heap exhaustion */
		DEBUG_ERROR("calloc: failed in %s\n", __func__);
		return;
	}

	f->start = addr;
	f->length = length;
	f->blocksize = SAMC_ROW_SIZE;
	f->erase = samc_flash_erase;
	f->write = samc_flash_write;
	f->writesize = SAMC_PAGE_SIZE;
	target_add_flash(t, f);
}

#define samc_VARIANT_STR_LENGTH 60U

typedef struct samc_priv {
	char samc_variant_string[samc_VARIANT_STR_LENGTH];
} samc_priv_s;

bool samc_probe(target_s *t)
{
	adiv5_access_port_s *ap = cortex_ap(t);
	const uint32_t cid = adiv5_ap_read_pidr(ap, SAMC_DSU_CID);
	const uint32_t pid = adiv5_ap_read_pidr(ap, SAMC_DSU_PID);

	/* Check the ARM Coresight Component and Peripheral IDs */
	if (cid != SAMC_CID_VALUE || (pid & SAMC_PID_MASK) != SAMC_PID_CONST_VALUE)
		return false;

	/* Read the Device ID */
	const uint32_t did = target_mem_read32(t, SAMC_DSU_DID);

	/* If the Device ID matches */
	if ((did & SAMC_DID_MASK) != SAMC_DID_CONST_VALUE)
		return false;

	samc_priv_s *priv_storage = calloc(1, sizeof(*priv_storage));
	t->target_storage = priv_storage;

	const uint32_t ctrlstat = target_mem_read32(t, SAMC_DSU_CTRLSTAT);
	const samc_descr_s samc = samc_parse_device_id(did);

	/* Protected? */
	const bool protected = (ctrlstat & SAMC_STATUSB_PROT);

	snprintf(priv_storage->samc_variant_string, samc_VARIANT_STR_LENGTH, "Atmel SAM%c%02d%c%d%c%s (rev %c)%s",
		samc.family, samc.series, samc.pin, samc.mem, samc.variant, samc.package, samc.revision,
		protected ? " protected" : "");

	/* Setup Target */
	t->driver = priv_storage->samc_variant_string;
	t->reset = samc_reset;
	t->mass_erase = samc_mass_erase;

	if (samc.series == 20 && samc.revision == 'B') {
		/*
		 * These functions check for an extended reset.
		 * Appears to be related to Errata 35.4.1 ref 12015
		 */
		t->detach = samc20_revB_detach;
		t->halt_resume = samc20_revB_halt_resume;
	} else if (samc.series == 11) {
		/*
		 * Attach routine that checks for an extended reset and releases it.
		 * This works around Errata 38.2.5 ref 9905
		 */
		t->attach = samc11_attach;
	}

	if (protected) {
		/*
		 * Overload the default cortexm attach
		 * for when the samc is protected.
		 * This function allows users to
		 * attach on a temporary basis so they
		 * can rescue the device.
		 */
		t->attach = samc_protected_attach;
	}

	target_add_ram(t, 0x20000000, samc.ram_size);
	samc_add_flash(t, 0x00000000, samc.flash_size);
	target_add_commands(t, samc_cmd_list, "samc");

	/* If we're not in reset here */
	if (!platform_nrst_get_val()) {
		/* We'll have to release the target from
		 * extended reset to make attach possible */
		if (target_mem_read32(t, SAMC_DSU_CTRLSTAT) & SAMC_STATUSA_CRSTEXT)
			/* Write bit to clear from extended reset */
			target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_CRSTEXT);
	}

	return true;
}

/* Temporary (until next reset) flash memory locking */
static void samc_lock_current_address(target_s *t)
{
	/* Issue the lock command */
	target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_LOCK);
}

/* Temporary (until next reset) flash memory unlocking */
static void samc_unlock_current_address(target_s *t)
{
	/* Issue the unlock command */
	target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_UNLOCK);
}

static bool samc_wait_nvm_ready(target_s *t)
{
	/* Poll for NVM Ready */
	while ((target_mem_read32(t, SAMC_NVMC_INTFLAG) & SAMC_NVMC_READY) == 0) {
		if (target_check_error(t))
			return false;
	}
	return true;
}

static bool samc_wait_dsu_ready(target_s *const t, uint32_t *const result, platform_timeout_s *const timeout)
{
	uint32_t status = 0;
	while ((status & (SAMC_STATUSA_DONE | SAMC_STATUSA_PERR | SAMC_STATUSA_FAIL)) == 0) {
		status = target_mem_read32(t, SAMC_DSU_CTRLSTAT);
		if (target_check_error(t))
			return false;
		if (timeout)
			target_print_progress(timeout);
	}
	*result = status;
	return true;
}

/* Erase flash row by row */
static bool samc_flash_erase(target_flash_s *const f, const target_addr_t addr, const size_t len)
{
	target_s *t = f->t;
	for (size_t offset = 0; offset < len; offset += f->blocksize) {
		/*
		 * Write address of first word in row to erase it
		 * Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address
		 */
		target_mem_write32(t, SAMC_NVMC_ADDRESS, (addr + offset) >> 1U);

		/* Unlock */
		samc_unlock_current_address(t);

		/* Issue the erase command */
		target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_ERASEROW);
		if (!samc_wait_nvm_ready(t))
			return false;

		/* Lock */
		samc_lock_current_address(t);
	}

	return true;
}

/*
 * Write flash page by page
 */
static bool samc_flash_write(target_flash_s *f, target_addr_t dest, const void *src, size_t len)
{
	target_s *t = f->t;

	/* Write within a single page. This may be part or all of the page */
	target_mem_write(t, dest, src, len);

	/* Unlock */
	samc_unlock_current_address(t);

	/* Issue the write page command */
	target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_WRITEPAGE);
	if (!samc_wait_nvm_ready(t))
		return false;

	/* Lock */
	samc_lock_current_address(t);

	return true;
}

/* Uses the Device Service Unit to erase the entire flash */
bool samc_mass_erase(target_s *t)
{
	/* Clear the DSU status bits */
	target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_DONE | SAMC_STATUSA_PERR | SAMC_STATUSA_FAIL);

	/* Erase all */
	target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_CTRL_CHIP_ERASE);

	uint32_t status = 0;
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, 500);
	if (!samc_wait_dsu_ready(t, &status, &timeout))
		return false;

	/* Test the protection error bit in Status A */
	if (status & SAMC_STATUSA_PERR) {
		tc_printf(t, "Erase failed due to a protection error.\n");
		return true;
	}

	/* Test the fail bit in Status A */
	return !(status & SAMC_STATUSA_FAIL);
}

/*
 * Sets the NVM region lock bits in the User Row. This value is read
 * at startup as the default value for the lock bits, and hence does
 * not take effect until a reset.
 *
 * 0x0000 = Lock, 0xffff = Unlock (default)
 */
static bool samc_set_flashlock(target_s *t, uint16_t value, const char **argv)
{
	(void)argv;
	uint32_t high = target_mem_read32(t, SAMC_NVM_USER_ROW_HIGH);
	uint32_t low = target_mem_read32(t, SAMC_NVM_USER_ROW_LOW);

	/* Write address of a word in the row to erase it */
	/* Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address */
	target_mem_write32(t, SAMC_NVMC_ADDRESS, SAMC_NVM_USER_ROW_LOW >> 1U);

	/* Issue the erase command */
	target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_ERASEAUXROW);
	if (!samc_wait_nvm_ready(t))
		return false;

	/* Modify the high byte of the user row */
	high = (high & 0x0000ffffU) | ((value << 16U) & 0xffff0000U);

	/* Write back */
	target_mem_write32(t, SAMC_NVM_USER_ROW_LOW, low);
	target_mem_write32(t, SAMC_NVM_USER_ROW_HIGH, high);

	/* Issue the page write command */
	target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_WRITEAUXPAGE);

	return true;
}

static bool parse_unsigned(const char *str, uint32_t *val)
{
	int result;
	unsigned long num;

	size_t len = strlen(str);
	// TODO: port to use substrate::toInt_t<> style parser for robustness and smaller code size
	if (len > 2U && str[0] == '0' && (str[1] == 'x' || str[1] == 'X'))
		result = sscanf(str + 2, "%lx", &num);
	else
		result = sscanf(str, "%lu", &num);

	if (result < 1)
		return false;

	*val = (uint32_t)num;
	return true;
}

static bool samc_cmd_lock_flash(target_s *t, int argc, const char **argv)
{
	if (argc > 2) {
		tc_printf(t, "usage: monitor lock_flash [number]\n");
		return false;
	}
	if (argc == 2) {
		uint32_t val = 0;
		if (!parse_unsigned(argv[1], &val)) {
			tc_printf(t, "number must be either decimal or 0x prefixed hexadecimal\n");
			return false;
		}

		if (val > 0xffffU) {
			tc_printf(t, "number must be between 0 and 65535\n");
			return false;
		}

		return samc_set_flashlock(t, (uint16_t)val, NULL);
	}
	return samc_set_flashlock(t, 0x0000, NULL);
}

static bool samc_cmd_unlock_flash(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	return samc_set_flashlock(t, 0xffff, NULL);
}

static bool samc_set_bootprot(target_s *t, uint16_t value, const char **argv)
{
	(void)argv;
	const uint32_t high = target_mem_read32(t, SAMC_NVM_USER_ROW_HIGH);
	uint32_t low = target_mem_read32(t, SAMC_NVM_USER_ROW_LOW);

	/*
	 * Write address of a word in the row to erase it
	 * Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address
	 */
	target_mem_write32(t, SAMC_NVMC_ADDRESS, SAMC_NVM_USER_ROW_LOW >> 1U);

	/* Issue the erase command */
	target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_ERASEAUXROW);
	if (!samc_wait_nvm_ready(t))
		return false;

	/* Modify the low word of the user row */
	low = (low & 0xfffffff8U) | ((value << 0U) & 0x00000007U);

	/* Write back */
	target_mem_write32(t, SAMC_NVM_USER_ROW_LOW, low);
	target_mem_write32(t, SAMC_NVM_USER_ROW_HIGH, high);

	/* Issue the page write command */
	target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_WRITEAUXPAGE);
	return true;
}

static bool samc_cmd_lock_bootprot(target_s *t, int argc, const char **argv)
{
	/* Locks first 0x7 .. 0, 0x6 .. 512, 0x5 .. 1024, ..., 0x0 .. 32768 bytes of flash*/
	if (argc > 2) {
		tc_printf(t, "usage: monitor lock_bootprot [number]\n");
		return false;
	}
	if (argc == 2) {
		uint32_t val = 0;
		if (!parse_unsigned(argv[1], &val)) {
			tc_printf(t, "number must be either decimal or 0x prefixed hexadecimal\n");
			return false;
		}

		if (val > 7U) {
			tc_printf(t, "number must be between 0 and 7\n");
			return false;
		}

		return samc_set_bootprot(t, (uint16_t)val, NULL);
	}
	return samc_set_bootprot(t, 0, NULL);
}

static bool samc_cmd_unlock_bootprot(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	return samc_set_bootprot(t, 7, NULL);
}

static bool samc_cmd_read_userrow(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	tc_printf(t, "User Row: 0x%08" PRIx32 "%08" PRIx32 "\n", target_mem_read32(t, SAMC_NVM_USER_ROW_HIGH),
		target_mem_read32(t, SAMC_NVM_USER_ROW_LOW));

	return true;
}

/* Reads the 128-bit serial number from the NVM */
static bool samc_cmd_serial(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	tc_printf(t, "Serial Number: 0x");

	for (size_t i = 0; i < 4U; ++i)
		tc_printf(t, "%08x", target_mem_read32(t, SAMC_NVM_SERIAL(i)));
	tc_printf(t, "\n");
	return true;
}

/* Returns the size (in bytes) of the current SAM D20's flash memory. */
static uint32_t samc_flash_size(target_s *t)
{
	/* Read the Device ID */
	const uint32_t did = target_mem_read32(t, SAMC_DSU_DID);
	/* Mask off the device select bits */
	const uint8_t devsel = did & SAMC_DID_DEVSEL_MASK;
	/* Shift the maximum flash size (256KB) down as appropriate */
	return (0x40000U >> (devsel % 5U));
}

/* Runs the Memory Built In Self Test (MBIST) */
static bool samc_cmd_mbist(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	/* Write the memory parameters to the DSU */
	target_mem_write32(t, SAMC_DSU_ADDRESS, 0);
	target_mem_write32(t, SAMC_DSU_LENGTH, samc_flash_size(t));

	/* Clear the fail bit */
	target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_STATUSA_FAIL);

	/* Write the MBIST command */
	target_mem_write32(t, SAMC_DSU_CTRLSTAT, SAMC_CTRL_MBIST);

	uint32_t status = 0;
	if (!samc_wait_dsu_ready(t, &status, NULL))
		return false;

	/* Test the protection error bit in Status A */
	if (status & SAMC_STATUSA_PERR) {
		tc_printf(t, "MBIST not run due to protection error.\n");
		return true;
	}

	/* Test the fail bit in Status A */
	if (status & SAMC_STATUSA_FAIL)
		tc_printf(t, "MBIST Fail @ 0x%08" PRIx32 "\n", target_mem_read32(t, SAMC_DSU_ADDRESS));
	else
		tc_printf(t, "MBIST Passed!\n");
	return true;
}

/*
 * Sets the security bit
 */
static bool samc_cmd_ssb(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	/* Issue the ssb command */
	target_mem_write32(t, SAMC_NVMC_CTRLA, SAMC_CTRLA_CMD_KEY | SAMC_CTRLA_CMD_SSB);
	if (!samc_wait_nvm_ready(t))
		return false;

	tc_printf(t, "Security bit set!\nScan again, attach and issue 'monitor erase_mass' to reset.\n");

	target_reset(t);
	return true;
}
