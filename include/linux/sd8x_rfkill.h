/*
 * rfkill power contorl for Marvell sd8xxx wlan/bt
 *
 * Copyright (C) 2009 Marvell, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _LINUX_SD8X_RFKILL_H
#define _LINUX_SD8X_RFKILL_H

#include <linux/rfkill.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

struct sd8x_rfkill_platform_data {
	int gpio_power_down;
	int gpio_reset;

	struct rfkill *wlan_rfkill;
	struct rfkill *bt_rfkill;

	/*for issue mmc card_detection interrupt*/
	void *mmc_data;
	irq_handler_t detect_irq;
};

int sd8x_sdh_init(struct device *dev,
	irq_handler_t detect_irq, void *data);

void* add_sd8x_rfkill_device(int gpio_power_down, int gpio_reset);
#endif
