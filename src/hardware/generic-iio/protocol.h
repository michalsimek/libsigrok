/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2015-2016 BayLibre, SAS.
 * Author : Hubert Chaumette <hchaumette@baylibre.com>
 * Author : Neil Armstrong <narmstrong@baylibre.com>
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

#ifndef LIBSIGROK_HARDWARE_GENERIC_IIO_PROTOCOL_H
#define LIBSIGROK_HARDWARE_GENERIC_IIO_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <unistd.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "generic-iio"

struct channel_priv {
	struct sr_channel_group *group;
	struct sr_channel *chan;
	struct iio_channel *iio_chan;
	int mq;
	int unit;
	int unitsize;
	float scale;
	float offset;
	unsigned is_signed;
};

/** Private, per channel group context. */
struct channel_group_priv {
	struct sr_channel_group *group;

	uint64_t limit_samples;
	uint64_t limit_msec;

	uint64_t samples_read;
	/* FIXME not updated */
	uint64_t samples_missed;
	int64_t start_time;
	int64_t last_sample_fin;
	uint64_t sample_rate;
	uint64_t buffer_size;

	struct iio_device *iio_dev;
	struct iio_buffer *iio_buf;

	struct sr_dev_inst *sdi;

	struct {
		unsigned enabled;
		GMutex mutex;
		unsigned running;
		GThread *ctx;
		int pipe_fds[2];
	} poll_thread;

	unsigned channels_count;
	unsigned channels_running;
};

SR_PRIV int gen_iio_receive_data(int fd, int revents, void *cb_data);

#endif
