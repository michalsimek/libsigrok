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

#include <config.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <glib/gstdio.h>
#include <iio.h>

#include "protocol.h"

SR_PRIV int gen_iio_receive_data(int fd, int revents, void *cb_data)
{
	struct channel_group_priv *chgp = NULL;
	struct sr_channel_group *chg = NULL;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_packet framep;
	struct sr_datafeed_analog_old analog;
	uint32_t samples_to_read;
	uint32_t samples_remaining;
	uint32_t tot_samples;
	GSList *chpl = NULL;
	size_t samples_count;
	float *fbuf = NULL;
	void *ibuf = NULL;
	unsigned sample;
	size_t datasize;
	ssize_t refill;
	int ret;

	(void)fd;
	(void)revents;

	chg = cb_data;
	if (!chg)
		return TRUE;

	chgp = chg->priv;
	if (!chgp)
		return TRUE;

	if (chgp->poll_thread.enabled) {
		ret = read(chgp->poll_thread.pipe_fds[0], &refill, sizeof(ssize_t));
		if (ret < (int)sizeof(ssize_t)) {
			sr_err("%s: Unable to get refill from pipe: %m", chg->name);
			return TRUE;
		}
	} else {
		sr_dbg("refilling...");
		refill = iio_buffer_refill(chgp->iio_buf);
		if (refill < 0) {
			sr_err("%s: Unable to refill IIO buffer: %s",
			       chg->name, strerror(-refill));
			return TRUE;
		}
	}
	sr_dbg("%s: %ld bytes read from buffer", chg->name, refill);

	tot_samples = refill / iio_device_get_sample_size(chgp->iio_dev);
	if (chgp->limit_samples > 0) {
		samples_remaining = chgp->limit_samples - chgp->samples_read;
		samples_to_read = MIN(tot_samples, samples_remaining);
	} else {
		samples_to_read = tot_samples;
	}

	framep.type = SR_DF_FRAME_BEGIN;
	sr_session_send(chgp->sdi, &framep);

	fbuf = g_malloc0_n(tot_samples, sizeof(float));
	ibuf = g_malloc0_n(samples_to_read, sizeof(int64_t));
	for (chpl = chg->channels ; chpl ; chpl = chpl->next) {
		struct sr_channel *chan = chpl->data;
		struct channel_priv *chanp = chan->priv;

		if (!chan->enabled)
			continue;

		datasize = chanp->unitsize * samples_to_read;
		if (datasize <= 0)
			continue;

		packet.type = SR_DF_ANALOG_OLD;
		packet.payload = &analog;
		analog.channels = g_slist_append(NULL, chan);
		analog.mq = chanp->mq;
		analog.unit = chanp->unit;
		analog.mqflags = 0;

		ret = iio_channel_read(chanp->iio_chan, chgp->iio_buf,
					ibuf, datasize);
		if (ret < 0) {
			sr_err("%s: failed to read samples", chan->name);
			goto failed_read;
		}
		
		samples_count = ret / chanp->unitsize;

		if (samples_count != samples_to_read)
			sr_warn("%s: Expected %d samples, got %d samples",
					chan->name,
					samples_to_read,
					(int)samples_count);

		for (sample = 0 ; sample < samples_count ; sample++) {
			void *ptr = ibuf + (sample * chanp->unitsize);
			float *f = &fbuf[sample];

			switch(chanp->unitsize) {
			case 1:
				if (chanp->is_signed)
					*f = (float)(*(int8_t *)ptr);
				else
					*f = (float)(*(uint8_t *)ptr);
				break;
			case 2:
				if (chanp->is_signed)
					*f = (float)(*(int16_t *)ptr);
				else
					*f = (float)(*(uint16_t *)ptr);
				break;
			case 4:
				if (chanp->is_signed)
					*f = (float)(*(int32_t *)ptr);
				else
					*f = (float)(*(uint32_t *)ptr);
				break;
			case 8:
				if (chanp->is_signed)
					*f = (float)(*(int64_t *)ptr);
				else
					*f = (float)(*(uint64_t *)ptr);
				break;
			default:
				sr_err("%s: Invalid sample length %d",
					chan->name, chanp->unitsize);
				return -1;
			}

			*f *= chanp->scale;
			*f += chanp->offset;
		}

		analog.data = fbuf;
		analog.num_samples = samples_count;

		sr_session_send(chgp->sdi, &packet);

		chgp->samples_read += samples_count;
	}

	if (chgp->poll_thread.enabled)
		g_mutex_unlock(&chgp->poll_thread.mutex);

	framep.type = SR_DF_FRAME_END;
	sr_session_send(chgp->sdi, &framep);

	if (chgp->limit_samples > 0 &&
	    chgp->samples_read >= chgp->limit_samples) {
		sr_info("Requested number of samples reached.");
		chgp->sdi->driver->dev_acquisition_stop(chgp->sdi, cb_data);
		chgp->last_sample_fin = g_get_monotonic_time();
		return TRUE;
	} else if (chgp->limit_msec > 0) {
		uint64_t cur_time = g_get_monotonic_time();
		uint64_t elapsed_time = cur_time - chgp->start_time;

		if (elapsed_time >= chgp->limit_msec) {
			sr_info("Sampling time limit reached.");
			chgp->sdi->driver->dev_acquisition_stop(chgp->sdi, cb_data);
			chgp->last_sample_fin = g_get_monotonic_time();
			return TRUE;
		}
	}

	chgp->last_sample_fin = g_get_monotonic_time();

	return TRUE;

failed_read:
	g_free(ibuf);
	g_free(fbuf);

	return FALSE;
}
