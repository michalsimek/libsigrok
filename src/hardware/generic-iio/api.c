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

#include "protocol.h"

#include <config.h>
#include <time.h>
#include <iio.h>
#include <poll.h>
#include <string.h>

SR_PRIV struct sr_dev_driver generic_iio_driver_info;

#define DEFAULT_BUFFER_SIZE	64

static const int32_t hwopts[] = {
	SR_CONF_CONN,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS | SR_CONF_SET,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_BUFFERSIZE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET,
};

static int init(struct sr_dev_driver *di, struct sr_context *sr_ctx)
{
	return std_init(sr_ctx, di, LOG_PREFIX);
}

static int gen_iio_channel_init(struct channel_priv *chanp, const char *id)
{
	unsigned int bits; /* Sample size */
	float scale = 1; /* Not taken into account in iio_channel_convert */
	float offset = 0; /* Ditto */
	ssize_t ret;
	char buf[50];
	char sign;

	if (!chanp)
		return SR_ERR;

	sscanf(id, "%[^0-9]%*u", buf);
	sr_spew("buf = %s", buf);
	if (!strncmp(buf, "voltage", 50)
	    || !strncmp(buf, "altvoltage", 50)) {
		chanp->mq = SR_MQ_VOLTAGE;
		chanp->unit = SR_UNIT_VOLT;
		/* received in mV */
		scale /= 1000;
	} else if (!strncmp(buf, "current", 50)) {
		chanp->mq = SR_MQ_CURRENT;
		chanp->unit = SR_UNIT_AMPERE;
		/* received in mA */
		scale /= 1000;
	} else if (!strncmp(buf, "power", 50)) {
		chanp->mq = SR_MQ_POWER;
		chanp->unit = SR_UNIT_WATT;
		/* received in mW */
		scale /= 1000;
	} else if (!strncmp(buf, "timestamp", 50)) {
		chanp->mq = SR_MQ_TIME;
		chanp->unit = SR_UNIT_SECOND;
		/* received in ns */
		scale /= 1e18;
	} else if (!strncmp(buf, "resistance", 50)) {
		chanp->mq = SR_MQ_RESISTANCE;
		chanp->unit = SR_UNIT_OHM;
	} else if (!strncmp(buf, "capacitance", 50)) {
		chanp->mq = SR_MQ_CAPACITANCE;
		chanp->unit = SR_UNIT_FARAD;
		/* received in nF */
		scale /= 1e9;
	} else if (!strncmp(buf, "temp", 50)) {
		chanp->mq = SR_MQ_TEMPERATURE;
		chanp->unit = SR_UNIT_CELSIUS;
		/* received in millidegrees C */
		scale /= 1000;
	} else if (!strncmp(buf, "humidityrelative", 50)) {
		chanp->mq = SR_MQ_RELATIVE_HUMIDITY;
		chanp->unit = SR_UNIT_PERCENTAGE;
		/* received in millipercent */
		scale /= 1000;
	}
	/* Unhandled types :
	 * - pressure
	 * - accel
	 * - anglvel
	 * - magn
	 * - illuminance
	 * - intensity
	 * - proximity
	 * - incli
	 * - rot
	 * - angl
	 * - cct
	 * - activity
	 * - steps
	 * - energy
	 * - distance
	 * - velocity
	 * - concentration
	 */
	else {
		sr_err("Unknown quantity \"%s\"", buf);
		return SR_ERR;
	}

	ret = iio_channel_attr_read(chanp->iio_chan, "type", buf, sizeof(buf));
	if (ret < 0) {
		return SR_ERR;
	}

	/* Get channel value signed and size from type */
	sscanf(buf, "%*ce:%c%*u/%u>>%*u", &sign, &bits);
	chanp->is_signed = (sign == 's' || sign == 'S');
	chanp->unitsize = bits / CHAR_BIT;
	sr_dbg("IIO channel type: %s (signed: %u; bits: %u)",
	       buf, chanp->is_signed, bits);

	/* TODO handle device-wide scale/offset attributes */

	ret = iio_channel_attr_read(chanp->iio_chan, "offset", buf, sizeof(buf));
	if (ret >= 0) {
		offset += scale * atof(buf);
	}
	chanp->offset = offset;

	ret = iio_channel_attr_read(chanp->iio_chan, "scale", buf, sizeof(buf));
	if (ret >= 0) {
		scale *= atof(buf);
	}
	chanp->scale = scale;

	sr_dbg("scale  = %f", scale);
	sr_dbg("offset = %f", offset);

	return SR_OK;
}

static void gen_iio_channel_group_register(struct sr_dev_inst *sdi,
					    struct iio_device *iiodev,
					    int device_id)
{
	struct channel_group_priv *chgp = NULL;
	struct iio_channel *iiochan = NULL;
	struct channel_priv *chanp = NULL;
	struct sr_channel *chan = NULL;
	unsigned int nb_chan;
	const char *devname;
	const char *channame;
	char groupname[128];
	char probename[255];
	char temp[255];
	unsigned int i;
	const char *id;
	int ret;

	if (!iiodev)
		return;

	devname = iio_device_get_name(iiodev);
	nb_chan = iio_device_get_channels_count(iiodev);
	sr_dbg("IIO device %s has %u channel(s)", devname, nb_chan);
	if (!nb_chan)
		return;

	/* Append device id to get unique group name */
	sprintf(groupname, "%s_%d", devname, device_id);

	chgp = g_malloc0(sizeof(struct channel_group_priv));
	if (!chgp) {
		sr_err("Failed to allocate private group structure");
		return;
	}

	chgp->group = g_malloc0(sizeof(struct sr_channel_group));
	if (!chgp->group) {
		sr_err("Failed to allocate channels group structure");
		return;
	}

	chgp->iio_dev = iiodev;
	chgp->sdi = sdi;
	chgp->channels_count = nb_chan;
	chgp->buffer_size = DEFAULT_BUFFER_SIZE;

	chgp->group->name = g_strdup(groupname);
	chgp->group->priv = chgp;
	chgp->group->channels = NULL;

	/* Try to get sampling frequency, else set to 1Hz as placeholder */
	ret = iio_device_attr_read(chgp->iio_dev, "in_sampling_frequency",
				   temp, 16);
	if (ret) {
		sr_info("%s: in_sampling_frequency: %s", chgp->group->name, temp);
		chgp->sample_rate = atoi(temp);
	}
	else {
		sr_warn("%s: sampling frequency is not available, returning 1Hz in place",
				chgp->group->name);
		chgp->sample_rate = 1;
	}

	for (i = 0; i < nb_chan; i++) {
		iiochan = iio_device_get_channel(iiodev, i);
		id = iio_channel_get_id(iiochan);
		channame = iio_channel_get_name(iiochan);

		/* Register only input channels. */
		if (!iio_channel_is_scan_element(iiochan)
		    || iio_channel_is_output(iiochan)) {
			sr_dbg("Non-input channel %s ignored", id);
			continue;
		}

		/* Prepend group name to get unique channel name */
		if (channame)
			sprintf(probename, "%s_%s", groupname, channame);
		else
			sprintf(probename, "%s_%s", groupname, id);

		sr_dbg("Channel #%d id: %s name: %s dir: %s",
		       i, id, probename,
		       iio_channel_is_output(iiochan) ? "OUT" : "IN");

		chanp = g_malloc0(sizeof(struct channel_priv));
		chanp->iio_chan = iiochan;
		chanp->group = chgp->group;

		/* Setup channel type, scale and offset */
		if (gen_iio_channel_init(chanp, id) != SR_OK) {
			sr_dbg("Failed to init channel #%d", i);
			continue;
		}

		chan = sr_channel_new(sdi, i, SR_CHANNEL_ANALOG, TRUE, probename);
		chanp->chan = chan;
		chan->priv = chanp;

		chgp->group->channels = g_slist_append(chgp->group->channels, chan);
	}

	if (chgp->group->channels == NULL) {
		sr_dbg("No input channels found, ignoring IIO device %s.",
		       devname);
		goto no_channel;
	}

	sdi->channel_groups = g_slist_append(sdi->channel_groups, chgp->group);

	return;

no_channel:
	g_free(chgp);
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc = NULL;
	struct sr_dev_inst *sdi = NULL;
	struct iio_device *iiodev = NULL;
	struct sr_config *src = NULL;
	GSList *l = NULL, *devices = NULL;
	struct iio_context *iio_ctx;
	unsigned int nb_devices;
	unsigned int major;
	unsigned int minor;
	char iio_ctx_version[255];
	char iio_ctx_name[255];
	const char *str;
	unsigned int i;
	const char *conn = NULL;
	char git_tag[8];

	/* Get options */
	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		}
	}

	devices = NULL;
	drvc = di->context;

	iio_library_get_version(&major, &minor, git_tag);
	sr_dbg("LibIIO version: %u.%u (git tag: %s)", major, minor, git_tag);

	if (conn) {
		/* Iterate over xml or network context types */
		sr_dbg("Creating IIO network context on : '%s'", conn);
		iio_ctx = iio_create_xml_context(conn);
		if (!iio_ctx)
			iio_ctx = iio_create_network_context(conn);
	}
	else
		iio_ctx = iio_create_default_context();
	if (!iio_ctx) {
		sr_err("Unable to create IIO context");
		return NULL;
	}

	iio_library_get_version(&major, &minor, git_tag);
	sr_dbg("LibIIO context version: %u.%u (git tag: %s)",
		major, minor, git_tag);
	sprintf(iio_ctx_version, "v%u.%u (git tag: %s)",
		major, minor, git_tag);

	str = iio_context_get_description(iio_ctx);
	/* Get first word only */
	if (str)
		sscanf(str, "%s%*[^\n]", iio_ctx_name);
	else
		sprintf(iio_ctx_name, "%s", iio_context_get_name(iio_ctx));

	sdi = g_malloc0(sizeof(struct sr_dev_inst));
	sdi->driver = di;
	sdi->status = SR_ST_INACTIVE;
	sdi->inst_type = SR_INST_USER;
	sdi->vendor = g_strdup("Generic IIO");
	sdi->model = g_strdup(iio_ctx_name);
	sdi->version = g_strdup(iio_ctx_version);
	sdi->serial_num = g_strdup(git_tag);
	sdi->connection_id = g_strdup(str);
	sdi->priv = iio_ctx;

	nb_devices = iio_context_get_devices_count(iio_ctx);
	sr_dbg("IIO context has %u device(s)", nb_devices);

	/* Create a sr_channel_group per IIO device. */
	for (i = 0; i < nb_devices; i++) {
		iiodev = iio_context_get_device(iio_ctx, i);
		gen_iio_channel_group_register(sdi, iiodev, i);
	}

	devices = g_slist_append(devices, sdi);
	drvc->instances = g_slist_append(drvc->instances, sdi);

	return devices;
}

static GSList *dev_list(const struct sr_dev_driver *di)
{
	return ((struct drv_context *)(di->context))->instances;
}

static int dev_clear(const struct sr_dev_driver *di)
{
	struct sr_dev_inst *sdi = NULL;
	GSList *i;

	for (i = dev_list(di); i ; i = i->next) {
		sdi = i->data;

		if (sdi && sdi->priv) {
			struct iio_context *iio_ctx = sdi->priv;
			sr_dbg("Freeing IIO context %s",
					iio_context_get_description(iio_ctx));
			iio_context_destroy(iio_ctx);
			sdi->priv = NULL;
		}
	}

	return std_dev_clear(di, NULL);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	/* Nothing to do here. */
	sdi->status = SR_ST_ACTIVE;

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	/* Nothing to do here. */
	sdi->status = SR_ST_INACTIVE;

	return SR_OK;
}

static int cleanup(const struct sr_dev_driver *di)
{
	dev_clear(di);

	return SR_OK;
}

static int config_get(uint32_t id, GVariant **data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *chgs)
{
	struct channel_group_priv *chgp = NULL;
	struct sr_channel_group *chg = NULL;
	GSList *chgl;
	int ret;

	if (chgs)
		sr_dbg("%s: %s: %d", __func__, chgs->name, id);
	else
		sr_dbg("%s: <all groups>: %d", __func__, id);

	if (!chgs) {
		switch (id) {
		case SR_CONF_SAMPLERATE: {
			uint64_t srate = 0;
			for (chgl = sdi->channel_groups;
					chgl; chgl = chgl->next) {
				chg = chgl->data;
				chgp = chg->priv;

				if (srate && chgp->sample_rate != srate) {
					sr_err("Sampling rate is different among devices");
					return SR_ERR_CHANNEL_GROUP;
				}
				srate = chgp->sample_rate;
			}
			*data = g_variant_new_uint64(srate);
			return SR_OK;
		}
		case SR_CONF_BUFFERSIZE: {
			uint64_t bufsize = 0;
			for (chgl = sdi->channel_groups;
					chgl; chgl = chgl->next) {
				chg = chgl->data;
				chgp = chg->priv;

				if (bufsize && chgp->buffer_size != bufsize) {
					sr_err("Buffer size among devices, report min");
					return SR_ERR_CHANNEL_GROUP;
				}
				bufsize = chgp->buffer_size;
			}
			*data = g_variant_new_uint64(bufsize);
			return SR_OK;
		}
		case SR_CONF_LIMIT_SAMPLES:
			for (chgl = sdi->channel_groups;
					chgl; chgl = chgl->next) {
				chg = chgl->data;
				chgp = chg->priv;

				*data = g_variant_new_uint64(chgp->limit_samples);
				return SR_OK;
			}
			return SR_ERR_NA;
		case SR_CONF_LIMIT_MSEC:
			for (chgl = sdi->channel_groups;
					chgl; chgl = chgl->next) {
				chg = chgl->data;
				chgp = chg->priv;

				*data = g_variant_new_uint64(chgp->limit_msec);
				return SR_OK;
			}
			return SR_ERR_NA;
		default:
			return SR_ERR_NA;
		}
	}

	chgp = chgs->priv;

	ret = SR_OK;
	switch (id) {
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(chgp->limit_samples);
		break;
	case SR_CONF_LIMIT_MSEC:
		*data = g_variant_new_uint64(chgp->limit_msec);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(chgp->sample_rate);
		break;
	case SR_CONF_BUFFERSIZE:
		*data = g_variant_new_uint64(chgp->buffer_size);
		break;
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int config_set(uint32_t id, GVariant *data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *chgs)
{
	struct channel_group_priv *chgp = NULL;
	struct sr_channel_group *chg = NULL;
	GSList *chgl;
	int ret;

	if (chgs)
		sr_dbg("%s: %s: %d", __func__, chgs->name, id);
	else
		sr_dbg("%s: <all groups>: %d", __func__, id);

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	ret = SR_OK;
	switch (id) {
	case SR_CONF_LIMIT_SAMPLES:
		for (chgl = sdi->channel_groups; chgl; chgl = chgl->next) {
			chg = chgl->data;
			chgp = chg->priv;
			
			chgp->limit_samples = g_variant_get_uint64(data);
			chgp->limit_msec = 0;
		}
		break;
	case SR_CONF_LIMIT_MSEC:
		for (chgl = sdi->channel_groups; chgl; chgl = chgl->next) {
			chg = chgl->data;
			chgp = chg->priv;
			
			chgp->limit_msec = g_variant_get_uint64(data) * 1000;
			chgp->limit_samples = 0;
		}
		break;
	case SR_CONF_CONTINUOUS:
		for (chgl = sdi->channel_groups; chgl; chgl = chgl->next) {
			chg = chgl->data;
			chgp = chg->priv;
			
			chgp->limit_msec = 0;
			chgp->limit_samples = 0;
		}
		break;
	case SR_CONF_BUFFERSIZE:
		for (chgl = sdi->channel_groups; chgl; chgl = chgl->next) {
			chg = chgl->data;
			chgp = chg->priv;
			
			/* Basic per-device config setting */
			if (chgs && chgs != chg)
				continue;

			chgp->buffer_size = g_variant_get_uint64(data);
		}
		break;
	default:
		ret = SR_ERR_NA;
	}

	return ret;
}

static int config_list(uint32_t info_id, GVariant **data,
		       const struct sr_dev_inst *sdi,
		       const struct sr_channel_group *chg)
{
	int ret = SR_OK;

	(void)sdi;
	(void)chg;

	switch (info_id) {
	case SR_CONF_SCAN_OPTIONS:
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_INT32,
				hwopts, ARRAY_SIZE(hwopts), sizeof(int32_t));
		break;
	case SR_CONF_DEVICE_OPTIONS:
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
			devopts, ARRAY_SIZE(devopts), sizeof(uint32_t));
		break;
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static gpointer generic_iio_thread(gpointer data)
{
	struct channel_group_priv *chgp = data;
	ssize_t ret;
	int err;

	while (chgp->poll_thread.running) {
		g_mutex_lock(&chgp->poll_thread.mutex);

		if (!chgp->poll_thread.running)
			return NULL;

		ret = iio_buffer_refill(chgp->iio_buf);
		if (ret < 0) {
			sr_err("%s: %s: Unable to refill IIO buffer: %s",
				__func__, chgp->group->name, strerror(-ret));
			chgp->poll_thread.running = FALSE;
			break;
		}

		sr_dbg("%s: %s: %d bytes refilled",
				__func__, chgp->group->name, (int)ret);

		err = write(chgp->poll_thread.pipe_fds[1], &ret, sizeof(ret));
		if (err < (int)sizeof(ret)) {
			sr_warn("%s: %s: Failed to write PIPE: %m",
				__func__, chgp->group->name);
			chgp->poll_thread.running = FALSE;
			break;
		}

		g_thread_yield();
	}

	close(chgp->poll_thread.pipe_fds[1]);

	sr_dbg("Exiting %s for Group %s", __func__, chgp->group->name);

	return NULL;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct channel_group_priv *chgp = NULL;
	struct sr_channel_group *chg = NULL;
	GSList *chgl = NULL, *chpl = NULL;
	int fd = -1;
	int ret;

	(void)cb_data;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	for (chgl = sdi->channel_groups; chgl; chgl = chgl->next) {
		chg = chgl->data;
		chgp = chg->priv;
		chgp->samples_read = 0;
		chgp->samples_missed = 0;
		chgp->channels_running = 0;

		/* Enable select channels */
		for (chpl = chg->channels ; chpl ; chpl = chpl->next) {
			struct sr_channel *chan = chpl->data;
			struct channel_priv *chanp = chan->priv;

			if (!chan->enabled)
				continue;
		
			iio_channel_enable(chanp->iio_chan);

			++chgp->channels_running;
		}

		/* Skip this device if not used */
		if (!chgp->channels_running)
			continue;

		/* Allocate buffer for n multichannel samples */
		chgp->iio_buf = iio_device_create_buffer(chgp->iio_dev,
							 chgp->buffer_size,
							 false);
		if (!chgp->iio_buf) {
			sr_err("Unable to allocate IIO buffer");
			return SR_ERR;
		}

		sr_dbg("%llu samples big IIO buffer allocated",
			(long long unsigned int)chgp->buffer_size);

		fd = iio_buffer_get_poll_fd(chgp->iio_buf);
		sr_dbg("Channel Group %s fd %d", chg->name, fd);

		/* Network context does not export a poll fd */
		if (fd < 0) {
			ret = pipe(chgp->poll_thread.pipe_fds);
			if (ret < 0) {
				sr_err("Failed to create thread PIPE for Group %s : %m",
					chg->name);
				continue;
			}
			chgp->poll_thread.enabled = TRUE;
			chgp->poll_thread.running = TRUE;
			g_mutex_init(&chgp->poll_thread.mutex);
			chgp->poll_thread.ctx = g_thread_new("iio",
						generic_iio_thread, chgp);
			sr_dbg("Running new poll thread for Group %s pipe %d/%d",
					chg->name,
					chgp->poll_thread.pipe_fds[0],
					chgp->poll_thread.pipe_fds[1]);
			sr_session_source_add(sdi->session,
					      chgp->poll_thread.pipe_fds[0],
					      POLLIN, 100,
					      gen_iio_receive_data,
					      (void *)chg);
		}
		else
			sr_session_source_add(sdi->session,
					      fd, POLLIN, 100,
					      gen_iio_receive_data,
					      (void *)chg);

		chgp->start_time = g_get_monotonic_time();
	}

	/* Send header packet to the session bus. */
	std_session_send_df_header(sdi, LOG_PREFIX);

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi, void *cb_data)
{
	struct sr_datafeed_packet packet;
	struct sr_channel_group *chg = NULL;
	struct channel_group_priv *chgp = NULL;
	struct iio_buffer *iiobuf = NULL;
	GSList *chgl = NULL, *chpl = NULL;

	(void)cb_data;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	for (chgl = sdi->channel_groups; chgl; chgl = chgl->next) {
		chg = chgl->data;
		chgp = chg->priv;
		iiobuf = chgp->iio_buf;

		if (!chgp->channels_running)
			continue;

		if (chgp->poll_thread.enabled) {
			sr_session_source_remove(sdi->session,
						 chgp->poll_thread.pipe_fds[0]);
			close(chgp->poll_thread.pipe_fds[0]);
			chgp->poll_thread.running = FALSE;
			g_mutex_unlock(&chgp->poll_thread.mutex);
			g_thread_join(chgp->poll_thread.ctx);
		}
		else
			sr_session_source_remove(sdi->session,
						 iio_buffer_get_poll_fd(iiobuf));

		if (chgp->samples_missed > 0)
			sr_warn("%" PRIu64 " samples missed",
				chgp->samples_missed);

		iio_buffer_destroy(chgp->iio_buf);
		chgp->iio_buf = NULL;

		/* Disable select channels */
		for (chpl = chg->channels ; chpl ; chpl = chpl->next) {
			struct sr_channel *chan = chpl->data;
			struct channel_priv *chanp = chan->priv;

			if (!chan->enabled)
				continue;

			iio_channel_disable(chanp->iio_chan);
		}

		chgp->channels_running = 0;
	}

	/* Send last packet. */
	packet.type = SR_DF_END;
	sr_session_send(sdi, &packet);

	return SR_OK;
}

SR_PRIV struct sr_dev_driver generic_iio_driver_info = {
	.name = "generic-iio",
	.longname = "Generic IIO wrapper",
	.api_version = 1,
	.init = init,
	.cleanup = cleanup,
	.scan = scan,
	.dev_list = dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
};
