/*
 * Copyright (c) 2021-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/assigned_numbers.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/lc3.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/audio/tmap.h>
#include <zephyr/bluetooth/audio/cap.h>
#include <zephyr/bluetooth/audio/mcc.h>
#include <zephyr/bluetooth/audio/vcp.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys_clock.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>

#include "stream_tx.h"
#include "audio_i2s.h"

#if defined(CONFIG_LIBLC3)
#include "lc3.h"

#define MAX_SAMPLE_RATE         48000U
#define MAX_FRAME_DURATION_US   10000U
#define MAX_NUM_SAMPLES         ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#endif

#define AVAILABLE_SINK_CONTEXT  (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
                 BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
                 BT_AUDIO_CONTEXT_TYPE_MEDIA | \
                 BT_AUDIO_CONTEXT_TYPE_GAME | \
                 BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL)

#define AVAILABLE_SOURCE_CONTEXT (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
                  BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
                  BT_AUDIO_CONTEXT_TYPE_MEDIA | \
                  BT_AUDIO_CONTEXT_TYPE_GAME)

#define BUTTON_DOUBLE_PRESS_WINDOW_MS 350U
#define BUTTON_DEBOUNCE_MS 30U

NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
              BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
              CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
    BT_AUDIO_CODEC_CAP_FREQ_48KHZ, BT_AUDIO_CODEC_CAP_DURATION_10,
    BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 120u, 1u,
    (BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));

#if defined(CONFIG_LIBLC3)
struct audio_sink_lc3 {
    lc3_decoder_t decoder;
    lc3_decoder_mem_48k_t decoder_mem;
    int frames_per_sdu;
    int octets_per_frame;
    int16_t pcm[MAX_NUM_SAMPLES];
    size_t decode_error_cnt;
    size_t audio_drop_cnt;
};
#endif

static struct bt_conn *default_conn;
static struct k_spinlock conn_lock;
static atomic_t mcs_ready = ATOMIC_INIT(0);
static atomic_t button_press_count = ATOMIC_INIT(0);
static atomic_t media_toggle_pending = ATOMIC_INIT(0);
static bool button_stable_pressed;
static struct k_work_delayable button_debounce_work;
static struct k_work_delayable button_click_work;
/* VCS volume is an unsigned level from 0 (silent) to 255 (full scale). */
static atomic_t audio_volume = ATOMIC_INIT(UINT8_MAX);
static atomic_t audio_muted = ATOMIC_INIT(BT_VCP_STATE_UNMUTED);
static struct audio_sink {
    struct bt_bap_stream stream;
    size_t recv_cnt;
    size_t lost_cnt;
    size_t error_cnt;
    size_t report_cnt;
#if defined(CONFIG_LIBLC3)
	struct audio_sink_lc3 lc3;
#endif
} sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static struct audio_source {
    struct bt_bap_stream stream;
    uint16_t seq_num;
    size_t send_cnt;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];
static size_t configured_source_stream_count;

static const struct bt_bap_qos_cfg_pref qos_pref =
    BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_1M, 0x02, 10, 40000, 40000, 40000, 40000);
    // Temporarily changed BT_GAP_LE_PHY_2M to 1M to see if streaming works 

static K_SEM_DEFINE(sem_disconnected, 0, 1);


// CUSTOM GPIO DEFINITIONS (GPIO, BUTTONS, LEDS)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec dac_sd = GPIO_DT_SPEC_GET(DT_ALIAS(dac_sd), gpios);

/* ======================================================================
 * 1. ADVERTISING DATA CONFIGURATION (EXTENDED ADVERTISING)
 * ======================================================================
 * Because the Network Core is now compiled with Extended Advertising 
 * (via sysbuild/ipc_radio.conf), the 31-byte limit no longer applies.
 * We can safely combine what used to be 'ad' and 'sd' into one large 
 * Primary Advertisement array.
 */

/* TMAP Role data: Unicast Media Receiver (UMR) and Call Terminal (CT) */
// static const uint8_t tmap_role[] = {
//     BT_UUID_16_ENCODE(BT_UUID_TMAS_VAL),
//     BT_BYTES_LIST_LE16(BT_TMAP_ROLE_UMR | BT_TMAP_ROLE_CT)
// };

/* Unicast Server Announcements (Contexts and Metadata) */
static uint8_t unicast_server_addata[] = {
    BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
    BT_AUDIO_UNICAST_ANNOUNCEMENT_GENERAL,
    BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
    BT_BYTES_LIST_LE16(AVAILABLE_SOURCE_CONTEXT),
    0x00, /* Metadata length */
};

/* 
 * The unified Extended Advertising payload. 
 * This contains over 40 bytes of data, which is perfectly legal now.
 */
static const struct bt_data ad[] = {
    /* 1. Flags: General discoverable */
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    
    /* 2. Appearance: 2066 translates to Audio Sink / Earbud */
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, BT_BYTES_LIST_LE16(CONFIG_BT_DEVICE_APPEARANCE)),
    
    /* 3. UUIDs: Broadcast ASCS and CAS so the phone knows we support LE Audio */
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_CAS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_VCS_VAL)),
                  
    /* 4. Service Data: TMAP Role and Unicast Server specifics */
    // BT_DATA(BT_DATA_SVC_DATA16, tmap_role, ARRAY_SIZE(tmap_role)),
    BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
    
    /* 5. Device Name: Can be safely expanded in prj.conf */
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
/* ====================================================================== */


void print_hex(const uint8_t *ptr, size_t len)
{
    while (len-- != 0) {
        printk("%02x", *ptr++);
    }
}

static bool print_cb(struct bt_data *data, void *user_data)
{
    const char *str = (const char *)user_data;

    printk("%s: type 0x%02x value_len %u\n", str, data->type, data->data_len);
    print_hex(data->data, data->data_len);
    printk("\n");

    return true;
}

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
    printk("codec_cfg 0x%02x cid 0x%04x vid 0x%04x count %u\n", codec_cfg->id, codec_cfg->cid,
           codec_cfg->vid, codec_cfg->data_len);

    if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
        enum bt_audio_location chan_allocation;
        int ret;

        bt_audio_data_parse(codec_cfg->data, codec_cfg->data_len, print_cb, "data");

        ret = bt_audio_codec_cfg_get_freq(codec_cfg);
        if (ret > 0) {
            printk("  Frequency: %d Hz\n", bt_audio_codec_cfg_freq_to_freq_hz(ret));
        }

        ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
        if (ret > 0) {
            printk("  Frame Duration: %d us\n",
                   bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret));
        }

        ret = bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &chan_allocation, false);
        if (ret == 0) {
            printk("  Channel allocation: 0x%x\n", chan_allocation);
        }

        printk("  Octets per frame: %d (negative means value not pressent)\n",
               bt_audio_codec_cfg_get_octets_per_frame(codec_cfg));
        printk("  Frames per SDU: %d\n",
               bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true));
    } else {
        print_hex(codec_cfg->data, codec_cfg->data_len);
    }

    bt_audio_data_parse(codec_cfg->meta, codec_cfg->meta_len, print_cb, "meta");
}

static void print_qos(const struct bt_bap_qos_cfg *qos)
{
    printk("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
           "rtn %u latency %u pd %u\n",
           qos->interval, qos->framing, qos->phy, qos->sdu,
           qos->rtn, qos->latency, qos->pd);
}

static enum bt_audio_dir stream_dir(const struct bt_bap_stream *stream)
{
    for (size_t i = 0U; i < ARRAY_SIZE(source_streams); i++) {
        if (stream == &source_streams[i].stream) {
            return BT_AUDIO_DIR_SOURCE;
        }
    }

    for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++) {
        if (stream == &sink_streams[i].stream) {
            return BT_AUDIO_DIR_SINK;
        }
    }

    __ASSERT(false, "Invalid stream %p", stream);
    return 0;
}

static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir)
{
    if (dir == BT_AUDIO_DIR_SOURCE) {
        for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
            struct bt_bap_stream *stream = &source_streams[i].stream;

            if (!stream->conn) {
                return stream;
            }
        }
    } else {
        for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++) {
            struct bt_bap_stream *stream = &sink_streams[i].stream;

            if (!stream->conn) {
                return stream;
            }
        }
    }

    return NULL;
}

// ==============================================
// LC3 CALLBACKS
// ==============================================

static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
              const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
              struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
    printk("ASE Codec Config: conn %p ep %p dir %u\n", conn, ep, dir);

    print_codec_cfg(codec_cfg);

    *stream = stream_alloc(dir);
    if (*stream == NULL) {
        printk("No streams available\n");
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

        return -ENOMEM;
    }

    printk("ASE Codec Config stream %p\n", *stream);

    if (dir == BT_AUDIO_DIR_SOURCE) {
        configured_source_stream_count++;
    }

    *pref = qos_pref;

    return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
            const struct bt_audio_codec_cfg *codec_cfg,
            struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
    printk("ASE Codec Reconfig: stream %p\n", stream);

    print_codec_cfg(codec_cfg);

    *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, BT_BAP_ASCS_REASON_NONE);
    return -ENOEXEC;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos,
           struct bt_bap_ascs_rsp *rsp)
{
    printk("QoS: stream %p qos %p\n", stream, qos);
    print_qos(qos);
    return 0;
}

static int lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
              struct bt_bap_ascs_rsp *rsp)
{
    printk("Enable: stream %p meta_len %zu\n", stream, meta_len);

#if defined(CONFIG_LIBLC3)
    {
        struct audio_sink *sink_stream = NULL;
        struct audio_sink_lc3 *lc3;
        int frame_duration_us;
        int freq;
        int ret;

        for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++) {
            if (stream == &sink_streams[i].stream) {
                sink_stream = &sink_streams[i];
                break;
            }
        }
        if (sink_stream == NULL) {
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                           BT_BAP_ASCS_REASON_CODEC_DATA);
            return -EINVAL;
        }

        lc3 = &sink_stream->lc3;
        memset(lc3, 0, sizeof(*lc3));

        ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
        if (ret > 0) {
            freq = bt_audio_codec_cfg_freq_to_freq_hz(ret);
        } else {
            printk("Error: Codec frequency not set\n");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                           BT_BAP_ASCS_REASON_CODEC_DATA);
            return ret;
        }

        ret = bt_audio_codec_cfg_get_frame_dur(stream->codec_cfg);
        if (ret > 0) {
            frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
        } else {
            printk("Error: Frame duration not set\n");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                           BT_BAP_ASCS_REASON_CODEC_DATA);
            return ret;
        }

        lc3->frames_per_sdu = bt_audio_codec_cfg_get_frame_blocks_per_sdu(
            stream->codec_cfg, true);
        lc3->octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(stream->codec_cfg);
        if (freq != 48000 || frame_duration_us != 10000) {
            printk("I2S output supports 48 kHz / 10 ms LC3 only\n");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                           BT_BAP_ASCS_REASON_CODEC_DATA);
            return -EINVAL;
        }

        if (lc3->frames_per_sdu <= 0 || lc3->octets_per_frame <= 0 ||
            ((uint32_t)frame_duration_us * (uint32_t)freq) / USEC_PER_SEC >
                MAX_NUM_SAMPLES) {
            printk("Unsupported LC3 frame configuration: frames %d octets %d\n",
                   lc3->frames_per_sdu, lc3->octets_per_frame);
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                           BT_BAP_ASCS_REASON_CODEC_DATA);
            return -EINVAL;
        }

        lc3->decoder = lc3_setup_decoder(frame_duration_us, freq, 0, &lc3->decoder_mem);

        if (lc3->decoder == NULL) {
            printk("ERROR: Failed to setup LC3 decoder\n");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                           BT_BAP_ASCS_REASON_CODEC_DATA);
            return -1;
        }
    }
#endif

    return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Start: stream %p\n", stream);

    for (size_t i = 0U; i < configured_source_stream_count; i++) {
        if (stream == &source_streams[i].stream) {
            source_streams[i].seq_num = 0U;
            break;
        }
    }

    return 0;
}

// Helper for the metadata callback
static bool data_func_cb(struct bt_data *data, void *user_data)
{
    struct bt_bap_ascs_rsp *rsp = (struct bt_bap_ascs_rsp *)user_data;

    if (!BT_AUDIO_METADATA_TYPE_IS_KNOWN(data->type)) {
        printk("Invalid metadata type %u or length %u\n", data->type, data->data_len);
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED, data->type);

        return -EINVAL;
    }

    return true;
}

static int lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
            struct bt_bap_ascs_rsp *rsp)
{
    printk("Metadata: stream %p meta_len %zu\n", stream, meta_len);
    return bt_audio_data_parse(meta, meta_len, data_func_cb, rsp);
}

static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Disable: stream %p\n", stream);
    return 0;
}

static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Stop: stream %p\n", stream);
    return 0;
}

static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Release: stream %p\n", stream);
    return 0;
}

// ========== END OF CALLBACKS ==========

static struct bt_bap_unicast_server_register_param param = {
    CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT,
    CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT
};

static const struct bt_bap_unicast_server_cb unicast_server_cb = {
    .config = lc3_config,
    .reconfig = lc3_reconfig,
    .qos = lc3_qos,
    .enable = lc3_enable,
    .start = lc3_start,
    .metadata = lc3_metadata,
    .disable = lc3_disable,
    .stop = lc3_stop,
    .release = lc3_release,
};

// STREAM HELPERS
static void stream_recv_report(struct bt_bap_stream *stream,
			       const struct bt_iso_recv_info *info,
			       struct net_buf *buf)
{
	struct audio_sink *sink_stream = CONTAINER_OF(stream, struct audio_sink, stream);

	sink_stream->report_cnt++;
	if (info->flags & BT_ISO_FLAGS_VALID) {
		sink_stream->recv_cnt++;
	}
	if (info->flags & BT_ISO_FLAGS_LOST) {
		sink_stream->lost_cnt++;
	}
	if (info->flags & BT_ISO_FLAGS_ERROR) {
		sink_stream->error_cnt++;
	}

	if ((sink_stream->report_cnt % 100U) == 0U) {
#if defined(CONFIG_LIBLC3)
         printk("ISO RX %p: valid %zu lost %zu error %zu LC3 errors %zu I2S drops %zu; last seq %u flags 0x%02x len %u\n",
               stream, sink_stream->recv_cnt, sink_stream->lost_cnt,
               sink_stream->error_cnt, sink_stream->lc3.decode_error_cnt,
             sink_stream->lc3.audio_drop_cnt,
               info->seq_num, info->flags, buf->len);
#else
		printk("ISO RX %p: valid %zu lost %zu error %zu; last seq %u flags 0x%02x len %u\n",
		       stream, sink_stream->recv_cnt, sink_stream->lost_cnt,
		       sink_stream->error_cnt, info->seq_num, info->flags, buf->len);
#endif
	}
}

#if defined(CONFIG_LIBLC3)

static void stream_recv_lc3_codec(struct bt_bap_stream *stream,
                  const struct bt_iso_recv_info *info,
                  struct net_buf *buf)
{
    struct audio_sink *sink_stream = NULL;
    struct audio_sink_lc3 *lc3;
    const bool valid_data = (info->flags & BT_ISO_FLAGS_VALID) != 0U;

    stream_recv_report(stream, info, buf);

    for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++) {
        if (stream == &sink_streams[i].stream) {
            sink_stream = &sink_streams[i];
            break;
        }
    }
    if (sink_stream == NULL) {
        return;
    }

    lc3 = &sink_stream->lc3;
    if (lc3->decoder == NULL || lc3->frames_per_sdu <= 0 || lc3->octets_per_frame <= 0) {
        return;
    }

    if (valid_data && buf->len < (size_t)(lc3->frames_per_sdu * lc3->octets_per_frame)) {
        sink_stream->error_cnt++;
        return;
    }

    for (int i = 0; i < lc3->frames_per_sdu; i++) {
        const uint8_t *frame = valid_data ? buf->data + (i * lc3->octets_per_frame) : NULL;
        const int ret = lc3_decode(lc3->decoder, frame, lc3->octets_per_frame,
                       LC3_PCM_FORMAT_S16, lc3->pcm, 1);

        if (ret < 0) {
            lc3->decode_error_cnt++;
        } else {
            const int32_t volume = atomic_get(&audio_volume);
            const bool muted = atomic_get(&audio_muted) != BT_VCP_STATE_UNMUTED;

            /* Apply VCS volume to decoded PCM before it enters the I2S queue. */
            for (size_t sample = 0U; sample < MAX_NUM_SAMPLES; sample++) {
                const int32_t scaled = muted ? 0 :
                    ((int32_t)lc3->pcm[sample] * volume) / UINT8_MAX;

                lc3->pcm[sample] = (int16_t)scaled;
            }

            if (audio_i2s_submit(lc3->pcm, MAX_NUM_SAMPLES) != 0) {
                lc3->audio_drop_cnt++;
            }
        }
    }
}

#else

static void stream_recv(struct bt_bap_stream *stream,
            const struct bt_iso_recv_info *info,
            struct net_buf *buf)
{
    stream_recv_report(stream, info, buf);
}

#endif

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
    printk("Audio Stream %p stopped with reason 0x%02X\n", stream, reason);
    if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
        audio_i2s_stop();
    }

    if (IS_ENABLED(CONFIG_BT_AUDIO_TX) && stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
        const int err = stream_tx_unregister(stream);
        if (err != 0) {
            printk("Failed to register stream %p for TX: %d\n", stream, err);
        }
    }
}

static void stream_started(struct bt_bap_stream *stream)
{
    struct bt_iso_info info;
    int err;

    err = bt_iso_chan_get_info(stream->iso, &info);
    __ASSERT(err == 0, "Failed to get ISO chan info: %d", err);

    printk("Audio Stream %p started  with CIG_ID %u and CIS_ID %u\n", stream,
           info.unicast.cig_id, info.unicast.cis_id);

    if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
        struct audio_sink *sink_stream = CONTAINER_OF(stream, struct audio_sink, stream);
        sink_stream->recv_cnt = 0U;
        sink_stream->lost_cnt = 0U;
        sink_stream->error_cnt = 0U;
        sink_stream->report_cnt = 0U;
    } else if (IS_ENABLED(CONFIG_BT_AUDIO_TX)) {
        const int err = stream_tx_register(stream);
        if (err != 0) {
            printk("Failed to register stream %p for TX: %d\n", stream, err);
        }
    }
}

static void stream_enabled_cb(struct bt_bap_stream *stream)
{
    if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
        const int err = bt_bap_stream_start(stream);
        if (err != 0) {
            printk("Failed to start stream %p: %d", stream, err);
        }
    }
}

static struct bt_bap_stream_ops stream_ops = {
#if defined(CONFIG_LIBLC3)
    .recv = stream_recv_lc3_codec,
#else
    .recv = stream_recv,
#endif
    .stopped = stream_stopped,
    .started = stream_started,
    .enabled = stream_enabled_cb,
};
// END OF STREAM HELPERS

// BLUETOOTH HELPERS
static struct bt_conn *media_conn_ref(void)
{
    k_spinlock_key_t key = k_spin_lock(&conn_lock);
    struct bt_conn *conn = default_conn;

    if (conn != NULL) {
        bt_conn_ref(conn);
    }
    k_spin_unlock(&conn_lock, key);

    return conn;
}

static bool media_conn_is_current(struct bt_conn *conn)
{
    k_spinlock_key_t key = k_spin_lock(&conn_lock);
    const bool is_current = conn != NULL && conn == default_conn;

    k_spin_unlock(&conn_lock, key);
    return is_current;
}

static void send_media_command(struct bt_conn *conn, uint8_t opcode)
{
    const struct mpl_cmd cmd = {
        .opcode = opcode,
        .use_param = false,
    };
    const int err = bt_mcc_send_cmd(conn, &cmd);

    if (err != 0) {
        printk("MCP command 0x%02x failed to queue (%d)\n", opcode, err);
    }
}

static void mcc_discover_mcs_cb(struct bt_conn *conn, int err)
{
    if (!media_conn_is_current(conn)) {
        return;
    }

    if (err != 0) {
        printk("MCP: phone MCS discovery failed (%d); media button commands unavailable\n",
               err);
        atomic_set(&mcs_ready, 0);
        return;
    }

    printk("MCP: phone MCS discovered; media button commands ready\n");
    atomic_set(&mcs_ready, 1);
}

static void mcc_media_state_cb(struct bt_conn *conn, int err, uint8_t state)
{
    if (!atomic_cas(&media_toggle_pending, 1, 0)) {
        return;
    }

    if (err != 0 || !media_conn_is_current(conn) || !atomic_get(&mcs_ready)) {
        printk("MCP: unable to read media state for play/pause (%d)\n", err);
        return;
    }

    send_media_command(conn, state == MEDIA_PROXY_STATE_PLAYING ?
                       MEDIA_PROXY_OP_PAUSE : MEDIA_PROXY_OP_PLAY);
}

static void mcc_send_cmd_cb(struct bt_conn *conn, int err, const struct mpl_cmd *cmd)
{
    if (err != 0) {
        printk("MCP: media command 0x%02x failed (%d)\n", cmd->opcode, err);
    }
}

static struct bt_mcc_cb mcc_callbacks = {
    .discover_mcs = mcc_discover_mcs_cb,
    .read_media_state = mcc_media_state_cb,
    .send_cmd = mcc_send_cmd_cb,
};

/* This handler dispatches commands to the phone, add additional commands here! */
static void button_click_work_handler(struct k_work *work)
{
    const atomic_val_t press_count = atomic_set(&button_press_count, 0);
    struct bt_conn *conn;

    ARG_UNUSED(work);

    if (press_count <= 0) {
        return;
    }

    if (!atomic_get(&mcs_ready)) {
        printk("Button media action ignored: phone MCS is not ready\n");
        return;
    }

    conn = media_conn_ref();
    if (conn == NULL) {
        printk("Button media action ignored: no phone connection\n");
        return;
    }

    if (press_count == 1) {
        atomic_set(&media_toggle_pending, 1);
        const int err = bt_mcc_read_media_state(conn);

        if (err != 0) {
            atomic_set(&media_toggle_pending, 0);
            printk("MCP media-state read failed (%d)\n", err);
        }
    } else {
        send_media_command(conn, MEDIA_PROXY_OP_NEXT_TRACK);
    }

    bt_conn_unref(conn);
}

static void button_click_work_init(void)
{
    k_work_init_delayable(&button_click_work, button_click_work_handler);
}

static void button_debounce_work_handler(struct k_work *work)
{
    int pressed = gpio_pin_get_dt(&button);

    ARG_UNUSED(work);

    if (pressed < 0) {
        printk("Could not read button state (%d)\n", pressed);
        return;
    }

    /* Accept a transition only after the pin has remained stable for the debounce interval. */
    if ((pressed != 0) == button_stable_pressed) {
        return;
    }

    button_stable_pressed = pressed != 0;
    gpio_pin_set_dt(&led, button_stable_pressed);

    if (button_stable_pressed) {
        atomic_inc(&button_press_count);
        (void)k_work_reschedule(&button_click_work,
                                K_MSEC(BUTTON_DOUBLE_PRESS_WINDOW_MS));
        printk("button pressed\n");
    } else {
        printk("button released\n");
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    k_spinlock_key_t key;

    if (err != 0) {
        printk("Failed to connect to %s %u %s\n", bt_conn_dst_str(conn),
               err, bt_hci_err_to_str(err));
        return;
    }

    printk("Connected: %s\n", bt_conn_dst_str(conn));
    key = k_spin_lock(&conn_lock);
    default_conn = bt_conn_ref(conn);
    k_spin_unlock(&conn_lock, key);

    atomic_set(&mcs_ready, 0);
    atomic_set(&media_toggle_pending, 0);
    err = bt_mcc_discover_mcs(conn, false);
    if (err != 0) {
        printk("MCP: could not start phone MCS discovery (%d)\n", err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    k_spinlock_key_t key = k_spin_lock(&conn_lock);
    struct bt_conn *disconnected_conn = NULL;

    if (conn == default_conn) {
        disconnected_conn = default_conn;
        default_conn = NULL;
    }
    k_spin_unlock(&conn_lock, key);

    if (disconnected_conn == NULL) {
        return;
    }

    printk("Disconnected: %s, reason 0x%02x %s\n", bt_conn_dst_str(conn),
           reason, bt_hci_err_to_str(reason));

    bt_conn_unref(disconnected_conn);
    atomic_set(&mcs_ready, 0);
    atomic_set(&media_toggle_pending, 0);

    k_sem_give(&sem_disconnected);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static struct bt_pacs_cap cap_sink = {
    .codec_cap = &lc3_codec_cap,
};

static struct bt_pacs_cap cap_source = {
    .codec_cap = &lc3_codec_cap,
};

static int set_location(void)
{
    int err;
    // Used for stereo audio, sets which location is used for headset audio
    if (IS_ENABLED(CONFIG_BT_PAC_SNK_LOC)) {
        err = bt_pacs_set_location(BT_AUDIO_DIR_SINK,
                       (BT_AUDIO_LOCATION_FRONT_LEFT |
                        BT_AUDIO_LOCATION_FRONT_RIGHT));
        if (err != 0) {
            printk("Failed to set sink location (err %d)\n", err);
            return err;
        }
    }
    // Used for stereo mic, currently only supports one location
    if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC)) {
        err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE,
                       BT_AUDIO_LOCATION_FRONT_CENTER);
        if (err != 0) {
            printk("Failed to set source location (err %d)\n", err);
            return err;
        }
    }

    printk("Location successfully set\n");
    return 0;
}

static int set_supported_contexts(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_PAC_SNK)) {
        err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK,
                             AVAILABLE_SINK_CONTEXT);
        if (err != 0) {
            printk("Failed to set sink supported contexts (err %d)\n", err);
            return err;
        }
    }

    if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
        err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE,
                             AVAILABLE_SOURCE_CONTEXT);
        if (err != 0) {
            printk("Failed to set source supported contexts (err %d)\n", err);
            return err;
        }
    }

    printk("Supported contexts successfully set\n");
    return 0;
}

static int set_available_contexts(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_PAC_SNK)) {
        err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK,
                             AVAILABLE_SINK_CONTEXT);
        if (err != 0) {
            printk("Failed to set sink available contexts (err %d)\n", err);
            return err;
        }
    }

    if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
        err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE,
                             AVAILABLE_SOURCE_CONTEXT);
        if (err != 0) {
            printk("Failed to set source available contexts (err %d)\n", err);
            return err;
        }
    }

    printk("Available contexts successfully set\n");
    return 0;
}
// END OF BLUETOOTH HELPERS

// ====== VOLUME (VCP) CALLBACK ======
static void vcp_state_cb(struct bt_conn *conn, int err, uint8_t volume, uint8_t mute)
{
    if (err != 0) {
        printk("VCP state error %d\n", err);
        return;
    }

    atomic_set(&audio_volume, volume);
    atomic_set(&audio_muted, mute);
    printk("VCP volume updated: %u/255, %s\n", volume,
           mute == BT_VCP_STATE_MUTED ? "muted" : "unmuted");
}

static struct bt_vcp_vol_rend_cb vcp_callbacks = {
    .state = vcp_state_cb,
};
// ===================================

// ========= BUTTON CALLBACK =========
static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* GPIO interrupts can chatter; debounce it until the level settles. */
    (void)k_work_reschedule(&button_debounce_work, K_MSEC(BUTTON_DEBOUNCE_MS));
}
int button_led_init(void)
{
	if (!gpio_is_ready_dt(&button) || !gpio_is_ready_dt(&led)) {
		return -ENODEV;
	}
    int err = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (err != 0) {
        return err;
    }
    err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (err != 0) {
        return err;
    }
    int pressed = gpio_pin_get_dt(&button);
    if (pressed < 0) {
        return pressed;
    }
    button_stable_pressed = pressed != 0;
    gpio_pin_set_dt(&led, button_stable_pressed);
    k_work_init_delayable(&button_debounce_work, button_debounce_work_handler);
    button_click_work_init();
	gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
    err = gpio_add_callback(button.port, &button_cb);
    if (err != 0) {
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    if (err != 0) {
        return err;
    }
    return 0;
}
// ===================================

/* 
* ======================================================================
* MAIN FUNCTION
* ======================================================================
*/

int main(void)
{
    /* Force DAC Shutdown pin active for testing */
    gpio_pin_configure_dt(&dac_sd, GPIO_OUTPUT_ACTIVE); // force the dac on
    // gpio_pin_set_dt(&led, 1); // TEST: LED should be forced on at boot

    struct bt_le_ext_adv *adv;
    const struct bt_pacs_register_param pacs_param = {
        .snk_pac = true,
        .snk_loc = true,
        .src_pac = true,
        .src_loc = true,
    };
    int err;

    /* Initialize button and led gpios */
    err = button_led_init();
    if (err != 0) {
        printk("Button and LED init failed (err%d)\n", err);
        return 0;
    }

    /* Configure I2S before advertising the matching 48 kHz LC3 capability. */
    err = audio_i2s_init();
    if (err != 0) {
        printk("I2S audio output init failed (err %d)\n", err);
        return 0;
    }

    /* Initialize Bluetooth stack */
    err = bt_enable(NULL);
    if (err != 0) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }
    printk("Bluetooth initialized\n");

    /* Restore bond keys and identity data before making the device discoverable. */
    err = settings_load();
    if (err != 0) {
        printk("Bluetooth settings load failed (err %d)\n", err);
        return 0;
    }

    /* Initialize the Media Control Client once; discover the phone's MCS per connection. */
    err = bt_mcc_init(&mcc_callbacks);
    if (err != 0) {
        printk("MCP client initialization failed (err %d)\n", err);
        return 0;
    }

    /* Expose VCS so the phone can control the rendered audio volume. */
    {
        struct bt_vcp_vol_rend_register_param vcp_param = {
            .step = 1U,
            .mute = BT_VCP_STATE_UNMUTED,
            .volume = UINT8_MAX,
            .cb = &vcp_callbacks,
        };

        err = bt_vcp_vol_rend_register(&vcp_param);
        if (err != 0) {
            printk("VCP volume renderer registration failed (err %d)\n", err);
            return 0;
        }
    }
    
    /* Register Telephony and Media Audio Profile (TMAP) */
    // err = bt_tmap_register(BT_TMAP_ROLE_UMR | BT_TMAP_ROLE_CT);
    // if (err != 0) {
    //     printk("Failed to register TMAP (err %d)\n", err);
    //     return 0;
    // }

    /* Register Published Audio Capabilities (PACS) */
    err = bt_pacs_register(&pacs_param);
    if (err) {
        printk("Could not register PACS (err %d)\n", err);
        return 0;
    }

    if (IS_ENABLED(CONFIG_BT_AUDIO_TX)) {
        stream_tx_init();
    }

    /* Register the Unicast Server callbacks and capabilities */
    bt_bap_unicast_server_register(&param);
    bt_bap_unicast_server_register_cb(&unicast_server_cb);

    bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);
    bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);

    // Register callbacks and number of incoming streams
    for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++) {
        bt_bap_stream_cb_register(&sink_streams[i].stream, &stream_ops);
    }

    // Register callbacks and number of outgoing streams
    for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
        bt_bap_stream_cb_register(&source_streams[i].stream, &stream_ops);
    }

    /* Configure Bluetooth and Audio hardware mappings */
    err = set_location();
    if (err != 0) return 0;

    err = set_supported_contexts();
    if (err != 0) return 0;

    err = set_available_contexts();
    if (err != 0) return 0;

    /* 
     * CREATE ADVERTISING SET
     * Using BT_BAP_ADV_PARAM_CONN_QUICK requests an Extended Connectable 
     * advertising set, optimized for BAP devices. 
     */
    err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_CONN_QUICK, NULL, &adv);
    if (err) {
        printk("Failed to create advertising set (err %d)\n", err);
        return 0;
    }

    /* 
     * SET ADVERTISING DATA
     * We pass our unified 'ad' array.
     * We pass 'NULL, 0' for the Scan Response because Extended Connectable 
     * advertising legally cannot have a scan response, and we no longer 
     * need it to bypass the 31-byte limit anyway.
     */
    err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Failed to set advertising data (err %d)\n", err);
        return 0;
    }

    /* Main Application Loop */
    while (true) {
        err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
        if (err) {
            printk("Failed to start advertising set (err %d)\n", err);
            return 0;
        }

        printk("Advertising successfully started\n");

        /* Sleep until a device connects and then disconnects */
        err = k_sem_take(&sem_disconnected, K_FOREVER);
        if (err != 0) {
            printk("failed to take sem_disconnected (err %d)\n", err);
            return 0;
        }

        /* Reset the stream counts for the next connection */
        configured_source_stream_count = 0U;
    }
    return 0;
}