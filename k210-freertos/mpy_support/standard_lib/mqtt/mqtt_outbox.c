#include "mqtt_outbox.h"
#include <stdlib.h>
#include <string.h>
#include "syslog.h"
#include "transport.h"

static const char *TAG = "OUTBOX";

outbox_handle_t outbox_init()
{
    outbox_handle_t outbox = pvPortMalloc(sizeof(struct outbox_list_t));
    K210_MEM_CHECK(TAG, outbox, return NULL);
    memset(outbox, 0, sizeof(struct outbox_list_t));
    STAILQ_INIT(outbox);
    return outbox;
}

outbox_item_handle_t outbox_enqueue(outbox_handle_t outbox, uint8_t *data, int len, int msg_id, int msg_type, int tick)
{
    outbox_item_handle_t item = pvPortMalloc(sizeof(outbox_item_t));
    K210_MEM_CHECK(TAG, item, return NULL);
    memset(item, 0, sizeof(outbox_item_t));
    item->msg_id = msg_id;
    item->msg_type = msg_type;
    item->tick = tick;
    item->len = len;
    item->buffer = pvPortMalloc(len);
    K210_MEM_CHECK(TAG, item->buffer, {
        vPortFree(item);
        return NULL;
    });
    memcpy(item->buffer, data, len);
    STAILQ_INSERT_TAIL(outbox, item, next);
    if (transport_debug) LOGD(TAG, "ENQUEUE msgid=%d, msg_type=%d, len=%d, size=%d", msg_id, msg_type, len, outbox_get_size(outbox));
    return item;
}

outbox_item_handle_t outbox_get(outbox_handle_t outbox, int msg_id)
{
    outbox_item_handle_t item;
    STAILQ_FOREACH(item, outbox, next) {
        if (item->msg_id == msg_id) {
            return item;
        }
    }
    return NULL;
}

outbox_item_handle_t outbox_dequeue(outbox_handle_t outbox)
{
    outbox_item_handle_t item;
    STAILQ_FOREACH(item, outbox, next) {
        if (!item->pending) {
            return item;
        }
    }
    return NULL;
}
int outbox_delete(outbox_handle_t outbox, int msg_id, int msg_type)
{
    outbox_item_handle_t item, tmp;
    STAILQ_FOREACH_SAFE(item, outbox, next, tmp) {
        if (item->msg_id == msg_id && item->msg_type == msg_type) {
            STAILQ_REMOVE(outbox, item, outbox_item, next);
            vPortFree(item->buffer);
            vPortFree(item);
            if (transport_debug) LOGD(TAG, "DELETED msgid=%d, msg_type=%d, remain size=%d", msg_id, msg_type, outbox_get_size(outbox));
            return 0;
        }

    }
    return -1;
}
int outbox_delete_msgid(outbox_handle_t outbox, int msg_id)
{
    outbox_item_handle_t item, tmp;
    STAILQ_FOREACH_SAFE(item, outbox, next, tmp) {
        if (item->msg_id == msg_id) {
            STAILQ_REMOVE(outbox, item, outbox_item, next);
            vPortFree(item->buffer);
            vPortFree(item);
        }

    }
    return 0;
}
int outbox_set_pending(outbox_handle_t outbox, int msg_id)
{
    outbox_item_handle_t item = outbox_get(outbox, msg_id);
    if (item) {
        item->pending = true;
        return 0;
    }
    return -1;
}

int outbox_delete_msgtype(outbox_handle_t outbox, int msg_type)
{
    outbox_item_handle_t item, tmp;
    STAILQ_FOREACH_SAFE(item, outbox, next, tmp) {
        if (item->msg_type == msg_type) {
            STAILQ_REMOVE(outbox, item, outbox_item, next);
            vPortFree(item->buffer);
            vPortFree(item);
        }

    }
    return 0;
}

int outbox_delete_expired(outbox_handle_t outbox, int current_tick, int timeout)
{
    outbox_item_handle_t item, tmp;
    STAILQ_FOREACH_SAFE(item, outbox, next, tmp) {
        if (current_tick - item->tick > timeout) {
            STAILQ_REMOVE(outbox, item, outbox_item, next);
            vPortFree(item->buffer);
            vPortFree(item);
        }

    }
    return 0;
}

int outbox_get_size(outbox_handle_t outbox)
{
    int siz = 0;
    outbox_item_handle_t item;
    STAILQ_FOREACH(item, outbox, next) {
        siz += item->len;
    }
    return siz;
}

int outbox_cleanup(outbox_handle_t outbox, int max_size)
{
    while(outbox_get_size(outbox) > max_size) {
        outbox_item_handle_t item = outbox_dequeue(outbox);
        if (item == NULL) {
            return -1;
        }
        STAILQ_REMOVE(outbox, item, outbox_item, next);
        vPortFree(item->buffer);
        vPortFree(item);
    }
    return 0;
}

void outbox_destroy(outbox_handle_t outbox)
{
    outbox_cleanup(outbox, 0);
    vPortFree(outbox);
}
