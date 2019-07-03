#include <standard_lib/include/platform_k210.h>
#include <stdlib.h>
#include <string.h>

#include "sys/queue.h"
#include "syslog.h"

#include "transport.h"

bool transport_debug = false;

static const char *TAG = "TRANSPORT";

/**
 * Transport layer structure, which will provide functions, basic properties for transport types
 */
struct transport_item_t {
    int             port;
    int             socket;         /*!< Socket to use in this transport */
    char            *scheme;        /*!< Tag name */
    void            *context;       /*!< Context data */
    void            *data;          /*!< Additional transport data */
    connect_func    _connect;       /*!< Connect function of this transport */
    io_read_func    _read;          /*!< Read */
    io_func         _write;         /*!< Write */
    trans_func      _close;         /*!< Close */
    poll_func       _poll_read;     /*!< Poll and read */
    poll_func       _poll_write;    /*!< Poll and write */
    trans_func      _destroy;       /*!< Destroy and free transport */
    STAILQ_ENTRY(transport_item_t) next;
};


/**
 * This list will hold all transport available
 */
STAILQ_HEAD(transport_list_t, transport_item_t);


transport_list_handle_t transport_list_init()
{
    transport_list_handle_t list = pvPortMalloc(sizeof(struct transport_list_t));
    K210_MEM_CHECK(TAG, list, return NULL);
    memset(list, 0, sizeof(struct transport_list_t));
    STAILQ_INIT(list);
    return list;
}

int transport_list_add(transport_list_handle_t list, transport_handle_t t, const char *scheme)
{
    if (list == NULL || t == NULL) {
        return K210_ERR_INVALID_ARG;
    }
    t->scheme = pvPortMalloc(strlen(scheme) + 1);
    K210_MEM_CHECK(TAG, t->scheme, return K210_ERR_NO_MEM);
    memset(t->scheme, 0, strlen(scheme) + 1);
    strcpy(t->scheme, scheme);
    STAILQ_INSERT_TAIL(list, t, next);
    return 0;
}

transport_handle_t transport_list_get_transport(transport_list_handle_t list, const char *scheme)
{
    if (!list) {
        return NULL;
    }
    if (scheme == NULL) {
        return STAILQ_FIRST(list);
    }
    transport_handle_t item;
    STAILQ_FOREACH(item, list, next) {
        if (strcasecmp(item->scheme, scheme) == 0) {
            return item;
        }
    }
    return NULL;
}

int transport_list_destroy(transport_list_handle_t list)
{
    transport_list_clean(list);
    vPortFree(list);
    return 0;
}

int transport_list_clean(transport_list_handle_t list)
{
    transport_handle_t item = STAILQ_FIRST(list);
    transport_handle_t tmp;
    while (item != NULL) {
        tmp = STAILQ_NEXT(item, next);
        if (item->_destroy) {
            item->_destroy(item);
        }
        transport_destroy(item);
        item = tmp;
    }
    STAILQ_INIT(list);
    return 0;
}

transport_handle_t transport_init()
{
    transport_handle_t t = pvPortMalloc(sizeof(struct transport_item_t));
    K210_MEM_CHECK(TAG, t, return NULL);
    memset(t, 0, sizeof(struct transport_item_t));
    return t;
}

int transport_destroy(transport_handle_t t)
{
    if (t->scheme) {
        vPortFree(t->scheme);
    }
    vPortFree(t);
    return 0;
}

int transport_connect(transport_handle_t t, const char *host, int port, int timeout_ms)
{
    int ret = -1;
    if (t && t->_connect) {
        return t->_connect(t, host, port, timeout_ms);
    }
    return ret;
}

int transport_read(transport_handle_t t, char *buffer, int len, int timeout_ms)
{
    if (t && t->_read) {
        return t->_read(t, buffer, len, timeout_ms);
    }
    return -1;
}

int transport_write(transport_handle_t t, const char *buffer, int len, int timeout_ms)
{
    if (t && t->_write) {
        return t->_write(t, buffer, len, timeout_ms);
    }
    return -1;
}

int transport_poll_read(transport_handle_t t, int timeout_ms)
{
    if (t && t->_poll_read) {
        return t->_poll_read(t, timeout_ms);
    }
    return -1;
}

int transport_poll_write(transport_handle_t t, int timeout_ms)
{
    if (t && t->_poll_write) {
        return t->_poll_write(t, timeout_ms);
    }
    return -1;
}

int transport_close(transport_handle_t t)
{
    if (t && t->_close) {
        return t->_close(t);
    }
    return 0;
}

void *transport_get_context_data(transport_handle_t t)
{
    if (t) {
        return t->data;
    }
    return NULL;
}

int transport_set_context_data(transport_handle_t t, void *data)
{
    if (t) {
        t->data = data;
        return 0;
    }
    return -1;
}

int transport_set_func(transport_handle_t t,
                             connect_func _connect,
                             io_read_func _read,
                             io_func _write,
                             trans_func _close,
                             poll_func _poll_read,
                             poll_func _poll_write,
                             trans_func _destroy)
{
    if (t == NULL) {
        return -1;
    }
    t->_connect = _connect;
    t->_read = _read;
    t->_write = _write;
    t->_close = _close;
    t->_poll_read = _poll_read;
    t->_poll_write = _poll_write;
    t->_destroy = _destroy;
    return 0;
}

int transport_get_default_port(transport_handle_t t)
{
    if (t == NULL) {
        return -1;
    }
    return t->port;
}

int transport_set_default_port(transport_handle_t t, int port)
{
    if (t == NULL) {
        return -1;
    }
    t->port = port;
    return 0;
}
