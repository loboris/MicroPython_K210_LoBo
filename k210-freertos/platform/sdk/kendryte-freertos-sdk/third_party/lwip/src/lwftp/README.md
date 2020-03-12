# Overview

A lightweight FTP client using raw API of LWIP

This client is designed as a state machine with a very low level
interface. It can be used as a library to build smarter clients with
more features.

The state machine supports 4 basic operations:
* connect
* store
* retrieve
* close

LWFTP requires the remote server to support binary transfer and
passive connection.

#### Notes on data callbacks

There is no storage back-end. The requester provides a callback to
source data.
* The callback is called once with a non null pointer to (char*), to be
used by the actual storage backend to write the location of data. The
return value is the length of available data, limited to the value of
argument maxlen.
* When the callback is called with a NULL pointer to (char*), the maxlen
argument is the number of bytes successfully sent since last call. This
shall be used by the storage backend as an acknowledge.

# Asynchronous example
```
static void ftp_retr_callback(void *arg, int result)
{
    lwftp_session_t *s = (lwftp_session_t)arg;

    if ( result != LWFTP_RESULT_OK ) {
        LOG_ERROR("retr failed (%d)", result);
        return lwftp_close(s);
    }
    // Test is done
    lwftp_close(s);
}

static uint data_sink(void *arg, const char* ptr, uint len)
{
    static const uint mylen = 12345;
    static char * const myconfig = (char*)0x20000000;
    static uint offset = 0;

    if (ptr) {
        len = min( len, mylen-offset );
        memcpy( myconfig+offset, ptr, len );
        offset += len;
    }
    return len;
}

static void ftp_stor_callback(void *arg, int result)
{
    lwftp_session_t *s = (lwftp_session_t)arg;
    err_t error;

    if ( result != LWFTP_RESULT_OK ) {
        LOG_ERROR("stor failed (%d)", result);
        return lwftp_close(s);
    }
    // Continue with RETR request
    s->data_sink = data_sink;
    s->done_fn = ftp_retr_callback;
    s->remote_path = "configfile";
    error = lwftp_retrieve(s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        LOG_ERROR("lwftp_retrieve failed (%d)", error);
    }
    // FTP session will continue with RETR and sink callbacks
}

static uint data_source(void *arg, const char** pptr, uint maxlen)
{
    static const uint mylen = 12345;
    static const char * const mydata = (char*)0x20000000;
    static uint offset = 0;
    uint len = 0;

    // Check for data request or data sent notice
    if (pptr) {
        len = mylen - offset;
        if ( len > maxlen ) len = maxlen;
        *pptr = mydata + offset;
    } else {
        offset += maxlen;
        if ( offset > mylen ) offset = mylen;
    }
    return len;
}

static void ftp_connect_callback(void *arg, int result)
{
    lwftp_session_t *s = (lwftp_session_t)arg;
    err_t error;

    if ( result != LWFTP_RESULT_LOGGED ) {
        LOG_ERROR("login failed (%d)", result);
        return lwftp_close(s);
    }
    // Continue with STOR request
    s->data_source = data_source;
    s->done_fn = ftp_stor_callback;
    s->remotepath = "logfile";
    error = lwftp_store(s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        LOG_ERROR("lwftp_store failed (%d)", error);
    }
    // FTP session will continue with STOR and source callbacks
}

static void ftp_test(void)
{
    static lwftp_session_t s;   // static content for the whole FTP session
    err_t error;

    // Initialize session data
    memset(&s, 0, sizeof(s));
    IP4_ADDR(&s.server_ip, 192,168,0,31);
    s.server_port = 21;
    s.done_fn = ftp_connect_callback;
    s.user = "username";
    s.pass = "password";
    // We have no extra user data, simply use the session structure
    s.handle = &s;

    // Start the connection state machine
    error = lwftp_connect(&s);
    if ( error != LWFTP_RESULT_INPROGRESS ) {
        LOG_ERROR("lwftp_connect failed (%d)", error);
    }
    // FTP session will continue with the connection callback
}
```
