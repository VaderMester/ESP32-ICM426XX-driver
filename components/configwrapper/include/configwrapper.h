#ifndef __CONFIGWRAPPER_C__
#define __CONFIGWRAPPER_C__


typedef enum {
    STATUS_ERROR            = 0x0,
    STATUS_FILE_MISSING     = 0x1,
    STATUS_FILE_IO_ERROR    = 0x2,
    STATUS_MEM_ALLOC_ERROR  = 0x3,
    STATUS_JSON_PARSE_ERROR = 0x4,
    STATUS_OK               = 0x5
} configwrapper_status_t;


configwrapper_status_t configwrapper_loadconfig(const char *filepath);
const char *getStartJSfile(void);


#endif /* __CONFIGWRAPPER_C__ */