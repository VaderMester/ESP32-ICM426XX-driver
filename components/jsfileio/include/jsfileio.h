#ifndef __JSFILEIO_H__
#define __JSFILEIO_H__


#include <duktape.h>


extern void jsfileio_init(duk_context *ctx);

duk_ret_t cubeos_read_file(duk_context *ctx);
duk_ret_t cubeos_write_file(duk_context *ctx);

#endif /* __JSFILEIO_H__ */