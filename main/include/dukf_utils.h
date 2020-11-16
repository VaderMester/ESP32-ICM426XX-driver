/*
 * dukf_utils.h
 *
 *  Created on: Dec 17, 2016
 *      Author: kolban
 */

#if !defined(MAIN_DUKF_UTILS_H_)
#define MAIN_DUKF_UTILS_H_
#include "duktape.h"
#include <stdint.h>
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

void        dukf_addRunAtStart(const char *fileName);
uint32_t    dukf_get_free_heap_size();
//void        dukf_init_nvs_values();
//const char *dukf_loadFileFromESPFS(const char *path, size_t *fileSize);
//char       *dukf_loadFileFromPosix(const char *path, size_t *fileSize);
void        dukf_log_heap(const char *tag);
void        dukf_runAtStart(duk_context *ctx);
int        dukf_runFile(duk_context *ctx, const char *fileName);
duk_ret_t   dukf_loadFile_buff(duk_context *ctx, const char *fileName);
int dukf_loadFile_ctx(duk_context *ctx, const char *fileName);
void dukf_modSearch_register(duk_context *ctx);
//int dukf_init_ctx(duk_context *ctx, xSemaphoreHandle xContextSem, StaticSemaphore_t xContextSemBuf);
//int dukf_init_ctx(duk_context *ctx);
void dukf_destruct_ctx(duk_context *ctx);

void dukf_fs_init(void);
void dukf_fs_info(void);
void dukf_showLogo(void);
void dukf_modSearch_register(duk_context *ctx);
void dukf_print_register(duk_context *ctx);

#endif /* MAIN_DUKF_UTILS_H_ */
