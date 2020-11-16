#ifndef __FS_JSON_H__
#define __FS_JSON_H__

#include <stdio.h>
#include <esp_system.h>
#include "cJSON.h"

#define LISTSIZE 128
#define PATHLEN 128

typedef struct pathVect {
	char **parts;
	uint numparts;
} pathVect_t;

typedef struct File {
	char m_path[PATHLEN];
	uint8_t m_type;
} File_t;

typedef struct FLIST_t {
	File_t *file[LISTSIZE];
} FLIST_t;

pathVect_t fs_path_split(char * path);
void fs_parts_free(pathVect_t parts);
FLIST_t fs_getDirectoryContents(char *path);

cJSON* fs_get_json_content(char *path);
cJSON* fs_get_json_directory(char *path, bool isRecursive);

#endif /* __SERVER_H__ */
