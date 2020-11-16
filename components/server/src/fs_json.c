#include <stdio.h>
#include <dirent.h> 
#include <errno.h>
#include <esp_system.h>         //<-- configuration
#include "esp_log.h"            //<-- log
#include <string.h>
#include <sys/stat.h>
#include "fs_json.h"
//#include "intrusive_list.h"

#define snprintf_nowarn(...) (snprintf(__VA_ARGS__) < 0 ? abort() : (void)0)

static const char TAG[] = "fs_json";

static void fs_flist_push(FLIST_t list, File_t *file);
static char *fs_getContents(char *filepath);

/************ HELPER FUNCTIONS *****************/
pathVect_t fs_path_split(char * path)
{
	const char slash[2] = "/";
	//char *saved = path;
	char *tok;

	char **parts = malloc(strlen(path)-1);
	tok = strtok(path, slash);
	uint i = 0;
   /* walk through other tokens */
   	while( tok != NULL ) {
		parts[i] = malloc(strlen(tok));
		strcpy(parts[i], tok);
      	tok = strtok(NULL, slash);
		i++;
   	}
	pathVect_t ret;
	ret.parts = parts;
	ret.numparts = i+1;
	for(int j = 0; j < ret.numparts; j++)
	{
		ESP_LOGI(TAG, "%s", ret.parts[j]);
	}
	return ret;
}

void fs_parts_free(pathVect_t path)
{
	for(int i = 0; i < path.numparts; i++)
	{
		free(path.parts[i]);
	}
	free(path.parts);
}

static void fs_flist_push(FLIST_t list, File_t *file)
{
	for(int i = 0; i < LISTSIZE; i++)
	{
		if(strlen(list.file[i]->m_path) == 0)
		{
			list.file[i] = file;
			return;
		}
	}
}

/*
void fs_flist_delete(FLIST_t list)
{
	for(int i = 0; i < LISTSIZE, i++)
	{
		list.file[i]->m_path = NULL;
		list.file[i]->m_type = NULL;
	}
}
*/

FLIST_t fs_getDirectoryContents(char *path)
{
	FLIST_t ret;
	for(int i = 0; i<LISTSIZE; i++) {
		ret.file[i] = NULL;
	}
	DIR *pDir = opendir(path);
	if (pDir != NULL)
	{
		struct dirent *pDirent;
		ESP_LOGD(TAG, "Directory dump of %s", path);
		while ((pDirent = readdir(pDir)) != NULL)
		{
			File_t file;
			//snprintf(file.m_path, PATHLEN, "%s%c%s", path, '/', pDirent->d_name);
    		snprintf_nowarn(file.m_path, PATHLEN, "%s%c%s", path, '/', pDirent->d_name);
			file.m_type = pDirent->d_type;
			fs_flist_push(ret, &file);
		}
	}
	else
	{
		ESP_LOGE(TAG, "getDirectoryContents: Unable to open directory: %s [errno=%d]", path, errno);
	}
	closedir(pDir);
	return ret;
}

static char *fs_getContents(char *filepath)
{
	uint32_t fsize;
	char *data = NULL;
	FILE *f = fopen(filepath, "r");
	if (NULL != f)
	{
		fseek(f, 0L, SEEK_END);
		fsize = ftell(f);
		rewind(f);
		data = malloc(fsize);
		ESP_LOGI(TAG, "%d bytes were read", fread(data, 1, fsize, f));
		fclose(f);
		ESP_LOGD(TAG, "File:: getContent(), path=%s, length=%d", filepath, fsize);
		if (fsize == 0)
			return NULL;
		if (data == NULL)
		{
			ESP_LOGE(TAG, "getContent: Failed to allocate memory");
			return NULL;
		}
	}
	return data;
}

	/**************** END OF HELPER FUNCTIONS *****************/

	cJSON *fs_get_json_content(char *path)
	{
		// Stat the file to make sure it is there and to determine what kind of a file
		// it is.
		struct stat statBuf;
		int rc = stat(path, &statBuf);

		cJSON *obj = cJSON_CreateObject();
		if (rc == -1)
		{
			ESP_LOGE(TAG, "Failed to stat file %s, errno=%s", path, strerror(errno));
			//obj.setInt("errno", errno);
			cJSON_AddItemToObject(obj, "errno", cJSON_CreateNumber((double)errno));
			return obj;
		}
		//obj.setString("path", path);
		cJSON_AddItemToObject(obj, "path", cJSON_CreateString(path));
		pathVect_t pathparts = fs_path_split(path);
		//obj.setString("name", parts[parts.numparts-1]);
		cJSON_AddItemToObject(obj, "name", cJSON_CreateString(pathparts.parts[pathparts.numparts-1]));
		fs_parts_free(pathparts);
		// Do one thing if the file is a regular file.
		if (S_ISREG(statBuf.st_mode))
		{
			//obj.setBoolean("directory", false);
			cJSON_AddItemToObject(obj, "directory", cJSON_CreateFalse());
			File_t file;
			snprintf(file.m_path, PATHLEN, "%s", path);
			char *data = fs_getContents(path);
			//obj.setString("data", data);
			cJSON_AddItemToObject(obj, "data", cJSON_CreateString(data));
	}
	// Do a different thing if the file is a directory.
	else if (S_ISDIR(statBuf.st_mode)) {
		//obj.setBoolean("directory", true);
		cJSON_AddItemToObject(obj, "directory", cJSON_CreateTrue());

	} // End ... found a directory.
	
	return obj;
} // FILESYSTEM_GET_JSON_CONTENT


/**
 * @brief Get the content of the specified path as a JSON object.
 * @param [in] path The path to access.
 * @return A JSON object containing what was found at the path.
 */
cJSON* fs_get_json_directory(char *path, bool isRecursive) {
	ESP_LOGD(TAG, "FILESYSTEM_GET_JSON_DIRECTORY: path is %s", path);

	// Stat the file to make sure it is there and to determine what kind of a file
	// it is.
	struct stat statBuf;
	int rc = stat(path, &statBuf);

	//JsonObject obj = JSON::createObject();
	cJSON *obj = cJSON_CreateObject();
	if (rc == -1) {
		ESP_LOGE(TAG, "Failed to stat file %s, errno=%s", path, strerror(errno));
		//obj.setInt("errno", errno);
		cJSON_AddItemToObject(obj, "errno", cJSON_CreateNumber((double) errno));
		return obj;
	}
	//obj.setString("path", path);
	cJSON_AddItemToObject(obj, "path", cJSON_CreateString(path));
	pathVect_t pathparts = fs_path_split(path);
	//obj.setString("name", parts[parts.numparts-1]);
	cJSON_AddItemToObject(obj, "name", cJSON_CreateString(pathparts.parts[pathparts.numparts-1]));
	fs_parts_free(pathparts);

	// Do one thing if the file is a regular file.
	if (S_ISREG(statBuf.st_mode)) {
			//obj.setBoolean("directory", false);
			cJSON_AddItemToObject(obj, "directory", cJSON_CreateFalse());
	}
	// Do a different thing if the file is a directory.
	else if (S_ISDIR(statBuf.st_mode)) {
		//obj.setBoolean("directory", true);
		cJSON_AddItemToObject(obj, "directory", cJSON_CreateTrue());
		cJSON *dirArray = cJSON_CreateArray();
		//obj.setArray("dir", dirArray);
		cJSON_AddItemToObject(obj, "dir", dirArray);

		ESP_LOGD(TAG, "Processing directory: %s", path);
		FLIST_t files = fs_getDirectoryContents(path);
		//std::vector<File> files = FileSystem::getDirectoryContents(path);

		// Now that we have the list of files in the directory, iterator over them adding them
		// to our array of entries.
		for (int i = 0; i < LISTSIZE; i++)
		{
			if (strlen(files.file[i]->m_path) > 0)
			{
				if (isRecursive)
				{
					char *fpath = malloc(snprintf(NULL, 0, "%s%c%s", path, '/', files.file[i]->m_path));
					sprintf(fpath, "%s%c%s", path, '/', files.file[i]->m_path);
					cJSON *dirEntry = fs_get_json_directory(fpath, isRecursive);
					//dirArray.addObject(dirEntry);
					cJSON_AddItemToArray(dirArray, dirEntry);
				}
				else
				{
					//JsonObject dirEntry = JSON::createObject();
					cJSON *dirEntry = cJSON_CreateObject();
					//dirEntry.setString("name", (*it).getName());
					cJSON_AddItemToObject(dirEntry, "name", cJSON_CreateString(files.file[i]->m_path));
					//dirEntry.setInt("type", (*it).getType());
					cJSON_AddItemToObject(dirEntry, "type", cJSON_CreateNumber((double)(files.file[i]->m_type)));
					//dirArray.addObject(dirEntry);
					cJSON_AddItemToArray(dirArray, dirEntry);
				}
			}
		} // End ... for each entry in the directory.
	}	  // End ... found a directory.
	return obj;
} // FILESYSTEM_GET_JSON_DIRECTORY

