idf_component_register( SRCS "src/server.c" "src/wifilog.c" "src/fs_json.c"
                        PRIV_INCLUDE_DIRS "include"
                        INCLUDE_DIRS "interface"
                        EMBED_FILES http_content/error.html http_content/log.js http_content/log.css http_content/data.html
                        REQUIRES log websocket json)

