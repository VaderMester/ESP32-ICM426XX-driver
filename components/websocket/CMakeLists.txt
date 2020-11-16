idf_component_register( SRCS "src/websocket.c" "src/websocket_server.c"
                        PRIV_INCLUDE_DIRS "include"
                        INCLUDE_DIRS "interface"
                        REQUIRES lwip mbedtls)
