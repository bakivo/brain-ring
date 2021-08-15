#ifndef GRAPHIJ_TREES_H
#define GRAPHIJ_TREES_H

typedef enum {
    LOST_CHILD,
    FOUND_PARENT
} message_type_t;

typedef struct {
    message_type_t message_type;
    uint8_t level;
} data_from_child_t;

typedef enum {
    MESH_MESSAGE,
    MESH_TREE_UPDATE,
    MESH_USER_DATA
} data_type_t;

typedef struct {
    uint8_t mac[6];
    uint8_t parent_mac[6];
    uint8_t level;
} child_info_t;

typedef struct {
    data_type_t type;
    union {
        char message[20];
        child_info_t child_info;
        int param1;
    };
} packet_t;

#endif //GRAPHIJ_TREES_H
