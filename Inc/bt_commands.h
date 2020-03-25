#include <stdint.h>

#ifndef __BT_COMMANDS_H__
#define __BT_COMMANDS_H__

typedef struct {
	const char *data; /*!< Pointer to data font data array */
} bt_commandsTypeDef;


extern bt_commandsTypeDef btCommand;
//extern char *c;

#endif // __BT_COMMANDS_H__
