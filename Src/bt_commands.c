
#include "bt_commands.h"

const char bt_command [][15] = {
{'C','O','M','+','P','W','O','S','\r','\n'},	//"COM+PWOS\r\n"
{'C','O','M','+','P','W','O','S','\r','\n'},	//"COM+PWDS\r\n"
{'C','O','M','+','R','E','B','O','O','T','\r','\n'}		//"COM+REBOOT\r\n"
};

char *c[]={0,"COM+PWOS\r\n","COM+PWDS\r\n","COM+REBOOT\r\n"};//так не работает
bt_commandsTypeDef btCommand = {(const char*)bt_command};
