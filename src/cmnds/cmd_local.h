#ifndef __CMD_LOCAL_H__
#define __CMD_LOCAL_H__

#include "cmd_public.h"

typedef struct command_s {
	const char *name;
	const char *argsFormat;
	commandHandler_t handler;
	const char *userDesc;
	const void *context;
	struct command_s *next;
} command_t;

command_t *CMD_Find(const char *name);
// for autocompletion?
void CMD_ListAllCommands(void *userData, void (*callback)(command_t *cmd, void *userData));
int get_cmd(const char *s, char *dest, int maxlen, int stripnum);
bool isWhiteSpace(char ch);


float CMD_EvaluateExpression(const char *s, const char *stop);
int CMD_If(const void *context, const char *cmd, const char *args, int cmdFlags);

#endif // __CMD_LOCAL_H__









