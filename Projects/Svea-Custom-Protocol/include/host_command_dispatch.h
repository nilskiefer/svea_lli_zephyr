#ifndef PROJECT_HOST_COMMAND_DISPATCH_H_
#define PROJECT_HOST_COMMAND_DISPATCH_H_

#include <stdbool.h>

/*
 * Dispatch one host command line (without trailing newline).
 * Returns true if a known command was parsed and published.
 */
bool host_command_dispatch_line(const char *line);

#endif
