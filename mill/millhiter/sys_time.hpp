#include <sys/time.h>

#ifndef SYS_TIME_H
#define SYS_TIME_H

static struct timeval tv;
static struct timezone tz;

typedef unsigned long long Sys_time;

/* get local system time (us) */
const Sys_time get_time();

/* transfer delta microsecond into delta second (us -> s) */
const double get_delta_t(Sys_time now, Sys_time past);


#endif