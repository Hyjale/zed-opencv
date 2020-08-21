#include <time.h>
#include <assert.h>
#include <stdio.h>
#include <iostream>

#define MILLI 1000
#define MICRO MILLI*MILLI
#define NANO MICRO*MILLI
// const long long MILLI = 1000;
// const long long MICRO = MILLI*MILLI;
// const long long NANO  = MICRO*MILLI;

static long long timespec_sub(struct timespec* a, struct timespec* b) {
	return (a->tv_sec - b->tv_sec) * NANO + (a->tv_nsec - b->tv_nsec);
}

typedef struct {
	const char* name;
	clockid_t clock_id;
	struct timespec start;
	struct timespec stop;
	long long diff;
} timer;

static void start_timer(timer* timer) {
	int r = clock_gettime(timer->clock_id, &timer->start);
	assert(!r);
}

static void stop_timer(timer* timer) {
	// assert timer was already started
	assert(timer->start.tv_nsec != 0 || timer->start.tv_sec != 0);

	int r = clock_gettime(timer->clock_id, &timer->stop);
	assert(!r);

	timer->diff = timespec_sub(&timer->stop, &timer->start);
}

static void print_timer(timer* timer) {
	assert(timer->name);
	int r = printf("cpu_timer,%s,%llu\n", timer->name, timer->diff);
	assert(r);
}
