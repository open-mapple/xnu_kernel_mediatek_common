/*
 * MT6781 process sampler: where is a daemon stuck?
 *
 * The phone has no debugger and daemons log nowhere readable. Every
 * interval a kernel thread prints, for each process named in boot-arg
 * mp_proc_sample (comma separated, default "SpringBoard,backboardd"; "-" = off)
 * and for each of its threads, the user pc/lr/sp and a frame-pointer
 * backtrace as "SMP name[pid] tN pc ... lr ... bt ..." (header line with the
 * shared region slide and the main binary's mach header, to symbolicate
 * with tools/sweet-crash.py). A thread blocked in a syscall shows the
 * syscall stub in pc and its caller in lr/bt.
 *
 * First sample 90 s after start, then every 120 s, 40 samples at most.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/proc_internal.h>
#include <kern/task.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>

extern task_t proc_task(proc_t);
extern int hz;
extern void proc_sampler_start(void);

static char proc_sampler_names[128];

static bool
proc_sampler_wanted(const char *name)
{
	size_t n = strlen(name);
	for (const char *t = proc_sampler_names; *t;) {
		const char *end = strchr(t, ',');
		size_t tl = end ? (size_t)(end - t) : strlen(t);
		if (tl == n && n > 0 && strncmp(t, name, n) == 0) {
			return true;
		}
		if (!end) {
			break;
		}
		t = end + 1;
	}
	return false;
}

extern void proc_sampler_task(const char *name, int pid, task_t task);  /* osfmk/kern/exception.c */

static int
proc_sampler_proc(proc_t p, __unused void *arg)
{
	char name[32];
	proc_name(proc_pid(p), name, sizeof(name));
	if (proc_sampler_wanted(name)) {
		task_t task = proc_task(p);
		if (task != TASK_NULL) {
			proc_sampler_task(name, proc_pid(p), task);
		}
	}
	return PROC_RETURNED;
}

static void
proc_sampler_thread(__unused void *arg, __unused wait_result_t wr)
{
	static int chan;
	if (!PE_parse_boot_argn_alias("mp_proc_sample", "sweetsample", proc_sampler_names, sizeof(proc_sampler_names))) {
		strlcpy(proc_sampler_names, "SpringBoard,backboardd", sizeof(proc_sampler_names));
	}
	if (proc_sampler_names[0] == '-') {
		return;
	}
	(void)tsleep(&chan, PZERO, "procsmp", 90 * hz);
	for (int i = 0; i < 40; i++) {
		printf("SMP --- sample %d\n", i);
		proc_iterate(PROC_ALLPROCLIST, proc_sampler_proc, NULL, NULL, NULL);
		(void)tsleep(&chan, PZERO, "procsmp", 120 * hz);
	}
}

void
proc_sampler_start(void)
{
	thread_t thread;
	if (kernel_thread_start(proc_sampler_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
