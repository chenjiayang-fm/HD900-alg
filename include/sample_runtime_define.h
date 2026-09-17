#ifndef __SAMPLE_RUNTIME_DEFINE_H
#define __SAMPLE_RUNTIME_DEFINE_H

#ifdef __cplusplus
extern "C"
{
#endif


#define CPU_TASK_AFFINITY "cpu_task_affinity:0 cpu_task_affinity:1 monitor_task_affinity:2"
#ifdef ON_BOARD
#define RESOURCE_DIR "./runtime/data/"
#else
#define RESOURCE_DIR "../data/"
#endif

#define MAX_ROI_NUM 300

#if DEBUG
    #define sample_debug(...) \
    do \
{ \
printf(__VA_ARGS__); \
} while (0)
#else
#define sample_debug(...)
#endif

#ifdef __cplusplus
}
#endif

#endif
