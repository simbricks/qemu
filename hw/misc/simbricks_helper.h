#ifndef SIMBRICKS_HELPER_H
#define SIMBRICKS_HELPER_H

#include "qemu/osdep.h"

/*
 * SimBricks TCG Synchronization Helpers to bridge QEMU's Tiny Code Generator
 * (TCG) execution engine with SimBricks' synchronous, lockstep MMIO 
 * requirements.
 */

/* Forward declaration. */
typedef struct CPUState CPUState;

/**
 * TCG Return Address Tracking
 * 
 * During synchronous MMIO operations, we must violently exit the TCG execution loop
 * to wait for the simulator. To prevent guest register corruption, 
 * we capture the exact host assembly instruction pointer (retaddr / ra) right 
 * before the memory dispatch occurs. 
 * 
 * These functions safely stash and retrieve this state using thread-local 
 * storage (__thread) to guarantee thread safety across multiple vCPUs.
 * 
 * For there usage see the functions 'int_st_mmio_leN' (called before mmio write)
 * and 'int_ld_mmio_beN' (called before mmio read) in accel/tcg/cputlb.c.
 */

/* Captures the active TCG JIT host return address for the current vCPU thread. */
void simbricks_set_tcg_retaddr(uintptr_t retaddr);

/* Retrieves the active TCG JIT host return address for the current vCPU thread. */
uintptr_t simbricks_get_tcg_retaddr(void);


/**
 * Unified CPU Suspend / Resume Wrappers
 */

/**
 * simbricks_suspend_cpu() - Safely pauses a vCPU mid-instruction.
 * 
 * Places the CPU into QEMU's internal pseudo-halted state (EXCP_HALTED) so time
 * can natively advance without injecting guest OS faults.
 * 
 * It consumes the saved TCG return address to natively unwind and synchronize
 * the guest OS registers before yielding control to the main loop.
 * 
 * @cpu: The vCPU thread to suspend.
 */
void simbricks_suspend_cpu(CPUState *cpu);

/**
 * simbricks_resume_cpu() - Wakes a suspended vCPU.
 * 
 * Clears the artificial halted state and wipes the pseudo-exception, ensuring
 * the TCG engine boots up completely clean.
 * 
 * @cpu: The vCPU thread to resume.
 */
void simbricks_resume_cpu(CPUState *cpu);

#endif /* SIMBRICKS_HELPER_H */