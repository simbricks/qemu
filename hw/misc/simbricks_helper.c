#include "qemu/osdep.h"
#include "hw/misc/simbricks_helper.h"
#include "hw/core/cpu.h"
#include "exec/cpu-common.h"
#include "system/cpus.h"

/* Thread-local storage guarantees safety across multiple vCPUs */
static __thread uintptr_t simbricks_current_retaddr = 0;

void simbricks_set_tcg_retaddr(uintptr_t retaddr) {
    simbricks_current_retaddr = retaddr;
}

uintptr_t simbricks_get_tcg_retaddr(void) {
    return simbricks_current_retaddr;
}

void simbricks_suspend_cpu(CPUState *cpu) {
    assert(cpu);
    
    cpu->halted = 1;
    cpu->exception_index = EXCP_HALTED;
    
    uintptr_t retaddr = simbricks_get_tcg_retaddr();
    
    if (retaddr != 0) {
        /* Safely syncs guest state up to the exact instruction boundary */
        cpu_loop_exit_restore(cpu, retaddr);
    } else {
        cpu_loop_exit(cpu);
    }
}

void simbricks_resume_cpu(CPUState *cpu) {
    assert(cpu);
    cpu->halted = 0;
    cpu->exception_index = -1;
}