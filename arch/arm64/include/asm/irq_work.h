#ifndef __ASM_IRQ_WORK_H
#define __ASM_IRQ_WORK_H

<<<<<<< HEAD
#ifdef CONFIG_SMP

=======
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
#include <asm/smp.h>

static inline bool arch_irq_work_has_interrupt(void)
{
	return !!__smp_cross_call;
}

<<<<<<< HEAD
#else

static inline bool arch_irq_work_has_interrupt(void)
{
	return false;
}

#endif

=======
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
#endif /* __ASM_IRQ_WORK_H */
