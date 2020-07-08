#ifndef __ASM_ARM_SUSPEND_H
#define __ASM_ARM_SUSPEND_H

struct sleep_save_sp {
	u32 *save_ptr_stash;
	u32 save_ptr_stash_phys;
};

extern void cpu_resume(void);
<<<<<<< HEAD
=======
extern void cpu_resume_no_hyp(void);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
extern int cpu_suspend(unsigned long, int (*)(unsigned long));

#endif
