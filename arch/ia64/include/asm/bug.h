#ifndef _ASM_IA64_BUG_H
#define _ASM_IA64_BUG_H

#ifdef CONFIG_BUG
#define ia64_abort()	__builtin_trap()
<<<<<<< HEAD
#define BUG() do { printk("kernel BUG at %s:%d!\n", __FILE__, __LINE__); ia64_abort(); } while (0)
=======
#define BUG() do {						\
	printk("kernel BUG at %s:%d!\n", __FILE__, __LINE__);	\
	barrier_before_unreachable();				\
	ia64_abort();						\
} while (0)
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

/* should this BUG be made generic? */
#define HAVE_ARCH_BUG
#endif

#include <asm-generic/bug.h>

#endif
