#ifndef _ARM_KERNEL_PATCH_H
#define _ARM_KERNEL_PATCH_H

void patch_text(void *addr, unsigned int insn);
<<<<<<< HEAD
void __patch_text(void *addr, unsigned int insn);
=======
void __patch_text_real(void *addr, unsigned int insn, bool remap);

static inline void __patch_text(void *addr, unsigned int insn)
{
	__patch_text_real(addr, insn, true);
}

static inline void __patch_text_early(void *addr, unsigned int insn)
{
	__patch_text_real(addr, insn, false);
}
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

#endif
