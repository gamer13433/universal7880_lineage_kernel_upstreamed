#include <linux/kernel.h>
#include <linux/jump_label.h>

#include "insn.h"
#include "patch.h"

#ifdef HAVE_JUMP_LABEL

static void __arch_jump_label_transform(struct jump_entry *entry,
					enum jump_label_type type,
					bool is_static)
{
	void *addr = (void *)entry->code;
	unsigned int insn;

	if (type == JUMP_LABEL_ENABLE)
		insn = arm_gen_branch(entry->code, entry->target);
	else
		insn = arm_gen_nop();

	if (is_static)
<<<<<<< HEAD
		__patch_text(addr, insn);
=======
		__patch_text_early(addr, insn);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	else
		patch_text(addr, insn);
}

void arch_jump_label_transform(struct jump_entry *entry,
			       enum jump_label_type type)
{
	__arch_jump_label_transform(entry, type, false);
}

void arch_jump_label_transform_static(struct jump_entry *entry,
				      enum jump_label_type type)
{
	__arch_jump_label_transform(entry, type, true);
}

#endif
