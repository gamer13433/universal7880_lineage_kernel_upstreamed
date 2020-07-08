#ifndef __LINUX_GOLDFISH_H
#define __LINUX_GOLDFISH_H

/* Helpers for Goldfish virtual platform */

<<<<<<< HEAD
static inline void gf_write64(unsigned long data,
		void __iomem *portl, void __iomem *porth)
{
	writel((u32)data, portl);
#ifdef CONFIG_64BIT
	writel(data>>32, porth);
#endif
}

=======
static inline void gf_write_ptr(const void *ptr, void __iomem *portl,
				void __iomem *porth)
{
	writel((u32)(unsigned long)ptr, portl);
#ifdef CONFIG_64BIT
	writel((unsigned long)ptr >> 32, porth);
#endif
}

static inline void gf_write_dma_addr(const dma_addr_t addr,
				     void __iomem *portl,
				     void __iomem *porth)
{
	writel((u32)addr, portl);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	writel(addr >> 32, porth);
#endif
}


>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
#endif /* __LINUX_GOLDFISH_H */
