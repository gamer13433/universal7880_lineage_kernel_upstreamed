/*
 * Copyright (c) 2008-2009 Patrick McHardy <kaber@trash.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Development of this code funded by Astaro AG (http://www.astaro.com/)
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/seqlock.h>
#include <linux/netlink.h>
#include <linux/netfilter.h>
#include <linux/netfilter/nf_tables.h>
#include <net/netfilter/nf_tables.h>

struct nft_counter {
	seqlock_t	lock;
	u64		bytes;
	u64		packets;
};

static void nft_counter_eval(const struct nft_expr *expr,
			     struct nft_data data[NFT_REG_MAX + 1],
			     const struct nft_pktinfo *pkt)
{
	struct nft_counter *priv = nft_expr_priv(expr);

	write_seqlock_bh(&priv->lock);
	priv->bytes += pkt->skb->len;
	priv->packets++;
	write_sequnlock_bh(&priv->lock);
}

static int nft_counter_dump(struct sk_buff *skb, const struct nft_expr *expr)
{
	struct nft_counter *priv = nft_expr_priv(expr);
	unsigned int seq;
	u64 bytes;
	u64 packets;

	do {
		seq = read_seqbegin(&priv->lock);
		bytes	= priv->bytes;
		packets	= priv->packets;
	} while (read_seqretry(&priv->lock, seq));

	if (nla_put_be64(skb, NFTA_COUNTER_BYTES, cpu_to_be64(bytes)))
		goto nla_put_failure;
	if (nla_put_be64(skb, NFTA_COUNTER_PACKETS, cpu_to_be64(packets)))
		goto nla_put_failure;
	return 0;

nla_put_failure:
	return -1;
}

static const struct nla_policy nft_counter_policy[NFTA_COUNTER_MAX + 1] = {
	[NFTA_COUNTER_PACKETS]	= { .type = NLA_U64 },
	[NFTA_COUNTER_BYTES]	= { .type = NLA_U64 },
};

static int nft_counter_init(const struct nft_ctx *ctx,
			    const struct nft_expr *expr,
			    const struct nlattr * const tb[])
{
	struct nft_counter *priv = nft_expr_priv(expr);

	if (tb[NFTA_COUNTER_PACKETS])
	        priv->packets = be64_to_cpu(nla_get_be64(tb[NFTA_COUNTER_PACKETS]));
	if (tb[NFTA_COUNTER_BYTES])
		priv->bytes = be64_to_cpu(nla_get_be64(tb[NFTA_COUNTER_BYTES]));

	seqlock_init(&priv->lock);
	return 0;
}

static int nft_counter_clone(struct nft_expr *dst, const struct nft_expr *src)
{
	struct nft_counter_percpu_priv *priv = nft_expr_priv(src);
	struct nft_counter_percpu_priv *priv_clone = nft_expr_priv(dst);
	struct nft_counter_percpu __percpu *cpu_stats;
	struct nft_counter_percpu *this_cpu;
	struct nft_counter total;

	nft_counter_fetch(priv->counter, &total);

	cpu_stats = __netdev_alloc_pcpu_stats(struct nft_counter_percpu,
					      GFP_ATOMIC);
	if (cpu_stats == NULL)
		return ENOMEM;

	preempt_disable();
	this_cpu = this_cpu_ptr(cpu_stats);
	this_cpu->counter.packets = total.packets;
	this_cpu->counter.bytes = total.bytes;
	preempt_enable();

	priv_clone->counter = cpu_stats;
	return 0;
}

static struct nft_expr_type nft_counter_type;
static const struct nft_expr_ops nft_counter_ops = {
	.type		= &nft_counter_type,
	.size		= NFT_EXPR_SIZE(sizeof(struct nft_counter)),
	.eval		= nft_counter_eval,
	.init		= nft_counter_init,
	.dump		= nft_counter_dump,
	.clone		= nft_counter_clone,
};

static struct nft_expr_type nft_counter_type __read_mostly = {
	.name		= "counter",
	.ops		= &nft_counter_ops,
	.policy		= nft_counter_policy,
	.maxattr	= NFTA_COUNTER_MAX,
	.owner		= THIS_MODULE,
};

static int __init nft_counter_module_init(void)
{
	return nft_register_expr(&nft_counter_type);
}

static void __exit nft_counter_module_exit(void)
{
	nft_unregister_expr(&nft_counter_type);
}

module_init(nft_counter_module_init);
module_exit(nft_counter_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Patrick McHardy <kaber@trash.net>");
MODULE_ALIAS_NFT_EXPR("counter");
