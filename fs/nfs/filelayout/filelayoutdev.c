/*
 *  Device operations for the pnfs nfs4 file layout driver.
 *
 *  Copyright (c) 2002
 *  The Regents of the University of Michigan
 *  All Rights Reserved
 *
 *  Dean Hildebrand <dhildebz@umich.edu>
 *  Garth Goodson   <Garth.Goodson@netapp.com>
 *
 *  Permission is granted to use, copy, create derivative works, and
 *  redistribute this software and such derivative works for any purpose,
 *  so long as the name of the University of Michigan is not used in
 *  any advertising or publicity pertaining to the use or distribution
 *  of this software without specific, written prior authorization. If
 *  the above copyright notice or any other identification of the
 *  University of Michigan is included in any copy of any portion of
 *  this software, then the disclaimer below must also be included.
 *
 *  This software is provided as is, without representation or warranty
 *  of any kind either express or implied, including without limitation
 *  the implied warranties of merchantability, fitness for a particular
 *  purpose, or noninfringement.  The Regents of the University of
 *  Michigan shall not be liable for any damages, including special,
 *  indirect, incidental, or consequential damages, with respect to any
 *  claim arising out of or in connection with the use of the software,
 *  even if it has been or is hereafter advised of the possibility of
 *  such damages.
 */

#include <linux/nfs_fs.h>
#include <linux/vmalloc.h>
#include <linux/module.h>

#include "../internal.h"
#include "../nfs4session.h"
#include "filelayout.h"

#define NFSDBG_FACILITY		NFSDBG_PNFS_LD

static unsigned int dataserver_timeo = NFS4_DEF_DS_TIMEO;
static unsigned int dataserver_retrans = NFS4_DEF_DS_RETRANS;

/*
 * Data server cache
 *
 * Data servers can be mapped to different device ids.
 * nfs4_pnfs_ds reference counting
 *   - set to 1 on allocation
 *   - incremented when a device id maps a data server already in the cache.
 *   - decremented when deviceid is removed from the cache.
 */
static DEFINE_SPINLOCK(nfs4_ds_cache_lock);
static LIST_HEAD(nfs4_data_server_cache);

/* Debug routines */
void
print_ds(struct nfs4_pnfs_ds *ds)
{
	if (ds == NULL) {
		printk("%s NULL device\n", __func__);
		return;
	}
	printk("        ds %s\n"
		"        ref count %d\n"
		"        client %p\n"
		"        cl_exchange_flags %x\n",
		ds->ds_remotestr,
		atomic_read(&ds->ds_count), ds->ds_clp,
		ds->ds_clp ? ds->ds_clp->cl_exchange_flags : 0);
}

static bool
same_sockaddr(struct sockaddr *addr1, struct sockaddr *addr2)
{
	struct sockaddr_in *a, *b;
	struct sockaddr_in6 *a6, *b6;

	if (addr1->sa_family != addr2->sa_family)
		return false;

	switch (addr1->sa_family) {
	case AF_INET:
		a = (struct sockaddr_in *)addr1;
		b = (struct sockaddr_in *)addr2;

		if (a->sin_addr.s_addr == b->sin_addr.s_addr &&
		    a->sin_port == b->sin_port)
			return true;
		break;

	case AF_INET6:
		a6 = (struct sockaddr_in6 *)addr1;
		b6 = (struct sockaddr_in6 *)addr2;

		/* LINKLOCAL addresses must have matching scope_id */
		if (ipv6_addr_src_scope(&a6->sin6_addr) ==
		    IPV6_ADDR_SCOPE_LINKLOCAL &&
		    a6->sin6_scope_id != b6->sin6_scope_id)
			return false;

		if (ipv6_addr_equal(&a6->sin6_addr, &b6->sin6_addr) &&
		    a6->sin6_port == b6->sin6_port)
			return true;
		break;

	default:
		dprintk("%s: unhandled address family: %u\n",
			__func__, addr1->sa_family);
		return false;
	}

	return false;
}

static bool
_same_data_server_addrs_locked(const struct list_head *dsaddrs1,
			       const struct list_head *dsaddrs2)
{
	struct nfs4_pnfs_ds_addr *da1, *da2;

	/* step through both lists, comparing as we go */
	for (da1 = list_first_entry(dsaddrs1, typeof(*da1), da_node),
	     da2 = list_first_entry(dsaddrs2, typeof(*da2), da_node);
	     da1 != NULL && da2 != NULL;
	     da1 = list_entry(da1->da_node.next, typeof(*da1), da_node),
	     da2 = list_entry(da2->da_node.next, typeof(*da2), da_node)) {
		if (!same_sockaddr((struct sockaddr *)&da1->da_addr,
				   (struct sockaddr *)&da2->da_addr))
			return false;
	}
	if (da1 == NULL && da2 == NULL)
		return true;

	return false;
}

/*
 * Lookup DS by addresses.  nfs4_ds_cache_lock is held
 */
static struct nfs4_pnfs_ds *
_data_server_lookup_locked(const struct list_head *dsaddrs)
{
	struct nfs4_pnfs_ds *ds;

	list_for_each_entry(ds, &nfs4_data_server_cache, ds_node)
		if (_same_data_server_addrs_locked(&ds->ds_addrs, dsaddrs))
			return ds;
	return NULL;
}

/*
 * Create an rpc connection to the nfs4_pnfs_ds data server
 * Currently only supports IPv4 and IPv6 addresses
 */
static int
nfs4_ds_connect(struct nfs_server *mds_srv, struct nfs4_pnfs_ds *ds)
{
	struct nfs_client *clp = ERR_PTR(-EIO);
	struct nfs4_pnfs_ds_addr *da;
	int status = 0;

	dprintk("--> %s DS %s au_flavor %d\n", __func__, ds->ds_remotestr,
		mds_srv->nfs_client->cl_rpcclient->cl_auth->au_flavor);

	list_for_each_entry(da, &ds->ds_addrs, da_node) {
		dprintk("%s: DS %s: trying address %s\n",
			__func__, ds->ds_remotestr, da->da_remotestr);

		clp = nfs4_set_ds_client(mds_srv->nfs_client,
					(struct sockaddr *)&da->da_addr,
					da->da_addrlen, IPPROTO_TCP,
					dataserver_timeo, dataserver_retrans);
		if (!IS_ERR(clp))
			break;
	}

	if (IS_ERR(clp)) {
		status = PTR_ERR(clp);
		goto out;
	}

	status = nfs4_init_ds_session(clp, mds_srv->nfs_client->cl_lease_time);
	if (status)
		goto out_put;

	smp_wmb();
	ds->ds_clp = clp;
	dprintk("%s [new] addr: %s\n", __func__, ds->ds_remotestr);
out:
	return status;
out_put:
	nfs_put_client(clp);
	goto out;
}

static void
destroy_ds(struct nfs4_pnfs_ds *ds)
{
	struct nfs4_pnfs_ds_addr *da;

	dprintk("--> %s\n", __func__);
	ifdebug(FACILITY)
		print_ds(ds);

	if (ds->ds_clp)
		nfs_put_client(ds->ds_clp);

	while (!list_empty(&ds->ds_addrs)) {
		da = list_first_entry(&ds->ds_addrs,
				      struct nfs4_pnfs_ds_addr,
				      da_node);
		list_del_init(&da->da_node);
		kfree(da->da_remotestr);
		kfree(da);
	}

	kfree(ds->ds_remotestr);
	kfree(ds);
}

void
nfs4_fl_free_deviceid(struct nfs4_file_layout_dsaddr *dsaddr)
{
	struct nfs4_pnfs_ds *ds;
	int i;

	nfs4_print_deviceid(&dsaddr->id_node.deviceid);

	for (i = 0; i < dsaddr->ds_num; i++) {
		ds = dsaddr->ds_list[i];
		if (ds != NULL)
			nfs4_pnfs_ds_put(ds);
	}
	kfree(dsaddr->stripe_indices);
	kfree(dsaddr);
}

/* Decode opaque device data and return the result */
struct nfs4_file_layout_dsaddr *
nfs4_fl_alloc_deviceid_node(struct nfs_server *server, struct pnfs_device *pdev,
		gfp_t gfp_flags)
{
	int i;
	u32 cnt, num;
	u8 *indexp;
	__be32 *p;
	u8 *stripe_indices;
	u8 max_stripe_index;
	struct nfs4_file_layout_dsaddr *dsaddr = NULL;
	struct xdr_stream stream;
	struct xdr_buf buf;
	struct page *scratch;
	struct list_head dsaddrs;
	struct nfs4_pnfs_ds_addr *da;

	/* set up xdr stream */
	scratch = alloc_page(gfp_flags);
	if (!scratch)
		goto out_err;

	xdr_init_decode_pages(&stream, &buf, pdev->pages, pdev->pglen);
	xdr_set_scratch_buffer(&stream, page_address(scratch), PAGE_SIZE);

	/* Get the stripe count (number of stripe index) */
	p = xdr_inline_decode(&stream, 4);
	if (unlikely(!p))
		goto out_err_free_scratch;

	cnt = be32_to_cpup(p);
	dprintk("%s stripe count  %d\n", __func__, cnt);
	if (cnt > NFS4_PNFS_MAX_STRIPE_CNT) {
		printk(KERN_WARNING "NFS: %s: stripe count %d greater than "
		       "supported maximum %d\n", __func__,
			cnt, NFS4_PNFS_MAX_STRIPE_CNT);
		goto out_err_free_scratch;
	}

	/* read stripe indices */
	stripe_indices = kcalloc(cnt, sizeof(u8), gfp_flags);
	if (!stripe_indices)
		goto out_err_free_scratch;

	p = xdr_inline_decode(&stream, cnt << 2);
	if (unlikely(!p))
		goto out_err_free_stripe_indices;

	indexp = &stripe_indices[0];
	max_stripe_index = 0;
	for (i = 0; i < cnt; i++) {
		*indexp = be32_to_cpup(p++);
		max_stripe_index = max(max_stripe_index, *indexp);
		indexp++;
	}

	/* Check the multipath list count */
	p = xdr_inline_decode(&stream, 4);
	if (unlikely(!p))
		goto out_err_free_stripe_indices;

	num = be32_to_cpup(p);
	dprintk("%s ds_num %u\n", __func__, num);
	if (num > NFS4_PNFS_MAX_MULTI_CNT) {
		printk(KERN_WARNING "NFS: %s: multipath count %d greater than "
			"supported maximum %d\n", __func__,
			num, NFS4_PNFS_MAX_MULTI_CNT);
		goto out_err_free_stripe_indices;
	}

	/* validate stripe indices are all < num */
	if (max_stripe_index >= num) {
		printk(KERN_WARNING "NFS: %s: stripe index %u >= num ds %u\n",
			__func__, max_stripe_index, num);
		goto out_err_free_stripe_indices;
	}

	dsaddr = kzalloc(sizeof(*dsaddr) +
			(sizeof(struct nfs4_pnfs_ds *) * (num - 1)),
			gfp_flags);
	if (!dsaddr)
		goto out_err_free_stripe_indices;

	dsaddr->stripe_count = cnt;
	dsaddr->stripe_indices = stripe_indices;
	stripe_indices = NULL;
	dsaddr->ds_num = num;
	nfs4_init_deviceid_node(&dsaddr->id_node, server, &pdev->dev_id);

	INIT_LIST_HEAD(&dsaddrs);

	for (i = 0; i < dsaddr->ds_num; i++) {
		int j;
		u32 mp_count;

		p = xdr_inline_decode(&stream, 4);
		if (unlikely(!p))
			goto out_err_free_deviceid;

		mp_count = be32_to_cpup(p); /* multipath count */
		for (j = 0; j < mp_count; j++) {
			da = nfs4_decode_mp_ds_addr(server->nfs_client->cl_net,
						    &stream, gfp_flags);
			if (da)
				list_add_tail(&da->da_node, &dsaddrs);
		}
		if (list_empty(&dsaddrs)) {
			dprintk("%s: no suitable DS addresses found\n",
				__func__);
			goto out_err_free_deviceid;
		}

		dsaddr->ds_list[i] = nfs4_pnfs_ds_add(&dsaddrs, gfp_flags);
		if (!dsaddr->ds_list[i])
			goto out_err_drain_dsaddrs;

		/* If DS was already in cache, free ds addrs */
		while (!list_empty(&dsaddrs)) {
			da = list_first_entry(&dsaddrs,
					      struct nfs4_pnfs_ds_addr,
					      da_node);
			list_del_init(&da->da_node);
			kfree(da->da_remotestr);
			kfree(da);
		}
	}

	__free_page(scratch);
	return dsaddr;

out_err_drain_dsaddrs:
	while (!list_empty(&dsaddrs)) {
		da = list_first_entry(&dsaddrs, struct nfs4_pnfs_ds_addr,
				      da_node);
		list_del_init(&da->da_node);
		kfree(da->da_remotestr);
		kfree(da);
	}
out_err_free_deviceid:
	nfs4_fl_free_deviceid(dsaddr);
	/* stripe_indicies was part of dsaddr */
	goto out_err_free_scratch;
out_err_free_stripe_indices:
	kfree(stripe_indices);
out_err_free_scratch:
	__free_page(scratch);
out_err:
	dprintk("%s ERROR: returning NULL\n", __func__);
	return NULL;
}

void
nfs4_fl_put_deviceid(struct nfs4_file_layout_dsaddr *dsaddr)
{
	nfs4_put_deviceid_node(&dsaddr->id_node);
}

/*
 * Want res = (offset - layout->pattern_offset)/ layout->stripe_unit
 * Then: ((res + fsi) % dsaddr->stripe_count)
 */
u32
nfs4_fl_calc_j_index(struct pnfs_layout_segment *lseg, loff_t offset)
{
	struct nfs4_filelayout_segment *flseg = FILELAYOUT_LSEG(lseg);
	u64 tmp;

	tmp = offset - flseg->pattern_offset;
	do_div(tmp, flseg->stripe_unit);
	tmp += flseg->first_stripe_index;
	return do_div(tmp, flseg->dsaddr->stripe_count);
}

u32
nfs4_fl_calc_ds_index(struct pnfs_layout_segment *lseg, u32 j)
{
	return FILELAYOUT_LSEG(lseg)->dsaddr->stripe_indices[j];
}

struct nfs_fh *
nfs4_fl_select_ds_fh(struct pnfs_layout_segment *lseg, u32 j)
{
	struct nfs4_filelayout_segment *flseg = FILELAYOUT_LSEG(lseg);
	u32 i;

	if (flseg->stripe_type == STRIPE_SPARSE) {
		if (flseg->num_fh == 1)
			i = 0;
		else if (flseg->num_fh == 0)
			/* Use the MDS OPEN fh set in nfs_read_rpcsetup */
			return NULL;
		else
			i = nfs4_fl_calc_ds_index(lseg, j);
	} else
		i = j;
	return flseg->fh_array[i];
}

/* Upon return, either ds is connected, or ds is NULL */
struct nfs4_pnfs_ds *
nfs4_fl_prepare_ds(struct pnfs_layout_segment *lseg, u32 ds_idx)
{
	struct nfs4_file_layout_dsaddr *dsaddr = FILELAYOUT_LSEG(lseg)->dsaddr;
	struct nfs4_pnfs_ds *ds = dsaddr->ds_list[ds_idx];
	struct nfs4_deviceid_node *devid = FILELAYOUT_DEVID_NODE(lseg);
	struct nfs4_pnfs_ds *ret = ds;
	struct nfs_server *s = NFS_SERVER(lseg->pls_layout->plh_inode);

	if (ds == NULL) {
		printk(KERN_ERR "NFS: %s: No data server for offset index %d\n",
			__func__, ds_idx);
		pnfs_generic_mark_devid_invalid(devid);
		goto out;
	}
	smp_rmb();
	if (ds->ds_clp)
		goto out_test_devid;

	nfs4_pnfs_ds_connect(s, ds, devid, dataserver_timeo,
			     dataserver_retrans, 4,
			     s->nfs_client->cl_minorversion,
			     s->nfs_client->cl_rpcclient->cl_auth->au_flavor);

out_test_devid:
	if (filelayout_test_devid_unavailable(devid))
		ret = NULL;
out:
	return ret;
}

module_param(dataserver_retrans, uint, 0644);
MODULE_PARM_DESC(dataserver_retrans, "The  number of times the NFSv4.1 client "
			"retries a request before it attempts further "
			" recovery  action.");
module_param(dataserver_timeo, uint, 0644);
MODULE_PARM_DESC(dataserver_timeo, "The time (in tenths of a second) the "
			"NFSv4.1  client  waits for a response from a "
			" data server before it retries an NFS request.");
