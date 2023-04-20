/*
 * Copyright (c) Intel Corporation, Inc.  All rights reserved.
 * Copyright (c) Amazon.com, Inc. or its affiliates. All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _SM2_H_
#define _SM2_H_

#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/statvfs.h>
#include <sys/types.h>

#include <rdma/fabric.h>
#include <rdma/fi_atomic.h>
#include <rdma/fi_cm.h>
#include <rdma/fi_domain.h>
#include <rdma/fi_endpoint.h>
#include <rdma/fi_eq.h>
#include <rdma/fi_errno.h>
#include <rdma/fi_tagged.h>
#include <rdma/fi_trigger.h>
#include <rdma/providers/fi_peer.h>
#include <rdma/providers/fi_prov.h>

#include <ofi.h>
#include <ofi_atomic.h>
#include <ofi_enosys.h>
#include <ofi_epoll.h>
#include <ofi_iov.h>
#include <ofi_list.h>
#include <ofi_lock.h>
#include <ofi_mr.h>
#include <ofi_rbuf.h>
#include <ofi_signal.h>
#include <ofi_util.h>

#include "sm2_coordination.h"

#define MAX_SM2_MSGS_PROGRESSED 32
#define SM2_IOV_LIMIT		4
#define SM2_PREFIX		"fi_sm2://"
#define SM2_PREFIX_NS		"fi_ns://"
#define SM2_VERSION		1
#define SM2_IOV_LIMIT		4
#define SM2_REMOTE_CQ_DATA	(1 << 0)
#define SM2_TX_COMPLETION	(1 << 1)
#define SM2_RX_COMPLETION	(1 << 2)

extern struct fi_provider sm2_prov;
extern struct fi_info sm2_info;
extern struct util_prov sm2_util_prov;
extern int sm2_global_ep_idx; // protected by the ep_list_lock

extern struct dlist_entry sm2_ep_name_list;
extern pthread_mutex_t sm2_ep_list_lock;

enum {
	sm2_proto_inject,
	sm2_proto_return,
	sm2_proto_max,
};

/*
 * sm2 Generic Message Header:
 * 	next - fifo linked list next ptr
 * 		This is volatile for a reason, many things touch this
 * 		and we do not want compiler optimization here
 * 	size - Holds total size of message
 * 	data - remote CQ data
 * 	tag - used for tagged messages
 * 	op - fi operation
 * 	op_flags - flags associated with op (ex. SM2_REMOTE_CQ_DATA, defined
 * above) sender_gid - id of msg sender proto - sm2 operation
 */
struct sm2_msg_hdr {
	volatile long int next;
	uint64_t size;
	uint64_t data;
	uint64_t tag;
	uint32_t op;
	uint32_t op_flags;
	int sender_gid;
	uint16_t proto;
} __attribute__((aligned(16)));

struct sm2_fqe {
	struct sm2_msg_hdr msg_hdr;
	uint8_t data[SM2_INJECT_SIZE];
};

struct sm2_ep_name {
	char name[SM2_NAME_MAX];
	struct sm2_region *region;
	struct dlist_entry entry;
};

static inline struct sm2_fifo *
sm2_recv_queue(struct sm2_region *smr)
{
	return (struct sm2_fifo *) ((char *) smr + smr->recv_queue_offset);
}

static inline struct smr_freestack *
sm2_free_stack(struct sm2_region *smr)
{
	return (struct smr_freestack *) ((char *) smr + smr->free_stack_offset);
}

int sm2_fabric(struct fi_fabric_attr *attr, struct fid_fabric **fabric,
	       void *context);

struct sm2_av {
	struct util_av util_av; /* FI_ADDR -> GID */
	fi_addr_t *fi_addr; /* GID -> FI_ADDR */
	struct sm2_mmap sm2_mmap;
};

int sm2_domain_open(struct fid_fabric *fabric, struct fi_info *info,
		    struct fid_domain **dom, void *context);

int sm2_eq_open(struct fid_fabric *fabric, struct fi_eq_attr *attr,
		struct fid_eq **eq, void *context);

int sm2_av_open(struct fid_domain *domain, struct fi_av_attr *attr,
		struct fid_av **av, void *context);

struct sm2_cq {
	struct util_cq util_cq;
	struct fid_peer_cq *peer_cq;
};

static inline int
sm2_match_id(fi_addr_t addr, fi_addr_t match_addr)
{
	return (addr == FI_ADDR_UNSPEC) || (match_addr == FI_ADDR_UNSPEC) ||
	       (addr == match_addr);
}

static inline int
sm2_match_tag(uint64_t tag, uint64_t ignore, uint64_t match_tag)
{
	return ((tag | ignore) == (match_tag | ignore));
}

struct sm2_fqe_ctx {
	struct dlist_entry entry;
	struct sm2_ep *ep;
	struct sm2_fqe fqe;
};

struct sm2_domain {
	struct util_domain util_domain;
	struct fid_peer_srx *srx;
};

struct sm2_rx_entry {
	struct fi_peer_rx_entry peer_entry;
	struct iovec iov[SM2_IOV_LIMIT];
	void *desc[SM2_IOV_LIMIT];
	int64_t peer_id;
	uint64_t ignore;
	int multi_recv_ref;
	uint64_t err;
};

struct sm2_queue {
	struct dlist_entry list;
	dlist_func_t *match_func;
};

OFI_DECLARE_FREESTACK(struct sm2_rx_entry, sm2_recv_fs);

struct sm2_match_attr {
	fi_addr_t id;
	uint64_t tag;
	uint64_t ignore;
};

struct sm2_srx_ctx {
	struct fid_peer_srx peer_srx;
	struct sm2_queue recv_queue;
	struct sm2_queue trecv_queue;
	bool dir_recv;
	size_t min_multi_recv_size;
	uint64_t rx_op_flags;
	uint64_t rx_msg_flags;

	struct util_cq *cq;
	struct sm2_queue unexp_msg_queue;
	struct sm2_queue unexp_tagged_queue;
	struct sm2_recv_fs *recv_fs;

	// TODO Determine if this spin lock is needed.
	ofi_spin_t lock;
};

struct sm2_rx_entry *sm2_alloc_rx_entry(struct sm2_srx_ctx *srx);

struct sm2_ep {
	struct util_ep util_ep;
	size_t tx_size;
	const char *name;
	int gid;
	struct sm2_mmap *mmap_regions;
	ofi_spin_t tx_lock;
	struct fid_ep *srx;
	struct ofi_bufpool *fqe_ctx_pool;
	int ep_idx;
};

static inline struct sm2_srx_ctx *
sm2_get_sm2_srx(struct sm2_ep *ep)
{
	return (struct sm2_srx_ctx *) ep->srx->fid.context;
}

static inline struct fid_peer_srx *
sm2_get_peer_srx(struct sm2_ep *ep)
{
	return container_of(ep->srx, struct fid_peer_srx, ep_fid);
}

#define sm2_ep_rx_flags(sm2_ep) ((sm2_ep)->util_ep.rx_op_flags)
#define sm2_ep_tx_flags(sm2_ep) ((sm2_ep)->util_ep.tx_op_flags)

int sm2_srx_context(struct fid_domain *domain, struct fi_rx_attr *attr,
		    struct fid_ep **rx_ep, void *context);

int sm2_endpoint(struct fid_domain *domain, struct fi_info *info,
		 struct fid_ep **ep, void *context);
int sm2_cq_open(struct fid_domain *domain, struct fi_cq_attr *attr,
		struct fid_cq **cq_fid, void *context);
int sm2_cntr_open(struct fid_domain *domain, struct fi_cntr_attr *attr,
		  struct fid_cntr **cntr_fid, void *context);

int sm2_verify_peer(struct sm2_ep *ep, fi_addr_t fi_addr);

void sm2_generic_format(struct sm2_fqe *cmd, int peer_gid, uint32_t op,
			uint64_t tag, uint64_t data, uint64_t op_flags);

typedef ssize_t (*sm2_proto_func)(struct sm2_ep *ep,
				  struct sm2_region *peer_smr, int peer_gid,
				  uint32_t op, uint64_t tag, uint64_t data,
				  uint64_t op_flags, enum fi_hmem_iface iface,
				  uint64_t device, const struct iovec *iov,
				  size_t iov_count, size_t total_len);
extern sm2_proto_func sm2_proto_ops[sm2_proto_max];

int sm2_write_err_comp(struct util_cq *cq, void *context, uint64_t flags,
		       uint64_t tag, uint64_t err);
int sm2_complete_tx(struct sm2_ep *ep, void *context, uint32_t op,
		    uint64_t flags);
int sm2_complete_rx(struct sm2_ep *ep, void *context, uint32_t op,
		    uint64_t flags, size_t len, void *buf, int gid,
		    uint64_t tag, uint64_t data);

static inline uint64_t
sm2_rx_cq_flags(uint32_t op, uint64_t rx_flags, uint16_t op_flags)
{
	rx_flags |= ofi_rx_cq_flags(op);
	if (op_flags & SM2_REMOTE_CQ_DATA)
		rx_flags |= FI_REMOTE_CQ_DATA;
	return rx_flags;
}

void sm2_ep_progress(struct util_ep *util_ep);

void sm2_progress_recv(struct sm2_ep *ep);

int sm2_unexp_start(struct fi_peer_rx_entry *rx_entry);

static inline struct sm2_region *
sm2_peer_region(struct sm2_ep *ep, int id)
{
	struct sm2_av *sm2_av;

	assert(id < SM2_MAX_PEERS);
	sm2_av = container_of(ep->util_ep.av, struct sm2_av, util_av);

	return (struct sm2_region *) sm2_mmap_ep_region(&sm2_av->sm2_mmap, id);
}

bool sm2_adjust_multi_recv(struct sm2_srx_ctx *srx,
			   struct fi_peer_rx_entry *rx_entry, size_t len);
void sm2_init_rx_entry(struct sm2_rx_entry *entry, const struct iovec *iov,
		       void **desc, size_t count, fi_addr_t addr, void *context,
		       uint64_t tag, uint64_t flags);
struct sm2_rx_entry *sm2_get_recv_entry(struct sm2_srx_ctx *srx,
					const struct iovec *iov, void **desc,
					size_t count, fi_addr_t addr,
					void *context, uint64_t tag,
					uint64_t ignore, uint64_t flags);

#endif /* _SM2_H_ */
