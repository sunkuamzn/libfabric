/* SPDX-License-Identifier: BSD-2-Clause OR GPL-2.0-only */
/* SPDX-FileCopyrightText: Copyright Amazon.com, Inc. or its affiliates. All rights reserved. */

#include "efa_unit_tests.h"
#include "efa_rdm_cq.h"
#include "efa_rdm_pke_req.h"
#include "efa_av.h"

/**
 * @brief Only works on nodes with EFA devices
 * This test calls fi_av_insert() twice with the same raw address,
 * and verifies that returned fi_addr is the same.
 * Since the addresses to be inserted have the same GID with the ep's self ah,
 * there should be only 1 ibv_create_ah call in the whole test.
 *
 * @param[in]	state		struct efa_resource that is managed by the framework
 */
void test_av_insert_duplicate_raw_addr(struct efa_resource **state)
{
	struct efa_resource *resource = *state;
	struct efa_ep_addr raw_addr = {0};
	size_t raw_addr_len = sizeof(struct efa_ep_addr);
	fi_addr_t addr1, addr2;
	int err, num_addr;

	g_efa_unit_test_mocks.ibv_create_ah = &efa_mock_ibv_create_ah_check_mock;
	/* the following will_return ensures ibv_create_ah is called exactly once */
	will_return(efa_mock_ibv_create_ah_check_mock, 0);

	efa_unit_test_resource_construct(resource, FI_EP_RDM, EFA_FABRIC_NAME);

	err = fi_getname(&resource->ep->fid, &raw_addr, &raw_addr_len);
	assert_int_equal(err, 0);
	raw_addr.qpn = 1;
	raw_addr.qkey = 0x1234;

	num_addr = fi_av_insert(resource->av, &raw_addr, 1, &addr1, 0 /* flags */, NULL /* context */);
	assert_int_equal(num_addr, 1);

	num_addr = fi_av_insert(resource->av, &raw_addr, 1, &addr2, 0 /* flags */, NULL /* context */);
	assert_int_equal(num_addr, 1);
	assert_int_equal(addr1, addr2);
}

/**
 * @brief Only works on nodes with EFA devices
 * This test calls fi_av_insert() twice with two difference raw address with same GID,
 * and verifies that returned fi_addr is different.
 * Since the addresses to be inserted have the same GID with the ep's self ah,
 * there should be only 1 ibv_create_ah call in the whole test.
 *
 * @param[in]	state		struct efa_resource that is managed by the framework
 */
void test_av_insert_duplicate_gid(struct efa_resource **state)
{
	struct efa_resource *resource = *state;
	struct efa_ep_addr raw_addr = {0};
	size_t raw_addr_len = sizeof(struct efa_ep_addr);
	fi_addr_t addr1, addr2;
	int err, num_addr;

	g_efa_unit_test_mocks.ibv_create_ah = &efa_mock_ibv_create_ah_check_mock;
	/* the following will_return ensures ibv_create_ah is called exactly once */
	will_return(efa_mock_ibv_create_ah_check_mock, 0);

	efa_unit_test_resource_construct(resource, FI_EP_RDM, EFA_FABRIC_NAME);

	err = fi_getname(&resource->ep->fid, &raw_addr, &raw_addr_len);
	assert_int_equal(err, 0);
	raw_addr.qpn = 1;
	raw_addr.qkey = 0x1234;

	num_addr = fi_av_insert(resource->av, &raw_addr, 1, &addr1, 0 /* flags */, NULL /* context */);
	assert_int_equal(num_addr, 1);

	raw_addr.qpn = 2;
	raw_addr.qkey = 0x5678;
	num_addr = fi_av_insert(resource->av, &raw_addr, 1, &addr2, 0 /* flags */, NULL /* context */);
	assert_int_equal(num_addr, 1);
	assert_int_not_equal(addr1, addr2);
}

void test_efa_ah_cnt_one_av(struct efa_resource **state)
{
	struct efa_resource *resource = *state;
	struct efa_ep_addr raw_addr = {0};
	size_t raw_addr_len = sizeof(struct efa_ep_addr);
	fi_addr_t addr1, addr2;
	int err, num_addr;
	struct efa_domain *efa_domain;
	struct efa_ah *efa_ah = NULL;

	efa_unit_test_resource_construct(resource, FI_EP_RDM, EFA_FABRIC_NAME);

	efa_domain = container_of(resource->domain, struct efa_domain, util_domain.domain_fid);

	err = fi_getname(&resource->ep->fid, &raw_addr, &raw_addr_len);
	assert_int_equal(err, 0);

	/* So far we should only have 1 ah from ep self ah, and its refcnt is 1 */
	assert_int_equal(HASH_CNT(hh, efa_domain->ah_map), 1);
	HASH_FIND(hh, efa_domain->ah_map, raw_addr.raw, EFA_GID_LEN, efa_ah);
	assert_non_null(efa_ah);
	assert_int_equal(efa_ah->refcnt, 1);

	raw_addr.qpn = 1;
	raw_addr.qkey = 0x1234;

	num_addr = fi_av_insert(resource->av, &raw_addr, 1, &addr1, 0 /* flags */, NULL /* context */);
	assert_int_equal(num_addr, 1);

	raw_addr.qpn = 2;
	raw_addr.qkey = 0x5678;
	num_addr = fi_av_insert(resource->av, &raw_addr, 1, &addr2, 0 /* flags */, NULL /* context */);
	assert_int_equal(num_addr, 1);
	assert_int_not_equal(addr1, addr2);

	/* So far we should still have 1 ah, and its refcnt is 3 (plus the 2 av entries) */
	assert_int_equal(HASH_CNT(hh, efa_domain->ah_map), 1);
	assert_int_equal(efa_ah->refcnt, 3);

	/* ah refcnt should be decremented to 1 after av entry removals */
	assert_int_equal(fi_av_remove(resource->av, &addr1, 1, 0), 0);
	assert_int_equal(fi_av_remove(resource->av, &addr2, 1, 0), 0);

	assert_int_equal(HASH_CNT(hh, efa_domain->ah_map), 1);
	assert_int_equal(efa_ah->refcnt, 1);

	/* ah map should be empty now after closing ep which destroys the self ah */
	assert_int_equal(fi_close(&resource->ep->fid), 0);
	assert_int_equal(HASH_CNT(hh, efa_domain->ah_map), 0);
	/* Reset to NULL to avoid test reaper closing again */
	resource->ep = NULL;
}

void test_efa_ah_cnt_multi_av(struct efa_resource **state)
{
	struct efa_resource *resource = *state;
	struct efa_ep_addr raw_addr = {0};
	size_t raw_addr_len = sizeof(struct efa_ep_addr);
	fi_addr_t addr1, addr2;
	int err, num_addr;
	struct efa_domain *efa_domain;
	struct efa_ah *efa_ah = NULL;
	struct fi_av_attr av_attr = {0};
	struct fid_av *av1, *av2;
	struct fid_ep *ep1, *ep2;

	efa_unit_test_resource_construct(resource, FI_EP_RDM, EFA_DIRECT_FABRIC_NAME);

	efa_domain = container_of(resource->domain, struct efa_domain, util_domain.domain_fid);

	err = fi_getname(&resource->ep->fid, &raw_addr, &raw_addr_len);
	assert_int_equal(err, 0);

	/* So far we should only have 1 ah from ep self ah, and its refcnt is 1 */
	assert_int_equal(HASH_CNT(hh, efa_domain->ah_map), 1);
	HASH_FIND(hh, efa_domain->ah_map, raw_addr.raw, EFA_GID_LEN, efa_ah);
	assert_non_null(efa_ah);
	assert_int_equal(efa_ah->refcnt, 1);


	/* We open 2 avs with the same domain (PD) so they should share same AH given the same GID */
	assert_int_equal(fi_av_open(resource->domain, &av_attr, &av1, NULL), 0);
	assert_int_equal(fi_av_open(resource->domain, &av_attr, &av2, NULL), 0);

	/* Due to the current restriction in efa provider, we have to bind av to ep before inserting av entry */
	/* These eps will not create self ah as they are not enabled */
	assert_int_equal(fi_endpoint(resource->domain, resource->info, &ep1, NULL), 0);
	assert_int_equal(fi_endpoint(resource->domain, resource->info, &ep2, NULL), 0);

	assert_int_equal(fi_ep_bind(ep1, &av1->fid, 0), 0);
	assert_int_equal(fi_ep_bind(ep2, &av2->fid, 0), 0);

	raw_addr.qpn = 1;
	raw_addr.qkey = 0x1234;

	num_addr = fi_av_insert(av1, &raw_addr, 1, &addr1, 0 /* flags */, NULL /* context */);
	assert_int_equal(num_addr, 1);

	raw_addr.qpn = 2;
	raw_addr.qkey = 0x5678;
	num_addr = fi_av_insert(av2, &raw_addr, 1, &addr2, 0 /* flags */, NULL /* context */);
	assert_int_equal(num_addr, 1);
	/* They should be as equal as 0 they are in different avs */
	assert_int_equal(addr1, addr2);

	/* So far we should still have 1 ah, and its refcnt is 3 (plus the 2 av entries) */
	assert_int_equal(HASH_CNT(hh, efa_domain->ah_map), 1);
	assert_int_equal(efa_ah->refcnt, 3);

	/* ah refcnt should be decremented to 1 after av close */
	assert_int_equal(fi_close(&ep1->fid), 0);
	assert_int_equal(fi_close(&ep2->fid), 0);
	assert_int_equal(fi_close(&av1->fid), 0);
	assert_int_equal(fi_close(&av2->fid), 0);

	assert_int_equal(HASH_CNT(hh, efa_domain->ah_map), 1);
	assert_int_equal(efa_ah->refcnt, 1);

	/* ah map should be empty now after closing ep which destroys the self ah */
	assert_int_equal(fi_close(&resource->ep->fid), 0);
	assert_int_equal(HASH_CNT(hh, efa_domain->ah_map), 0);
	/* Reset to NULL to avoid test reaper closing again */
	resource->ep = NULL;
}

/**
 * @brief This test verifies that multiple endpoints can bind to the same AV
 *
 * @param[in]	state		struct efa_resource that is managed by the framework
 */
void test_av_multiple_ep_impl(struct efa_resource **state, char *fabric_name)
{
	struct efa_resource *resource = *state;
	struct fid_ep *ep1, *ep2;
	int ret;

	/* Resource construct function creates and binds 1 EP to the AV */
	efa_unit_test_resource_construct(resource, FI_EP_RDM, fabric_name);

	/* Create and bind two new endpoints to the same AV */
	fi_endpoint(resource->domain, resource->info, &ep1, NULL);
	ret = fi_ep_bind(ep1, &resource->av->fid, 0);
	assert_int_equal(ret, 0);

	fi_endpoint(resource->domain, resource->info, &ep2, NULL);
	ret = fi_ep_bind(ep2, &resource->av->fid, 0);
	assert_int_equal(ret, 0);

	/* Bind the two new endpoints to the same CQ and enable them */
	fi_ep_bind(ep1, &resource->cq->fid, FI_SEND | FI_RECV);
	ret = fi_enable(ep1);
	assert_int_equal(ret, 0);

	fi_ep_bind(ep2, &resource->cq->fid, FI_SEND | FI_RECV);
	ret = fi_enable(ep2);
	assert_int_equal(ret, 0);

	fi_close(&ep1->fid);
	fi_close(&ep2->fid);
}


/**
 * @brief This test verifies that multiple endpoints can bind to the same AV
 * for the efa fabric
 *
 * @param[in]	state		struct efa_resource that is managed by the framework
 */
void test_av_multiple_ep_efa(struct efa_resource **state)
{
	return test_av_multiple_ep_impl(state, EFA_FABRIC_NAME);
}

/**
 * @brief This test verifies that multiple endpoints can bind to the same AV
 * for the efa-direct fabric
 *
 * @param[in]	state		struct efa_resource that is managed by the framework
 */
void test_av_multiple_ep_efa_direct(struct efa_resource **state)
{
	return test_av_multiple_ep_impl(state, EFA_DIRECT_FABRIC_NAME);
}

static void test_av_verify_av_hash_cnt(struct efa_av *av, int explicit_av_count, int implicit_av_count) {
	assert_int_equal(HASH_CNT(hh, av->util_av.hash), explicit_av_count);
	assert_int_equal(HASH_CNT(hh, av->cur_reverse_av), explicit_av_count);
	assert_int_equal(HASH_CNT(hh, av->prv_reverse_av), 0);

	assert_int_equal(HASH_CNT(hh, av->util_av_implicit.hash), implicit_av_count);
	assert_int_equal(HASH_CNT(hh, av->cur_reverse_av_implicit), implicit_av_count);
	assert_int_equal(HASH_CNT(hh, av->prv_reverse_av_implicit), 0);
}

/**
 * @brief This test removes a peer and inserts it again
 *
 * @param[in]	state	struct efa_resource that is managed by the framework
 */
void test_av_reinsertion(struct efa_resource **state)
{
	struct efa_resource *resource = *state;
	struct efa_rdm_peer *peer;
	struct efa_ep_addr raw_addr, raw_addr_2;
	size_t raw_addr_len = sizeof(struct efa_ep_addr);
	fi_addr_t fi_addr;
	struct efa_av *av;
	struct efa_rdm_ep *efa_rdm_ep;
	int err;

	efa_unit_test_resource_construct(resource, FI_EP_RDM, EFA_FABRIC_NAME);

	err = fi_getname(&resource->ep->fid, &raw_addr, &raw_addr_len);
	assert_int_equal(err, 0);
	raw_addr.qpn = 174;
	raw_addr.qkey = 0x1234;

	av = container_of(resource->av, struct efa_av, util_av.av_fid);
	efa_rdm_ep = container_of(resource->ep, struct efa_rdm_ep, base_ep.util_ep.ep_fid);

	err = fi_av_insert(resource->av, &raw_addr, 1, &fi_addr, 0, NULL);
	assert_int_equal(err, 1);
	assert_int_equal(fi_addr, 0);
	test_av_verify_av_hash_cnt(av, 1, 0);

	err = fi_av_lookup(resource->av, fi_addr, &raw_addr_2, &raw_addr_len);
	assert_int_equal(err, 0);
	assert_int_equal(efa_is_same_addr(&raw_addr, &raw_addr_2), 1);

	peer = efa_rdm_ep_get_peer(efa_rdm_ep, fi_addr);
	assert_int_equal(peer->conn->fi_addr, fi_addr);
	assert_int_equal(efa_is_same_addr(&raw_addr, peer->conn->ep_addr), 1);

	err = fi_av_remove(resource->av, &fi_addr, 1, 0);
	assert_int_equal(err, 0);
	test_av_verify_av_hash_cnt(av, 0, 0);

	err = fi_av_insert(resource->av, &raw_addr, 1, &fi_addr, 0, NULL);
	assert_int_equal(err, 1);
	assert_int_equal(fi_addr, 0);
	test_av_verify_av_hash_cnt(av, 1, 0);

	err = fi_av_lookup(resource->av, fi_addr, &raw_addr_2, &raw_addr_len);
	assert_int_equal(err, 0);
	assert_int_equal(efa_is_same_addr(&raw_addr, &raw_addr_2), 1);

	peer = efa_rdm_ep_get_peer(efa_rdm_ep, fi_addr);
	assert_int_equal(peer->conn->fi_addr, fi_addr);
	assert_int_equal(efa_is_same_addr(&raw_addr, peer->conn->ep_addr), 1);

	err = fi_av_remove(resource->av, &fi_addr, 1, 0);
	assert_int_equal(err, 0);
	test_av_verify_av_hash_cnt(av, 0, 0);
}

/**
 * @brief This test simulates a packet received before the application calls
 * fi_av_insert. It verifies that the peer is in the implicit AV after the
 * packet is received and that the peer is moved to the xplicit AV after
 * fi_av_insert is called. It verifies that the peer information including
 * the next expected message id etc are also moved.
 *
 * @param[in]	state	struct efa_resource that is managed by the framework
 */
void test_av_recv_msg_before_fi_av_insert(struct efa_resource **state)
{
	struct efa_resource *resource = *state;
	struct efa_rdm_pke *pkt_entry;
	struct efa_rdm_base_hdr *base_hdr;
	char *opt_hdr;
	struct efa_rdm_req_opt_raw_addr_hdr *raw_addr_hdr;
	struct efa_rdm_ep *efa_rdm_ep;
	struct efa_rdm_cq *efa_rdm_cq;
	struct ibv_cq_ex *ibv_cqx;
	struct efa_ep_addr raw_addr, raw_addr_2;
	size_t raw_addr_len = sizeof(struct efa_ep_addr);
	fi_addr_t explicit_fi_addr, implicit_fi_addr = 0;
	int err, numaddr;
	struct efa_av *av;
	struct fi_cq_data_entry cq_entry;
	struct efa_rdm_peer *peer;
	struct efa_rdm_rtm_base_hdr *rtm_hdr;
	struct efa_rdm_ope *ope;
	struct dlist_entry *entry, *tmp;

	/* Disable handshake packet send */
	g_efa_unit_test_mocks.efa_rdm_ep_has_unfinished_send = &efa_mock_efa_rdm_ep_has_unfinished_send_return_mock;
	will_return_always(efa_mock_efa_rdm_ep_has_unfinished_send_return_mock, false);

	efa_unit_test_resource_construct(resource, FI_EP_RDM, EFA_FABRIC_NAME);

	av = container_of(resource->av, struct efa_av, util_av.av_fid);
	efa_rdm_ep = container_of(resource->ep, struct efa_rdm_ep, base_ep.util_ep.ep_fid);

	err = fi_getname(&resource->ep->fid, &raw_addr, &raw_addr_len);
	assert_int_equal(err, 0);
	raw_addr.qpn = 156;
	raw_addr.qkey = 0x1234;

	/* Simulate fi_cq_read receiving a packet from a peer not in explicit AV
	 * Logic copied from tests in efa_unit_test_cq.c */
	pkt_entry = efa_rdm_pke_alloc(efa_rdm_ep, efa_rdm_ep->efa_rx_pkt_pool, EFA_RDM_PKE_FROM_EFA_RX_POOL);
	assert_non_null(pkt_entry);

	/* initialize the pkt as an eager RTM packet */
	base_hdr = (struct efa_rdm_base_hdr *) pkt_entry->wiredata;
	base_hdr->type = EFA_RDM_EAGER_MSGRTM_PKT;
	base_hdr->version = EFA_RDM_PROTOCOL_VERSION;
	base_hdr->flags = 0;
	base_hdr->flags |= EFA_RDM_REQ_OPT_RAW_ADDR_HDR;
	opt_hdr = (char *)base_hdr + efa_rdm_pke_get_req_base_hdr_size(pkt_entry);
	raw_addr_hdr = (struct efa_rdm_req_opt_raw_addr_hdr *)opt_hdr;
	raw_addr_hdr->addr_len = EFA_RDM_REQ_OPT_RAW_ADDR_HDR_SIZE - sizeof(struct efa_rdm_req_opt_raw_addr_hdr);
	memcpy(raw_addr_hdr->raw_addr, &raw_addr, sizeof(raw_addr));

	rtm_hdr = (struct efa_rdm_rtm_base_hdr *)pkt_entry->wiredata;
	rtm_hdr->flags |= EFA_RDM_REQ_MSG;
	rtm_hdr->msg_id = 0;

	efa_rdm_ep->efa_rx_pkts_posted = efa_rdm_ep_get_rx_pool_size(efa_rdm_ep);

	efa_rdm_cq = container_of(resource->cq, struct efa_rdm_cq, efa_cq.util_cq.cq_fid.fid);
	ibv_cqx = efa_rdm_cq->efa_cq.ibv_cq.ibv_cq_ex;

	ibv_cqx->start_poll = &efa_mock_ibv_start_poll_return_mock;
	ibv_cqx->end_poll = &efa_mock_ibv_end_poll_check_mock;
	ibv_cqx->read_opcode = &efa_mock_ibv_read_opcode_return_mock;
	ibv_cqx->read_vendor_err = &efa_mock_ibv_read_vendor_err_return_mock;
	ibv_cqx->read_qp_num = &efa_mock_ibv_read_qp_num_return_mock;
	ibv_cqx->read_byte_len = &efa_mock_ibv_read_byte_len_return_mock;
	ibv_cqx->read_wc_flags = &efa_mock_ibv_read_wc_flags_return_mock;
	ibv_cqx->read_slid = &efa_mock_ibv_read_slid_return_mock;
	ibv_cqx->read_src_qp = &efa_mock_ibv_read_src_qp_return_mock;
	ibv_cqx->next_poll = &efa_mock_ibv_next_poll_return_mock;

	will_return(efa_mock_ibv_start_poll_return_mock, 0);
	/* efa_base_ep_flush_cq calls ibv_start_poll twice more */
	will_return(efa_mock_ibv_start_poll_return_mock, ENOENT);
	will_return(efa_mock_ibv_start_poll_return_mock, ENOENT);

	will_return(efa_mock_ibv_end_poll_check_mock, NULL);
	will_return(efa_mock_ibv_read_byte_len_return_mock, pkt_entry->pkt_size);
	will_return(efa_mock_ibv_read_wc_flags_return_mock, 0);
	will_return(efa_mock_ibv_read_slid_return_mock, efa_rdm_ep->base_ep.self_ah->ahn);
	will_return(efa_mock_ibv_read_src_qp_return_mock, raw_addr.qpn);

	will_return_always(efa_mock_ibv_next_poll_return_mock, ENOENT);

	will_return_always(efa_mock_ibv_read_opcode_return_mock, IBV_WC_RECV);
	will_return_always(efa_mock_ibv_read_qp_num_return_mock, efa_rdm_ep->base_ep.qp->qp_num);
	ibv_cqx->wr_id = (uintptr_t)pkt_entry;
	ibv_cqx->status = IBV_WC_SUCCESS;

	err = fi_cq_read(resource->cq, &cq_entry, 1);

	/* No fi_recv posted, so we expect to get -FI_EAGAIN */
	assert_int_equal(err, -FI_EAGAIN);

	test_av_verify_av_hash_cnt(av, 0, 1);
	peer = efa_rdm_ep_get_peer_implicit(efa_rdm_ep, implicit_fi_addr);
	assert_int_equal(peer->conn->implicit_fi_addr, implicit_fi_addr);
	assert_int_equal(efa_is_same_addr(&raw_addr, peer->conn->ep_addr), 1);
	assert_int_equal(ofi_recvwin_next_exp_id((&peer->robuf)), 1);

	/* Now insert the address into the AV */
	numaddr = fi_av_insert(resource->av, &raw_addr, 1, &explicit_fi_addr, 0, NULL);
	assert_int_equal(numaddr, 1);

	err = fi_av_lookup(resource->av, explicit_fi_addr, &raw_addr_2, &raw_addr_len);
	assert_int_equal(err, 0);
	assert_int_equal(efa_is_same_addr(&raw_addr, &raw_addr_2), 1);

	peer = efa_rdm_ep_get_peer(efa_rdm_ep, 0);
	assert_int_equal(peer->conn->fi_addr, explicit_fi_addr);
	assert_int_equal(efa_is_same_addr(&raw_addr, peer->conn->ep_addr), 1);
	assert_int_equal(ofi_recvwin_next_exp_id((&peer->robuf)), 1);

	test_av_verify_av_hash_cnt(av, 1, 0);

	/* TODO: Remove this hack after either (1) fixing the mocking of
	 * efa_rdm_ep_has_unfinished_send or (2) The change to not re-post
	 * handshake packets when closing the endpoint */
	efa_rdm_ep->efa_outstanding_tx_ops = 0;
	uint64_t queued_ope_flags = EFA_RDM_OPE_QUEUED_CTRL | EFA_RDM_OPE_QUEUED_RNR;
	dlist_foreach_safe(&efa_rdm_ep_domain(efa_rdm_ep)->ope_queued_list, entry, tmp) {
		ope = container_of(entry, struct efa_rdm_ope,
					queued_entry);
		ope->internal_flags &= ~queued_ope_flags;
	}
}
