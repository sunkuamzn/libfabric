/*
 * Copyright (c) 2021 Amazon.com, Inc. or its affiliates.
 * All rights reserved.
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

#include "efa.h"
#include "rxr_pkt_cmd.h"

/**
 * @brief return the optional connid header pointer in a packet
 *
 * @param[in]	pkt_entry	an packet entry
 * @return	If the input has the optional connid header, return the pointer to connid header
 * 		Otherwise, return NULL
 */
uint32_t *rxr_pkt_connid_ptr(struct efa_rdm_pke *pkt_entry)
{
	struct rxr_base_hdr *base_hdr;

	base_hdr = rxr_get_base_hdr(pkt_entry->wiredata);

	if (base_hdr->type >= RXR_REQ_PKT_BEGIN)
		return rxr_pkt_req_connid_ptr(pkt_entry);

	if (!(base_hdr->flags & RXR_PKT_CONNID_HDR))
		return NULL;

	switch (base_hdr->type) {
	case RXR_CTS_PKT:
		return &(rxr_get_cts_hdr(pkt_entry->wiredata)->connid);

	case RXR_RECEIPT_PKT:
		return &(rxr_get_receipt_hdr(pkt_entry->wiredata)->connid);

	case RXR_DATA_PKT:
		return &(rxr_get_data_hdr(pkt_entry->wiredata)->connid_hdr->connid);

	case RXR_READRSP_PKT:
		return &(rxr_get_readrsp_hdr(pkt_entry->wiredata)->connid);

	case RXR_ATOMRSP_PKT:
		return &(rxr_get_atomrsp_hdr(pkt_entry->wiredata)->connid);

	case RXR_EOR_PKT:
		return &rxr_get_eor_hdr(pkt_entry->wiredata)->connid;

	case RXR_HANDSHAKE_PKT:
		return &(rxr_get_handshake_opt_connid_hdr(pkt_entry->wiredata)->connid);

	default:
		EFA_WARN(FI_LOG_CQ, "unknown packet type: %d\n", base_hdr->type);
		assert(0 && "Unknown packet type");
	}

	return NULL;
}

/**
 * @brief set up data in a packet entry using txe/rxe information, such that the packet is ready to be sent.
 *        Depending on the ope, this function can either copy data to packet entry, or point
 *        pkt_entry->iov to ope->iov.
 *        It requires the packet header to be set.
 *
 * @param[in]		ep				end point.
 * @param[in,out]	pkt_entry		packet entry. Header must have been set when the function is called
 * @param[in]		pkt_data_offset	the data offset in packet, (in reference to pkt_entry->wiredata).
 * @param[in]		ope		This function will use iov, iov_count and desc of ope
 * @param[in]		op_data_offset	source offset of the data (in reference to ope->iov)
 * @param[in]		data_size		length of the data to be set up.
 * @return		0 on success, negative FI code on error
 */
int rxr_pkt_init_data_from_ope(struct efa_rdm_ep *ep,
				    struct efa_rdm_pke *pkt_entry,
				    size_t pkt_data_offset,
				    struct efa_rdm_ope *ope,
				    size_t tx_data_offset,
				    size_t data_size)
{
	int tx_iov_index;
	char *data;
	size_t tx_iov_offset, copied;
	bool expose_iov_to_device = false;
	struct efa_mr *iov_mr;
	int ret;

	assert(pkt_data_offset > 0);

	pkt_entry->ope = ope;
	if (data_size == 0) {
		pkt_entry->pkt_size = pkt_data_offset;
		return 0;
	}

	ret = ofi_iov_locate(ope->iov, ope->iov_count, tx_data_offset,
			     &tx_iov_index, &tx_iov_offset);
	if (OFI_UNLIKELY(ret)) {
		EFA_WARN(FI_LOG_CQ, "ofi_iov_locate failed! err: %d\n", ret);
		return ret;
	}

	assert(tx_iov_index < ope->iov_count);
	iov_mr = ope->desc[tx_iov_index];
	assert(tx_iov_index < ope->iov_count);
	assert(tx_iov_offset < ope->iov[tx_iov_index].iov_len);

	if (pkt_entry->mr) {
		/* When using EFA device, EFA device can access memory that
		 * that has been registered, and p2p is allowed to be used.
		 */
		if (iov_mr) {
			ret = efa_rdm_ep_use_p2p(ep, iov_mr);
			if (ret < 0)
				return ret;
			expose_iov_to_device = ret;
		} else {
			expose_iov_to_device = false;
		}
	} else {
		/**
		 * This branch is hit when EAGER RTM protocol is selected between EFA and SHM to
		 * transfer small messages. To achieve faster cuda memory copies, we do not expose
		 * the HMEM iov to SHM, i.e. device, because it only supports cudaMemcpy which incurs
		 * a high overhead.
		 *
		 * Instead, we copy the data to the bounce buffer with gdrcopy(if available) which is
		 * faster for small messages, before dispatching it to SHM.
		 */
		expose_iov_to_device = !iov_mr || iov_mr->peer.iface == FI_HMEM_SYSTEM;
	}

	/*
	 * Copy can be avoid if the following 2 conditions are true:
	 * 1. EFA/shm can directly access the memory
	 * 2. data to be send is in 1 iov, because device only support 2 iov, and we use
	 *    1st iov for header.
	 */
	if (expose_iov_to_device &&
	    (tx_iov_offset + data_size <= ope->iov[tx_iov_index].iov_len)) {
		pkt_entry->payload = (char *)ope->iov[tx_iov_index].iov_base + tx_iov_offset;
		pkt_entry->payload_size = data_size;
		pkt_entry->payload_mr = ope->desc[tx_iov_index];
		pkt_entry->pkt_size = pkt_data_offset + data_size;
		return 0;
	}

	data = pkt_entry->wiredata + pkt_data_offset;
	if (iov_mr && FI_HMEM_CUDA == iov_mr->peer.iface &&
	    (iov_mr->peer.flags & OFI_HMEM_DATA_GDRCOPY_HANDLE)) {
		assert(iov_mr->peer.hmem_data);
		copied = ofi_gdrcopy_from_cuda_iov((uint64_t)iov_mr->peer.hmem_data, data,
		                                   ope->iov, ope->iov_count,
		                                   tx_data_offset, data_size);
	} else {
		copied = ofi_copy_from_hmem_iov(data, data_size,
		                                iov_mr ? iov_mr->peer.iface : FI_HMEM_SYSTEM,
		                                iov_mr ? iov_mr->peer.device.reserved : 0,
		                                ope->iov, ope->iov_count, tx_data_offset);
	}
	assert(copied == data_size);
	pkt_entry->payload = data;
	pkt_entry->payload_size = copied;
	pkt_entry->payload_mr = pkt_entry->mr;
	pkt_entry->pkt_size = pkt_data_offset + copied;
	return 0;
}

/* @brief return the data size in a packet entry
 *
 * @param[in]	pkt_entry		packet entry
 * @return	the data size in the packet entry.
 * 		if the packet entry does not contain data,
 * 		return 0.
 */
size_t rxr_pkt_data_size(struct efa_rdm_pke *pkt_entry)
{
	int pkt_type;

	assert(pkt_entry);

	/*
	 * pkt entry from read copy pool only stores actual
	 * application data in pkt_entry->wiredata, so its
	 * data_size is just pkt_entry->pkt_size.
	 */
	if (pkt_entry->alloc_type == EFA_RDM_PKE_FROM_READ_COPY_POOL)
		return pkt_entry->pkt_size;

	pkt_type = rxr_get_base_hdr(pkt_entry->wiredata)->type;

	if (pkt_type == RXR_DATA_PKT)
		return rxr_get_data_hdr(pkt_entry->wiredata)->seg_length;

	if (pkt_type == RXR_READRSP_PKT)
		return rxr_get_readrsp_hdr(pkt_entry->wiredata)->seg_length;

	if (pkt_type >= RXR_REQ_PKT_BEGIN) {
		assert(pkt_type == RXR_EAGER_MSGRTM_PKT || pkt_type == RXR_EAGER_TAGRTM_PKT ||
		       pkt_type == RXR_MEDIUM_MSGRTM_PKT || pkt_type == RXR_MEDIUM_TAGRTM_PKT ||
		       pkt_type == RXR_LONGCTS_MSGRTM_PKT || pkt_type == RXR_LONGCTS_TAGRTM_PKT ||
		       pkt_type == RXR_EAGER_RTW_PKT ||
		       pkt_type == RXR_LONGCTS_RTW_PKT ||
		       pkt_type == RXR_DC_EAGER_MSGRTM_PKT ||
		       pkt_type == RXR_DC_EAGER_TAGRTM_PKT ||
		       pkt_type == RXR_DC_MEDIUM_MSGRTM_PKT ||
		       pkt_type == RXR_DC_MEDIUM_TAGRTM_PKT ||
		       pkt_type == RXR_DC_LONGCTS_MSGRTM_PKT ||
		       pkt_type == RXR_DC_LONGCTS_TAGRTM_PKT ||
		       pkt_type == RXR_DC_EAGER_RTW_PKT ||
		       pkt_type == RXR_DC_LONGCTS_RTW_PKT ||
		       pkt_type == RXR_RUNTREAD_MSGRTM_PKT ||
		       pkt_type == RXR_RUNTREAD_TAGRTM_PKT);

		return rxr_pkt_req_data_size(pkt_entry);
	}

	/* other packet type does not contain data, thus return 0
	 */
	return 0;
}

/**
 * @brief flush queued blocking copy to hmem
 *
 * The copying of data from bounce buffer to hmem receiving buffer
 * is queued, and we copy them in batch.
 *
 * This functions is used to flush all the queued hmem copy.
 *
 * It can be called in two scenarios:
 *
 * 1. the number of queued hmem copy reached limit
 *
 * 2. all the data of one of the queued message has arrived.
 *
 * @param[in,out]	ep	endpoint, where queue_copy_num and queued_copy_vec reside.
 *
 */
int efa_rdm_ep_flush_queued_blocking_copy_to_hmem(struct efa_rdm_ep *ep)
{
	size_t i;
	size_t bytes_copied[RXR_EP_MAX_QUEUED_COPY] = {0};
	struct efa_mr *desc;
	struct efa_rdm_ope *rxe;
	struct efa_rdm_pke *pkt_entry;
	char *data;
	size_t data_size, data_offset;

	for (i = 0; i < ep->queued_copy_num; ++i) {
		pkt_entry = ep->queued_copy_vec[i].pkt_entry;
		data = ep->queued_copy_vec[i].data;
		data_size = ep->queued_copy_vec[i].data_size;
		data_offset = ep->queued_copy_vec[i].data_offset;

		rxe = pkt_entry->ope;
		desc = rxe->desc[0];
		assert(desc && desc->peer.iface != FI_HMEM_SYSTEM);

		if (FI_HMEM_CUDA == desc->peer.iface &&
		    (desc->peer.flags & OFI_HMEM_DATA_GDRCOPY_HANDLE)) {
			assert(desc->peer.hmem_data);
			bytes_copied[i] = ofi_gdrcopy_to_cuda_iov((uint64_t)desc->peer.hmem_data,
			                                          rxe->iov, rxe->iov_count,
			                                          data_offset + ep->msg_prefix_size,
			                                          data, data_size);
		} else {
			bytes_copied[i] = ofi_copy_to_hmem_iov(desc->peer.iface,
			                                       desc->peer.device.reserved,
			                                       rxe->iov, rxe->iov_count,
			                                       data_offset + ep->msg_prefix_size,
			                                       data, data_size);
		}
	}

	for (i = 0; i < ep->queued_copy_num; ++i) {
		pkt_entry = ep->queued_copy_vec[i].pkt_entry;
		data_size = ep->queued_copy_vec[i].data_size;
		data_offset = ep->queued_copy_vec[i].data_offset;
		rxe = pkt_entry->ope;

		if (bytes_copied[i] != MIN(data_size, rxe->cq_entry.len - data_offset)) {
			EFA_WARN(FI_LOG_CQ, "wrong size! bytes_copied: %ld\n",
				bytes_copied[i]);
			return -FI_EIO;
		}

		rxe->bytes_queued_blocking_copy -= data_size;
		rxr_pkt_handle_data_copied(ep, pkt_entry, data_size);
	}

	ep->queued_copy_num = 0;
	return 0;
}

/*
 * @brief copy data to hmem buffer by queueing
 *
 * This function queue multiple (up to RXR_EP_MAX_QUEUED_COPY) copies to
 * device memory, and do them at the same time. This is to avoid any memory
 * barrier between copies, which will cause a flush.
 *
 * @param[in]		ep		endpoint
 * @param[in]		pkt_entry	the packet entry that contains data, which
 *                                      x_entry pointing to the correct rxe.
 * @param[in]		data		the pointer pointing to the beginning of data
 * @param[in]		data_size	the length of data
 * @param[in]		data_offset	the offset of the data in the packet in respect
 *					of the receiving buffer.
 * @return		On success, return 0
 * 			On failure, return libfabric error code
 */
static inline
int rxr_pkt_queued_copy_data_to_hmem(struct efa_rdm_ep *ep,
				     struct efa_rdm_pke *pkt_entry,
				     char *data,
				     size_t data_size,
				     size_t data_offset)
{
	struct efa_rdm_ope *rxe;

	assert(ep->queued_copy_num < RXR_EP_MAX_QUEUED_COPY);
	ep->queued_copy_vec[ep->queued_copy_num].pkt_entry = pkt_entry;
	ep->queued_copy_vec[ep->queued_copy_num].data = data;
	ep->queued_copy_vec[ep->queued_copy_num].data_size = data_size;
	ep->queued_copy_vec[ep->queued_copy_num].data_offset = data_offset;
	ep->queued_copy_num += 1;

	rxe = pkt_entry->ope;
	assert(rxe);
	rxe->bytes_queued_blocking_copy += data_size;

	if (ep->queued_copy_num < RXR_EP_MAX_QUEUED_COPY &&
	    rxe->bytes_copied + rxe->bytes_queued_blocking_copy < rxe->total_len) {
		return 0;
	}

	return efa_rdm_ep_flush_queued_blocking_copy_to_hmem(ep);
}

/* @brief copy data in pkt_entry to CUDA memory
 *
 * There are 3 ways to copy data to CUDA memory. None of them is guaranteed to
 * be available:
 *
 * gdrcopy, which is avaibale only when cuda_is_gdrcopy_enabled() is true
 *
 * cudaMemcpy, which is available only when endpoint is permitted to call CUDA api
 *
 * localread copy, which is available only when p2p is supported by device, and device support read.
 *
 * gdrcopy and cudaMemcpy is mutally exclusive, when they are both available, cudaMemcpy is used.
 * so we consider them as blocking copy.
 *
 * When neither blocking copy and localread copy is available, this function return error.
 *
 * When only one method is available, the availble one will be used.
 *
 * When both methods are available, we used a mixed approach, e.g.
 *
 * we use blocking copy up to certain number.
 *
 * For the rest of the receive buffers, we use local read copy.
 *
 * @param[in]		ep		endpoint
 * @param[in]		pkt_entry	the packet entry that contains data, which
 *                                      x_entry pointing to the correct rxe.
 * @param[in]		data		the pointer pointing to the beginning of data
 * @param[in]		data_size	the length of data
 * @param[in]		data_offset	the offset of the data in the packet in respect
 *					of the receiving buffer.
 * @return		On success, return 0
 * 			On failure, return libfabric error code
 */
static inline
int rxr_pkt_copy_data_to_cuda(struct efa_rdm_ep *ep,
			      struct efa_rdm_pke *pkt_entry,
			      char *data,
			      size_t data_size,
			      size_t data_offset)
{
	static const int max_blocking_copy_rxe_num = 4;
	struct efa_rdm_ope *rxe;
	struct efa_mr *desc;
	bool p2p_available, local_read_available, gdrcopy_available, cuda_memcpy_available;
	int ret, err;

	rxe = pkt_entry->ope;
	desc = rxe->desc[0];
	assert(efa_mr_is_cuda(desc));

	ret = efa_rdm_ep_use_p2p(ep, desc);
	if (ret < 0)
		return ret;

	p2p_available = ret;
	local_read_available = p2p_available && efa_rdm_ep_support_rdma_read(ep);
	cuda_memcpy_available = ep->cuda_api_permitted;
	gdrcopy_available = desc->peer.flags & OFI_HMEM_DATA_GDRCOPY_HANDLE;

	/* For in-order aligned send/recv, only allow local read to be used to copy data */
	if (ep->sendrecv_in_order_aligned_128_bytes) {
		cuda_memcpy_available = false;
		gdrcopy_available = false;
	}

	if (!local_read_available && !gdrcopy_available && !cuda_memcpy_available) {
		EFA_WARN(FI_LOG_CQ, "None of the copy methods: localread, gdrcopy or cudaMemcpy is available,"
			"thus libfabric is not able to copy received data to Nvidia GPU\n");
		return -FI_EINVAL;
	}

	if (!local_read_available) {
		assert(cuda_memcpy_available || gdrcopy_available);
		return rxr_pkt_queued_copy_data_to_hmem(ep, pkt_entry, data, data_size, data_offset);
	}

	assert(local_read_available);

	if (!gdrcopy_available) {
		/* prefer local read over cudaMemcpy (when it is available)
		 * because local read copy is faster
		 */
		err = efa_rdm_rxe_post_local_read_or_queue(rxe, data_offset,
							    pkt_entry, data, data_size);
		if (err)
			EFA_WARN(FI_LOG_CQ, "cannot post read to copy data\n");
		return err;
	}

	assert(gdrcopy_available && local_read_available);

	/* when both local read and gdrcopy are available, we use a mixed approach */

	if (rxe->cuda_copy_method != EFA_RDM_CUDA_COPY_LOCALREAD) {
		assert(rxe->bytes_copied + data_size <= rxe->total_len);

		/* If this packet is the last uncopied piece (or the only piece), copy it right away
		 * to achieve best latency.
		 */
		if (rxe->bytes_copied + data_size == rxe->total_len) {
			assert(desc->peer.hmem_data);
			ofi_gdrcopy_to_cuda_iov((uint64_t)desc->peer.hmem_data,
			                        rxe->iov, rxe->iov_count,
			                        data_offset + ep->msg_prefix_size,
			                        data, data_size);
			rxr_pkt_handle_data_copied(ep, pkt_entry, data_size);
			return 0;
		}

		/* If this rxe is already been chosen to use gdrcopy/cudaMemcpy, keep using on it */
		if (rxe->cuda_copy_method == EFA_RDM_CUDA_COPY_BLOCKING)
			return rxr_pkt_queued_copy_data_to_hmem(ep, pkt_entry, data, data_size, data_offset);

		/* If there are still empty slot for using gdrcopy, use gdrcopy on this rxe */
		if (rxe->cuda_copy_method == EFA_RDM_CUDA_COPY_UNSPEC && ep->blocking_copy_rxe_num < max_blocking_copy_rxe_num) {
			rxe->cuda_copy_method = EFA_RDM_CUDA_COPY_BLOCKING;
			ep->blocking_copy_rxe_num += 1;
			return rxr_pkt_queued_copy_data_to_hmem(ep, pkt_entry, data, data_size, data_offset);
		}
	}

	if (rxe->cuda_copy_method == EFA_RDM_CUDA_COPY_UNSPEC)
		rxe->cuda_copy_method = EFA_RDM_CUDA_COPY_LOCALREAD;

	err = efa_rdm_rxe_post_local_read_or_queue(rxe, data_offset,
						    pkt_entry, data, data_size);
	if (err)
		EFA_WARN(FI_LOG_CQ, "cannot post read to copy data\n");

	/* At this point data has NOT been copied yet, thus we cannot call
	 * rxr_pkt_handle_data_copied(). The function will be called
	 * when the completion of the local read request is received
	 * (by progress engine).
	 */
	return err;
}


/**
 * @brief copy data to application's receive buffer and update counter in rxe.
 *
 * Depend on when application's receive buffer is located (host or device) and
 * the software stack, this function will select different, strategies to copy data.
 *
 * When application's receive buffer is on device, there are two scenarios:
 *
 *    If memory is on cuda GPU, and gdrcopy is not available, this function
 *    will post a local read request to copy data. (This is because NCCL forbids its
 *    plugin to make cuda calls). In this case, the data is not copied upon return of
 *    this function, and the function rxr_pkt_handle_copied() is not called. It will
 *    be called upon the completion of local read operation by the progress engine.
 *
 *    Otherwise, this function calls rxr_pkt_copy_data_to_hmem(), which will batch
 *    multiple copies, and perform the copy (then call rxr_pkt_handle_copied()) together
 *    to improve performance.
 *
 * When application's receive buffer is on host, data is copied immediately, and
 * rxr_pkt_handle_copied() is called.
 *
 * @param[in]		ep		endpoint
 * @param[in,out]	ope	ope contains information of the receive
 *                      	        op. This function uses receive buffer in it.
 * @param[in]		data_offset	data offset in the packet in the receiving buffer.
 * @param[in]		pkt_entry	the packet entry that contains data
 * @param[in]		data		the pointer pointing to the beginning of data
 * @param[in]		data_size	the length of data
 * @return		On success, return 0
 * 			On failure, return libfabric error code
 */
ssize_t rxr_pkt_copy_data_to_ope(struct efa_rdm_ep *ep,
				      struct efa_rdm_ope *ope,
				      size_t data_offset,
				      struct efa_rdm_pke *pkt_entry,
				      char *data, size_t data_size)
{
	struct efa_mr *desc;
	ssize_t bytes_copied;

	pkt_entry->ope = ope;

	/*
	 * Under 3 rare situations, this function does not perform the copy
	 * action, but still consider data is copied:
	 *
	 * 1. application cancelled the receive, thus the receive buffer is not
	 *    available for copying to. In the case, this function is still
	 *    called because sender will keep sending data as receiver did not
	 *    notify the sender about the cancelation,
	 *
	 * 2. application's receiving buffer is smaller than incoming message size,
	 *    and data in the packet is outside of receiving buffer (truncated message).
	 *    In this case, this function is still called because sender will
	 *    keep sending data as receiver did not notify the sender about the
	 *    truncation.
	 *
	 * 3. message size is 0, thus no data to copy.
	 */
	if (OFI_UNLIKELY((ope->rxr_flags & EFA_RDM_RXE_RECV_CANCEL)) ||
	    OFI_UNLIKELY(data_offset >= ope->cq_entry.len) ||
	    OFI_UNLIKELY(data_size == 0)) {
		rxr_pkt_handle_data_copied(ep, pkt_entry, data_size);
		return 0;
	}

	desc = ope->desc[0];

	if (efa_mr_is_cuda(desc))
		return rxr_pkt_copy_data_to_cuda(ep, pkt_entry, data, data_size, data_offset);

	if (efa_mr_is_hmem(desc))
		return rxr_pkt_queued_copy_data_to_hmem(ep, pkt_entry, data, data_size, data_offset);

	assert( !desc || desc->peer.iface == FI_HMEM_SYSTEM);
	bytes_copied = ofi_copy_to_iov(ope->iov, ope->iov_count,
				       data_offset + ep->msg_prefix_size,
				       data, data_size);

	if (bytes_copied != MIN(data_size, ope->cq_entry.len - data_offset)) {
		EFA_WARN(FI_LOG_CQ, "wrong size! bytes_copied: %ld\n",
			bytes_copied);
		return -FI_EIO;
	}

	rxr_pkt_handle_data_copied(ep, pkt_entry, data_size);
	return 0;
}
