/*
 * Nidhugg formal verification test:
 * Verify that an EFA RDM endpoint never posts an RDMA read to a peer
 * before receiving that peer's handshake.
 *
 * This test includes the real production headers and calls the actual
 * inline functions (efa_rdm_peer_support_rdma_read, efa_both_support_rdma_read,
 * efa_rdm_interop_rdma_read). No logic is duplicated.
 *
 * Prerequisites:
 *   - Linux with libfabric configured (./configure --enable-efa)
 *   - Nidhugg (https://github.com/nidhugg/nidhugg)
 *   - Or GenMC (https://github.com/MPI-SWS/GenMC)
 *
 * Build & run (from libfabric root):
 *   make -C prov/efa/test/formal_verification test
 *
 * Or manually:
 *   ROOT=/path/to/libfabric
 *   clang -emit-llvm -S -g -std=gnu11 \
 *       -I${ROOT} -I${ROOT}/include \
 *       -I${ROOT}/prov/efa/src \
 *       -I${ROOT}/prov/efa/src/rdm \
 *       prov/efa/test/formal_verification/nidhugg_rdma_read_handshake.c \
 *       -o test.ll
 *   nidhugg --sc test.ll
 *   nidhugg --tso test.ll
 *   nidhugg --arm test.ll
 */

#include "config.h"
#include "efa_rdm_ep.h"

#include <stdatomic.h>
#include <pthread.h>
#include <assert.h>
#include <string.h>

/*
 * Minimal struct setup to wire the container_of chain:
 *   ep->base_ep.util_ep.domain -> &domain.util_domain
 *   domain.device -> &device
 *
 * These are statically allocated and zero-initialized. We set only
 * the fields that the efa_rdm_interop_rdma_read() call chain reads.
 */
static struct efa_device g_device;
static struct efa_domain g_domain;
static struct efa_rdm_ep g_ep;
static struct efa_rdm_peer g_peer;

static void setup(void)
{
	memset(&g_device, 0, sizeof(g_device));
	memset(&g_domain, 0, sizeof(g_domain));
	memset(&g_ep, 0, sizeof(g_ep));
	memset(&g_peer, 0, sizeof(g_peer));

	/* Wire the container_of chain: ep -> domain -> device */
	g_domain.device = &g_device;
	g_ep.base_ep.util_ep.domain = &g_domain.util_domain;

	/* Local endpoint supports RDMA read */
	g_ep.use_device_rdma = true;
	g_device.device_caps = EFADV_DEVICE_ATTR_CAPS_RDMA_READ;
	g_device.ibv_attr.vendor_part_id = 0xEFA1; /* newer platform */

	/* Peer: no handshake received yet, no known capabilities */
	g_peer.flags = 0;
	g_peer.extra_info[0] = 0;
	g_peer.device_version = 0xEFA1; /* newer platform */
	g_peer.is_self = false;
	g_peer.ep = &g_ep;
}

/*
 * Thread 1: Sender — attempts to select a read-based RTM protocol.
 *
 * Calls the REAL efa_rdm_interop_rdma_read() function which gates
 * on efa_both_support_rdma_read() -> efa_rdm_peer_support_rdma_read().
 */
static void *sender_thread(void *arg)
{
	(void)arg;

	if (efa_rdm_interop_rdma_read(&g_ep, &g_peer)) {
		/*
		 * PROPERTY UNDER VERIFICATION:
		 * An RDMA read is only posted if the handshake has been received.
		 * Nidhugg checks this under all possible interleavings.
		 */
		assert(g_peer.flags & EFA_RDM_PEER_HANDSHAKE_RECEIVED);
	}

	return NULL;
}

/*
 * Thread 2: Progress engine — processes an incoming handshake packet.
 *
 * Mirrors efa_rdm_pke_proc_handshake():
 *   1. Copy peer's extra_info (feature flags including RDMA_READ)
 *   2. Set EFA_RDM_PEER_HANDSHAKE_RECEIVED in peer->flags
 */
static void *handshake_thread(void *arg)
{
	(void)arg;

	/* Peer advertises RDMA read support in its handshake */
	g_peer.extra_info[0] = EFA_RDM_EXTRA_FEATURE_RDMA_READ;
	g_peer.flags |= EFA_RDM_PEER_HANDSHAKE_RECEIVED;

	return NULL;
}

int main(void)
{
	setup();

	pthread_t t1, t2;
	pthread_create(&t1, NULL, sender_thread, NULL);
	pthread_create(&t2, NULL, handshake_thread, NULL);
	pthread_join(t1, NULL);
	pthread_join(t2, NULL);

	return 0;
}
