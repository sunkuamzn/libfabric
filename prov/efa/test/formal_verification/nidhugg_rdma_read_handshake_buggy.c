/*
 * Nidhugg formal verification test — EXPECTED FAILURE
 *
 * Demonstrates what goes wrong if the RDMA read gate only checks
 * the handshake flag without verifying the peer's RDMA_READ feature bit.
 *
 * This models a hypothetical bug: someone replaces the correct
 * efa_rdm_peer_support_rdma_read() check with just:
 *   if (peer->flags & EFA_RDM_PEER_HANDSHAKE_RECEIVED)
 *       post_rdma_read();
 *
 * Nidhugg will find the interleaving:
 *   1. handshake_thread: sets extra_info[0] = 0 (no RDMA read)
 *   2. handshake_thread: sets HANDSHAKE_RECEIVED
 *   3. sender_thread: sees HANDSHAKE_RECEIVED, posts RDMA read
 *   => Assertion violation: peer doesn't support RDMA read!
 *
 * Build & run (from libfabric root):
 *   make -C prov/efa/test/formal_verification test_buggy
 *
 * Or manually:
 *   ROOT=/path/to/libfabric
 *   clang -emit-llvm -S -g -std=gnu11 \
 *       -I${ROOT} -I${ROOT}/include \
 *       -I${ROOT}/prov/efa/src \
 *       -I${ROOT}/prov/efa/src/rdm \
 *       nidhugg_rdma_read_handshake_buggy.c -o buggy.ll
 *   nidhugg --sc buggy.ll   # EXPECTED: assertion violation
 */

#include "config.h"
#include "efa_rdm_ep.h"

#include <pthread.h>
#include <assert.h>
#include <string.h>

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

	g_domain.device = &g_device;
	g_ep.base_ep.util_ep.domain = &g_domain.util_domain;

	g_ep.use_device_rdma = true;
	g_device.device_caps = EFADV_DEVICE_ATTR_CAPS_RDMA_READ;
	g_device.ibv_attr.vendor_part_id = 0xEFA1;

	g_peer.flags = 0;
	g_peer.extra_info[0] = 0;
	g_peer.device_version = 0xEFA1;
	g_peer.is_self = false;
	g_peer.ep = &g_ep;
}

/*
 * BUGGY gate: only checks handshake received, skips the feature flag.
 * This is NOT what production does — it demonstrates what would break.
 */
static bool buggy_rdma_read_check(struct efa_rdm_ep *ep, struct efa_rdm_peer *peer)
{
	if (!ep->use_device_rdma)
		return false;

	/* BUG: should also check efa_rdm_peer_support_rdma_read(peer) */
	return (peer->flags & EFA_RDM_PEER_HANDSHAKE_RECEIVED);
}

/*
 * Sender using the buggy check — will sometimes post RDMA read
 * to a peer that doesn't support it.
 */
static void *sender_thread(void *arg)
{
	(void)arg;

	if (buggy_rdma_read_check(&g_ep, &g_peer)) {
		/*
		 * ASSERTION: peer actually supports RDMA read.
		 * This WILL FAIL because the peer does NOT advertise RDMA_READ.
		 */
		assert(g_peer.extra_info[0] & EFA_RDM_EXTRA_FEATURE_RDMA_READ);
	}

	return NULL;
}

/*
 * Handshake from a peer that does NOT support RDMA read.
 */
static void *handshake_thread(void *arg)
{
	(void)arg;

	g_peer.extra_info[0] = 0; /* no RDMA read support */
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
