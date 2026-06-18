/*
 * Nidhugg/GenMC formal verification test:
 * Check for data races between efa_rdm_msg_send and efa_rdm_cq_readfrom
 * (the real implementations behind fi_send and fi_cq_read).
 *
 * This test calls the actual EFA provider functions directly — not
 * reimplementations. The rdma-core layer is mocked via --wrap (same as
 * the existing unit tests).
 *
 * Two threads run concurrently:
 *   Thread 1: calls efa_rdm_msg_send (the real fi_send implementation)
 *   Thread 2: calls fi_cq_read via the ops vtable (dispatches to efa_rdm_cq_readfrom)
 *
 * Nidhugg explores all thread interleavings to verify:
 *   - The srx_lock correctly serializes access to shared state
 *   - No data races on ep->efa_outstanding_tx_ops, peer->next_msg_id, etc.
 *   - Provider assertions (assert()) never fire
 *
 * Build requirements:
 *   This test must be linked with the EFA provider object files and libfabric
 *   core, exactly like the existing unit tests. It uses the same --wrap mocks.
 *
 *   For Nidhugg: compile all sources to LLVM IR, llvm-link into one module.
 *   For TSan: compile normally with -fsanitize=thread.
 *
 *   ROOT=/path/to/libfabric
 *
 *   # Option 1: ThreadSanitizer (dynamic, fast but non-exhaustive)
 *   gcc -fsanitize=thread -g -std=gnu11 \
 *       -I${ROOT} -I${ROOT}/include \
 *       -I${ROOT}/prov/efa/src -I${ROOT}/prov/efa/src/rdm \
 *       -DEFA_UNIT_TEST=1 \
 *       nidhugg_fi_send_cq_read_race.c \
 *       <all EFA .o files> <libfabric core .o files> \
 *       -Wl,--wrap=efa_qp_post_send \
 *       -Wl,--wrap=efa_ibv_cq_start_poll \
 *       ... (same wraps as unit tests) \
 *       -lpthread -o test_race
 *   ./test_race
 *
 *   # Option 2: Nidhugg (exhaustive, requires llvm-link)
 *   # Compile each .c to .ll, llvm-link, then:
 *   nidhugg --sc combined.ll
 */

#include "efa_unit_tests.h"

#include <pthread.h>

/*
 * The real provider functions we call directly.
 * efa_rdm_msg_send is non-static — it's the fi_ops_msg.send implementation.
 */
extern ssize_t efa_rdm_msg_send(struct fid_ep *ep, const void *buf, size_t len,
				void *desc, fi_addr_t dest_addr, void *context);

/*
 * We call fi_cq_read() which is an inline that dispatches through:
 *   cq->ops->read (= ofi_cq_read) -> fi_cq_readfrom -> cq->ops->readfrom (= efa_rdm_cq_readfrom)
 *
 * This exercises the real EFA CQ read path including the srx_lock acquisition
 * and the progress engine.
 */

/*
 * Test fixture — same pattern as efa_unit_test_common.c but minimal.
 * We set up the struct state that efa_rdm_msg_send and fi_cq_read need.
 */
static struct efa_resource g_resource;
static char g_send_buf[64];

/*
 * setup_resource: Initialize libfabric objects for the test.
 * This is the same as efa_unit_test_resource_construct() — it calls
 * fi_getinfo, fi_fabric, fi_domain, fi_endpoint, fi_cq_open, fi_enable.
 *
 * We use the existing unit test infrastructure for this setup.
 */
static void setup_resource(void)
{
	struct fi_av_attr av_attr = {0};
	struct fi_cq_attr cq_attr = { .format = FI_CQ_FORMAT_DATA };
	struct fi_info *hints;
	int ret;

	hints = fi_allocinfo();
	assert(hints);
	hints->ep_attr->type = FI_EP_RDM;
	hints->fabric_attr->prov_name = strdup("efa");
	/* Request FI_THREAD_SAFE so srx_lock is a real mutex */
	hints->domain_attr->threading = FI_THREAD_SAFE;

	ret = fi_getinfo(FI_VERSION(2, 0), NULL, NULL, 0, hints, &g_resource.info);
	assert(ret == 0);

	ret = fi_fabric(g_resource.info->fabric_attr, &g_resource.fabric, NULL);
	assert(ret == 0);

	ret = fi_domain(g_resource.fabric, g_resource.info, &g_resource.domain, NULL);
	assert(ret == 0);

	ret = fi_endpoint(g_resource.domain, g_resource.info, &g_resource.ep, NULL);
	assert(ret == 0);

	ret = fi_av_open(g_resource.domain, &av_attr, &g_resource.av, NULL);
	assert(ret == 0);
	fi_ep_bind(g_resource.ep, &g_resource.av->fid, 0);

	ret = fi_cq_open(g_resource.domain, &cq_attr, &g_resource.cq, NULL);
	assert(ret == 0);
	fi_ep_bind(g_resource.ep, &g_resource.cq->fid, FI_SEND | FI_RECV);

	ret = fi_enable(g_resource.ep);
	assert(ret == 0);

	/* Insert a peer address so we have a valid dest_addr */
	struct efa_ep_addr raw_addr;
	size_t raw_addr_len = sizeof(raw_addr);
	fi_getname(&g_resource.ep->fid, &raw_addr, &raw_addr_len);
	raw_addr.qpn = 1;
	raw_addr.qkey = 0x1234;
	fi_addr_t addr;
	ret = fi_av_insert(g_resource.av, &raw_addr, 1, &addr, 0, NULL);
	assert(ret == 1);

	fi_freeinfo(hints);
}

static void teardown_resource(void)
{
	fi_close(&g_resource.ep->fid);
	fi_close(&g_resource.cq->fid);
	fi_close(&g_resource.av->fid);
	fi_close(&g_resource.domain->fid);
	fi_close(&g_resource.fabric->fid);
	fi_freeinfo(g_resource.info);
}

/*
 * Thread 1: Application send thread.
 * Calls the real efa_rdm_msg_send (= fi_send's backend).
 */
static void *send_thread(void *arg)
{
	(void)arg;
	ssize_t ret;

	ret = fi_send(g_resource.ep, g_send_buf, sizeof(g_send_buf),
		      NULL, 0 /* dest_addr */, NULL /* context */);

	/* -FI_EAGAIN is normal (peer in backoff, resource exhaustion) */
	assert(ret == 0 || ret == -FI_EAGAIN);
	return NULL;
}

/*
 * Thread 2: Application CQ polling thread.
 * Calls the real fi_cq_read (-> ofi_cq_read -> efa_rdm_cq_readfrom).
 */
static void *cq_thread(void *arg)
{
	(void)arg;
	struct fi_cq_data_entry cq_entry;
	ssize_t ret;

	ret = fi_cq_read(g_resource.cq, &cq_entry, 1);

	/* -FI_EAGAIN = empty CQ (normal), positive = completions read */
	assert(ret > 0 || ret == -FI_EAGAIN);
	return NULL;
}

int main(void)
{
	setup_resource();

	pthread_t t_send, t_cq;
	pthread_create(&t_send, NULL, send_thread, NULL);
	pthread_create(&t_cq, NULL, cq_thread, NULL);
	pthread_join(t_send, NULL);
	pthread_join(t_cq, NULL);

	teardown_resource();
	return 0;
}
