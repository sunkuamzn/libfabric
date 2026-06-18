# EFA RDM Formal Verification Tests (Nidhugg)

Stateless model checking tests that exhaustively explore all thread
interleavings to verify protocol invariants in the EFA RDM provider.

These tests include the **real production headers** and call the actual
inline functions — no logic is duplicated, so there's no drift between
the test and the production code.

## Prerequisites

- **Linux** (libfabric uses Linux-specific headers: `asm/types.h`, `infiniband/verbs.h`)
- **libfabric configured** with EFA enabled (`./configure --enable-efa`)
- **clang** (for compiling to LLVM IR)
- **Nidhugg** or **GenMC**

### Installing Nidhugg

```bash
# Ubuntu/Debian
sudo apt install nidhugg

# From source: https://github.com/nidhugg/nidhugg
git clone https://github.com/nidhugg/nidhugg.git
cd nidhugg && ./configure && make && sudo make install
```

### Installing GenMC (alternative)

```bash
# From source: https://github.com/MPI-SWS/GenMC
git clone https://github.com/MPI-SWS/GenMC.git
cd GenMC && mkdir build && cd build && cmake .. && make && sudo make install
```

## Running

```bash
# From libfabric root (after ./configure --enable-efa):
make -C prov/efa/test/formal_verification test

# Or run the buggy variant (demonstrates what Nidhugg catches):
make -C prov/efa/test/formal_verification test_buggy
```

### Manual invocation

```bash
ROOT=/path/to/libfabric
cd prov/efa/test/formal_verification

# Compile
clang -emit-llvm -S -g -std=gnu11 \
    -I${ROOT} -I${ROOT}/include \
    -I${ROOT}/prov/efa/src -I${ROOT}/prov/efa/src/rdm \
    nidhugg_rdma_read_handshake.c -o test.ll

# Verify under different memory models
nidhugg --sc test.ll     # Sequential Consistency
nidhugg --tso test.ll    # x86 TSO
nidhugg --arm test.ll    # ARM (weakest)
```

## Tests

### `nidhugg_rdma_read_handshake.c` — SHOULD PASS

Verifies: **an endpoint never posts an RDMA read before receiving the
peer's handshake.**

Models two concurrent threads:
- **Sender thread**: calls the real `efa_rdm_interop_rdma_read(ep, peer)`
  to decide whether to use the read-based RTM protocol
- **Handshake thread**: simulates the progress engine receiving a
  handshake packet (sets `extra_info[0]` then `HANDSHAKE_RECEIVED`)

The assertion inside the sender verifies that if the gate returns true,
`EFA_RDM_PEER_HANDSHAKE_RECEIVED` is set. Nidhugg explores all possible
interleavings of these two threads.

### `nidhugg_rdma_read_handshake_buggy.c` — SHOULD FAIL

Demonstrates the value of `efa_rdm_peer_support_rdma_read()` checking
both the handshake flag AND the feature bit. The buggy variant only
checks `HANDSHAKE_RECEIVED`, which allows posting RDMA read to a peer
that doesn't support it.

Nidhugg finds the violating interleaving:
1. Handshake thread sets `extra_info[0] = 0` (no RDMA read)
2. Handshake thread sets `HANDSHAKE_RECEIVED`
3. Sender sees `HANDSHAKE_RECEIVED`, posts RDMA read
4. Assertion fails: peer doesn't have `EFA_RDM_EXTRA_FEATURE_RDMA_READ`

## Adding new tests

Good candidates for additional formal verification:

- **Peer removal vs in-flight ops**: verify generation counter prevents
  use-after-free (recent commit `e68f33ae4`)
- **RNR backoff state machine**: verify no sends during backoff period
- **Reorder buffer ordering**: verify msg_id monotonicity per-peer
- **Connection ID after handshake**: verify connid included only after
  handshake is received
- **Handshake queue retry**: verify queued handshakes are eventually sent

Pattern for new tests:
1. Include the real headers (`#include "efa_rdm_ep.h"` etc.)
2. Set up structs with `memset` + wire the `container_of` chain
3. One thread for the "actor", one for the concurrent event
4. Assert the invariant inside the decision branch
5. Run under SC, TSO, ARM memory models
