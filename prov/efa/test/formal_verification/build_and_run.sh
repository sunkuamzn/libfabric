#!/bin/bash
#
# Build and run formal verification tests for EFA RDM provider.
#
# Prerequisites:
#   - Linux with EFA-capable rdma-core installed
#   - libfabric configured: ./configure --enable-efa --enable-efa-unit-test
#   - nidhugg and/or genmc installed
#   - cmocka installed (for the fi_send/fi_cq_read test)
#
# Usage:
#   ./build_and_run.sh [test_name] [tool]
#
#   test_name: handshake | send_cq | all (default: all)
#   tool:      nidhugg | genmc | tsan (default: nidhugg)

set -e

ROOT="$(git rev-parse --show-toplevel)"
TESTDIR="${ROOT}/prov/efa/test/formal_verification"
TOOL="${2:-nidhugg}"
TEST="${1:-all}"

CFLAGS="-g -std=gnu11 -DEFA_UNIT_TEST=1"
INCLUDES="-I${ROOT} -I${ROOT}/include -I${ROOT}/prov/efa/src -I${ROOT}/prov/efa/src/rdm -I${ROOT}/prov/efa/test"

# ============================================================
# Test 1: Handshake/RDMA-read race (self-contained, no linking needed)
# ============================================================
build_handshake() {
    echo "=== Building: nidhugg_rdma_read_handshake ==="
    cd "${TESTDIR}"

    case "${TOOL}" in
        nidhugg)
            clang -emit-llvm -S ${CFLAGS} ${INCLUDES} \
                nidhugg_rdma_read_handshake.c -o handshake.ll
            ;;
        genmc)
            # GenMC compiles directly, no .ll step needed
            echo "(GenMC compiles on the fly)"
            ;;
        tsan)
            clang ${CFLAGS} -fsanitize=thread ${INCLUDES} \
                nidhugg_rdma_read_handshake.c -lpthread -o handshake_tsan
            ;;
    esac
}

run_handshake() {
    echo "=== Running: nidhugg_rdma_read_handshake ==="
    cd "${TESTDIR}"

    case "${TOOL}" in
        nidhugg)
            echo "--- Sequential Consistency ---"
            nidhugg --sc handshake.ll
            echo "--- TSO (x86) ---"
            nidhugg --tso handshake.ll
            echo "--- ARM ---"
            nidhugg --arm handshake.ll
            ;;
        genmc)
            echo "--- RC11 (default) ---"
            genmc ${CFLAGS} ${INCLUDES} -- nidhugg_rdma_read_handshake.c
            ;;
        tsan)
            echo "--- ThreadSanitizer (100 iterations) ---"
            for i in $(seq 1 100); do
                ./handshake_tsan
            done
            echo "PASS: no races detected in 100 runs"
            ;;
    esac
}

build_handshake_buggy() {
    echo "=== Building: nidhugg_rdma_read_handshake_buggy (expect failure) ==="
    cd "${TESTDIR}"

    case "${TOOL}" in
        nidhugg)
            clang -emit-llvm -S ${CFLAGS} ${INCLUDES} \
                nidhugg_rdma_read_handshake_buggy.c -o handshake_buggy.ll
            ;;
        genmc)
            echo "(GenMC compiles on the fly)"
            ;;
        tsan)
            clang ${CFLAGS} -fsanitize=thread ${INCLUDES} \
                nidhugg_rdma_read_handshake_buggy.c -lpthread -o handshake_buggy_tsan
            ;;
    esac
}

run_handshake_buggy() {
    echo "=== Running: nidhugg_rdma_read_handshake_buggy (EXPECT FAILURE) ==="
    cd "${TESTDIR}"

    case "${TOOL}" in
        nidhugg)
            echo "--- SC (should find assertion violation) ---"
            nidhugg --sc handshake_buggy.ll || echo "^^^ Expected assertion violation found"
            ;;
        genmc)
            genmc ${CFLAGS} ${INCLUDES} -- nidhugg_rdma_read_handshake_buggy.c || \
                echo "^^^ Expected assertion violation found"
            ;;
        tsan)
            ./handshake_buggy_tsan || echo "^^^ Expected failure"
            ;;
    esac
}

# ============================================================
# Test 2: fi_send / fi_cq_read race (requires full EFA linkage)
# ============================================================

# EFA provider source files
EFA_SRCS="
    prov/efa/src/efa_device.c
    prov/efa/src/efa_hmem.c
    prov/efa/src/efa_shm.c
    prov/efa/src/efa_av.c
    prov/efa/src/efa_ah.c
    prov/efa/src/efa_conn.c
    prov/efa/src/efa_domain.c
    prov/efa/src/efa_fabric.c
    prov/efa/src/efa_mr.c
    prov/efa/src/efa_strerror.c
    prov/efa/src/efa_user_info.c
    prov/efa/src/efa_prov_info.c
    prov/efa/src/efa_fork_support.c
    prov/efa/src/efa_tp_def.c
    prov/efa/src/efa_base_ep.c
    prov/efa/src/efa_prov.c
    prov/efa/src/efa_env.c
    prov/efa/src/efa_cntr.c
    prov/efa/src/efa_msg.c
    prov/efa/src/efa_rma.c
    prov/efa/src/efa_cq.c
    prov/efa/src/efa_ep.c
    prov/efa/src/rdm/efa_rdm_peer.c
    prov/efa/src/rdm/efa_rdm_cq.c
    prov/efa/src/rdm/efa_rdm_ep_utils.c
    prov/efa/src/rdm/efa_rdm_ep_fiops.c
    prov/efa/src/rdm/efa_rdm_rma.c
    prov/efa/src/rdm/efa_rdm_msg.c
    prov/efa/src/rdm/efa_rdm_pke.c
    prov/efa/src/rdm/efa_rdm_pke_utils.c
    prov/efa/src/rdm/efa_rdm_pke_rtm.c
    prov/efa/src/rdm/efa_rdm_pke_rtr.c
    prov/efa/src/rdm/efa_rdm_pke_rtw.c
    prov/efa/src/rdm/efa_rdm_pke_rta.c
    prov/efa/src/rdm/efa_rdm_pke_req.c
    prov/efa/src/rdm/efa_rdm_pke_nonreq.c
    prov/efa/src/rdm/efa_rdm_pke_cmd.c
    prov/efa/src/rdm/efa_rdm_pke_print.c
    prov/efa/src/rdm/efa_rdm_pkt_type.c
    prov/efa/src/rdm/efa_rdm_ope.c
    prov/efa/src/rdm/efa_rdm_rxe_map.c
    prov/efa/src/rdm/efa_rdm_atomic.c
    prov/efa/src/rdm/efa_rdm_tracepoint_def.c
    prov/efa/src/rdm/efa_rdm_srx.c
    prov/efa/src/rdm/efa_rdm_util.c
"

# Unit test mock/common files (provides setup infrastructure)
TEST_SRCS="
    prov/efa/test/efa_unit_test_mocks.c
    prov/efa/test/efa_unit_test_common.c
"

# --wrap flags for mocking rdma-core
WRAP_FLAGS="
    -Wl,--wrap=ibv_create_ah
    -Wl,--wrap=ibv_destroy_ah
    -Wl,--wrap=ibv_is_fork_initialized
    -Wl,--wrap=efadv_query_device
    -Wl,--wrap=efa_ah_alloc
    -Wl,--wrap=efa_ah_release
    -Wl,--wrap=ofi_cudaMalloc
    -Wl,--wrap=ofi_copy_from_hmem_iov
    -Wl,--wrap=efa_rdm_pke_copy_payload_to_ope
    -Wl,--wrap=efa_rdm_pke_read
    -Wl,--wrap=efa_rdm_pke_proc_matched_rtm
    -Wl,--wrap=efa_rdm_ope_post_send
    -Wl,--wrap=efa_device_support_unsolicited_write_recv
    -Wl,--wrap=efa_qp_post_recv
    -Wl,--wrap=efa_qp_post_send
    -Wl,--wrap=efa_qp_post_read
    -Wl,--wrap=efa_qp_post_write
    -Wl,--wrap=efa_ibv_cq_start_poll
    -Wl,--wrap=efa_ibv_cq_next_poll
    -Wl,--wrap=efa_ibv_cq_end_poll
    -Wl,--wrap=efa_ibv_cq_wc_read_opcode
    -Wl,--wrap=efa_ibv_cq_wc_read_qp_num
    -Wl,--wrap=efa_ibv_cq_wc_read_vendor_err
    -Wl,--wrap=efa_ibv_cq_wc_read_src_qp
    -Wl,--wrap=efa_ibv_cq_wc_read_slid
    -Wl,--wrap=efa_ibv_cq_wc_read_byte_len
    -Wl,--wrap=efa_ibv_cq_wc_read_wc_flags
    -Wl,--wrap=efa_ibv_cq_wc_read_imm_data
    -Wl,--wrap=efa_ibv_cq_wc_is_unsolicited
    -Wl,--wrap=efa_ibv_cq_wc_read_sgid
    -Wl,--wrap=efa_ibv_get_cq_event
    -Wl,--wrap=efa_ibv_req_notify_cq
"

build_send_cq_tsan() {
    echo "=== Building: fi_send/fi_cq_read race test (TSan) ==="
    cd "${ROOT}"

    # Build libfabric first (needed for linkback)
    if [ ! -f src/.libs/libfabric.so ]; then
        echo "Building libfabric... (run ./configure --enable-efa --enable-efa-unit-test first)"
        make -j$(nproc)
    fi

    # Compile the race test with ThreadSanitizer
    gcc -g -std=gnu11 -fsanitize=thread \
        -DEFA_UNIT_TEST=1 \
        ${INCLUDES} \
        ${TESTDIR}/nidhugg_fi_send_cq_read_race.c \
        ${TEST_SRCS} \
        ${EFA_SRCS} \
        src/.libs/libfabric.a \
        ${WRAP_FLAGS} \
        -lcmocka -lpthread -lrt -ldl -libverbs -lefadv \
        -o ${TESTDIR}/send_cq_tsan

    echo "Build successful: ${TESTDIR}/send_cq_tsan"
}

build_send_cq_nidhugg() {
    echo "=== Building: fi_send/fi_cq_read race test (Nidhugg) ==="
    cd "${ROOT}"

    # Compile all sources to LLVM IR
    mkdir -p ${TESTDIR}/build_ll

    echo "Compiling EFA sources to LLVM IR..."
    for src in ${EFA_SRCS} ${TEST_SRCS}; do
        base=$(basename ${src} .c)
        clang -emit-llvm -S ${CFLAGS} ${INCLUDES} \
            "${ROOT}/${src}" -o "${TESTDIR}/build_ll/${base}.ll" 2>/dev/null || \
            echo "  WARN: failed to compile ${src}"
    done

    # Compile the test itself
    clang -emit-llvm -S ${CFLAGS} ${INCLUDES} \
        "${TESTDIR}/nidhugg_fi_send_cq_read_race.c" \
        -o "${TESTDIR}/build_ll/nidhugg_fi_send_cq_read_race.ll"

    # Link all .ll files into one module
    echo "Linking LLVM IR modules..."
    llvm-link ${TESTDIR}/build_ll/*.ll -S -o ${TESTDIR}/send_cq_combined.ll

    echo "Build successful: ${TESTDIR}/send_cq_combined.ll"
}

run_send_cq() {
    echo "=== Running: fi_send/fi_cq_read race test ==="

    case "${TOOL}" in
        nidhugg)
            echo "--- Sequential Consistency ---"
            nidhugg --sc ${TESTDIR}/send_cq_combined.ll
            echo "--- TSO (x86) ---"
            nidhugg --tso ${TESTDIR}/send_cq_combined.ll
            ;;
        genmc)
            echo "GenMC requires single-file compilation."
            echo "Use: genmc ${CFLAGS} ${INCLUDES} -- nidhugg_fi_send_cq_read_race.c"
            echo "(Requires all sources to be #included or pre-linked)"
            ;;
        tsan)
            echo "--- ThreadSanitizer (100 iterations) ---"
            for i in $(seq 1 100); do
                ${TESTDIR}/send_cq_tsan
            done
            echo "PASS: no races detected in 100 runs"
            ;;
    esac
}

# ============================================================
# Main dispatch
# ============================================================

case "${TEST}" in
    handshake)
        build_handshake
        run_handshake
        echo ""
        build_handshake_buggy
        run_handshake_buggy
        ;;
    send_cq)
        case "${TOOL}" in
            tsan)    build_send_cq_tsan ;;
            nidhugg) build_send_cq_nidhugg ;;
        esac
        run_send_cq
        ;;
    all)
        build_handshake
        run_handshake
        echo ""
        build_handshake_buggy
        run_handshake_buggy
        echo ""
        case "${TOOL}" in
            tsan)    build_send_cq_tsan ;;
            nidhugg) build_send_cq_nidhugg ;;
        esac
        run_send_cq
        ;;
    *)
        echo "Usage: $0 [handshake|send_cq|all] [nidhugg|genmc|tsan]"
        exit 1
        ;;
esac

echo ""
echo "=== Done ==="
