from collections import namedtuple

import pytest
from common import ClientServerTest


pytestmark = pytest.mark.pre_release


# fi_mr_abort fabtest will allocate MR_ABORT_NUM_MRS and attempt
# to post N transfers per MR until the provider returns -FI_EAGAIN
# or we posted transactions for each MR. A larger number corresponds
# to increased load on the NIC and increased memory preassure on the
# instance.
MR_ABORT_NUM_MRS = 2046

BASE_MESSAGE_SIZES = [
    64,
    4096,
    65536,
    1048576,
    10485760,
]

SL_LOW_LATENCY_MAX_WRITE_SIZE = 128


def combined_msg_size_params():
    for rma_op in ["write", "read", "writedata"]:
        for size in BASE_MESSAGE_SIZES:

            high_pps_vals = [None]
            sl_low_latency_vals = [None]

            if (rma_op == "write" or rma_op == "writedata"):
                # High PPS parametrization is only applicable for RDMA writes
                high_pps_vals = [True, False]

                # Low latency SL is only applicable for small RDMA writes
                if size < SL_LOW_LATENCY_MAX_WRITE_SIZE:
                    sl_low_latency_vals = [True, False]

            marks = []
            if size == 10485760:
                # 10 MiB: large transfers consume far more memory/NIC resources
                # Run this size serially to avoid resource contention with
                # parallel workers
                marks = [pytest.mark.serial]

            for high_pps_val in high_pps_vals:
                for sl_low_latency_val in sl_low_latency_vals:

                    id = f"{rma_op}-{size}"

                    if high_pps_val is not None:
                        id += f"-high_pps-{high_pps_val}"

                    if sl_low_latency_val is not None:
                        id += f"-sl_low_lat-{sl_low_latency_val}"

                    yield pytest.param(size, rma_op, high_pps_val,
                                       sl_low_latency_val, marks=marks, id=id)


# Mirrors EFA_MR_ABORT_MAX_EPS in fabtests/prov/efa/src/mr_abort.c. The
# fabtest rejects endpoint counts above this
MR_ABORT_MAX_EPS = 512

MR_ABORT_EPS_PER_DOMAIN = 32

EpConfig = namedtuple("EpConfig",
                      ["initiator_ep_count", "target_ep_count", "eps_per_domain"])

# Ids of the endpoint/domain layouts every mr_abort test runs against. The ids
# are hardware-independent because the number of EFA domains cannot be queried
# at collection time.
# Keeping the id set fixed also keeps -k expressions and exclusion lists
# working across instance types.
EP_CONFIG_IDS = [
    "single_ep",
    "symm_single_domain",
    "incast_single_domain",
    "symm_all_domains_single_ep",
    "symm-all_domains_multi_ep",
    "incast_all_domains_single_ep",
    "incast_all_domains_multi_ep",
    "incast_all_domains_multi_ep_single_ep_target",
]


def spread_over_all_domains(num_domains):
    """
    Return the (n_eps, eps_per_domain) that puts endpoints on every one of
    num_domains domains, reduced to respect MR_ABORT_MAX_EPS.

    The fabtest derives the number of domains a side uses from
    ceil(n_eps / eps_per_domain), so capping n_eps alone would quietly narrow
    the spread to the first few domains. Shrinking eps_per_domain instead
    keeps every domain in play, which is what these layouts exist to exercise.
    """
    eps_per_domain = min(MR_ABORT_EPS_PER_DOMAIN,
                         max(1, MR_ABORT_MAX_EPS // num_domains))

    return min(num_domains * eps_per_domain, MR_ABORT_MAX_EPS), eps_per_domain


def resolve_ep_configs(num_domains):
    """
    Return an id -> EpConfig mapping covering EP_CONFIG_IDS, for a host pair
    offering num_domains EFA domains.

    Note that --eps-per-domain is one value applied to whichever side the
    process is on, so an incast layout gives the initiator and the target
    different domain counts from the same flag: the initiator's larger endpoint
    count spans more domains than the target's.
    """
    # Endpoint count and per-domain fill for a side that covers every domain.
    all_dom_eps, all_dom_fill = spread_over_all_domains(num_domains)

    # A side that names one endpoint per domain is bounded by the ep limit too.
    one_per_dom_eps = min(num_domains, MR_ABORT_MAX_EPS)

    multi = MR_ABORT_EPS_PER_DOMAIN

    configs = {
        # Single endpoint, symmetric
        "single_ep": EpConfig(1, 1, 1),

        # Multiple endpoints, symmetric on both sides, single domain
        "symm_single_domain": EpConfig(multi, multi, multi),

        # Multiple endpoints, incast, single domain
        "incast_single_domain": EpConfig(multi, 1, multi),

        # Symmetric, every domain, one endpoint per domain
        "symm_all_domains_single_ep": EpConfig(one_per_dom_eps, one_per_dom_eps, 1),

        # Symmetric, every domain, multiple endpoints per domain
        "symm-all_domains_multi_ep": EpConfig(all_dom_eps, all_dom_eps, all_dom_fill),

        # Incast, every domain, one initiator endpoint per domain
        "incast_all_domains_single_ep": EpConfig(one_per_dom_eps, 1, 1),

        # Incast, initiator over every domain with multiple endpoints per
        # domain, target holding a domain's worth of endpoints
        "incast_all_domains_multi_ep": EpConfig(all_dom_eps, all_dom_fill, all_dom_fill),

        # Incast, initiator over every domain with multiple endpoints per
        # domain, single target endpoint
        "incast_all_domains_multi_ep_single_ep_target": EpConfig(all_dom_eps, 1, all_dom_fill),
    }

    assert set(configs) == set(EP_CONFIG_IDS)

    return configs


def ep_config_args(ep_config):
    """The fi_mr_abort options selecting an endpoint/domain layout."""
    return (f" --num-initiator-eps {ep_config.initiator_ep_count}"
            f" --num-target-eps {ep_config.target_ep_count}"
            f" --eps-per-domain {ep_config.eps_per_domain}")


@pytest.fixture(scope="function", params=EP_CONFIG_IDS)
def ep_config(request, num_domains):
    """
    Resolve one EP_CONFIG_IDS entry to the endpoint counts to run with.

    Several layouts describe the same counts on a host with few domains -- on a
    single-domain instance every "alldom" layout collapses onto its
    single-domain counterpart. Rather than drop those ids from the id set,
    which would make the ids instance-specific, run the first id that produces
    a given layout and skip the rest.
    """
    configs = resolve_ep_configs(num_domains)
    config = configs[request.param]

    first_id = next(id for id, other in configs.items() if other == config)
    if first_id != request.param:
        pytest.skip(f"{request.param} is {config} with {num_domains} "
                    f"domain(s), same as {first_id}")

    return config


# --- Test: abort (RMA) ---
@pytest.mark.functional
@pytest.mark.fabric(params=["efa-direct"])  # TODO add test for efa fabric
@pytest.mark.parametrize("cancel_order", ["reverse", "random"])
@pytest.mark.parametrize("close_side", ["initiator", "target"])
@pytest.mark.parametrize("ops_per_mr", [1, 4])
@pytest.mark.parametrize("message_size, rma_op, high_pps, sl_low_latency",
                         list(combined_msg_size_params()))
def test_mr_abort(cmdline_args, rma_fabric, rma_op, cancel_order, close_side, ops_per_mr,
                  high_pps, sl_low_latency, message_size, memory_type_symm, ep_config):
    if rma_fabric == "efa" and cmdline_args.server_id == cmdline_args.client_id:
        pytest.skip("fi_mr_abort not supported with efa with SHM")

    if message_size == 10485760 and (cancel_order == 'random' or ops_per_mr == 4):
        pytest.skip("fi_mr_abort 10MB test only runs with reverse cancel order on 1 operation per MR to save run time")

    command = (f"fi_mr_abort -T abort -o {rma_op} -C {cancel_order}"
               f" -R {close_side} -N {ops_per_mr} -W {MR_ABORT_NUM_MRS}"
               f" -S {message_size}" + ep_config_args(ep_config))

    if high_pps:
        assert(rma_op != "read")
        command += " --high-pps"

    if sl_low_latency:
        assert(rma_op != "read")
        assert(message_size < 128)
        command += " --sl-low-latency"

    test = ClientServerTest(cmdline_args, command, timeout=300, fabric=rma_fabric, memory_type=memory_type_symm)
    test.run()


# --- Test: partial (2 MRs on same buffer) ---
@pytest.mark.functional
@pytest.mark.fabric(params=["efa-direct"]) # TODO add test for efa fabric
@pytest.mark.parametrize("message_size, rma_op, high_pps, sl_low_latency",
                         list(combined_msg_size_params()))
def test_mr_abort_partial(cmdline_args, rma_fabric, rma_op, high_pps, sl_low_latency,
                          message_size, memory_type_symm, ep_config):
    if rma_fabric == "efa" and cmdline_args.server_id == cmdline_args.client_id:
        pytest.skip("fi_mr_abort not supported with efa with SHM")

    command = (f"fi_mr_abort -T partial -o {rma_op} -S {message_size}"
               + ep_config_args(ep_config))

    if high_pps:
        assert(rma_op != "read")
        command += " --high-pps"

    if sl_low_latency:
        assert(rma_op != "read")
        assert(message_size < 128)
        command += " --sl-low-latency"

    test = ClientServerTest(cmdline_args, command, timeout=300, fabric=rma_fabric, memory_type=memory_type_symm)
    test.run()


def determine_settings_for_proto(protocol, memory_type_symm, fabric):
    """
    Return the (env, message_size) needed to exercise a specific EFA RDM
    two-sided wire protocol with fi_mr_abort.

    Deterministic protocol pinning via env vars only applies to the
    host-memory RDM path (fabric == "efa" and host_to_host). For
    efa-direct there is no RDM protocol selection at all. In those cases
    we fall back to the previous behavior which was to pick a small, medium
    and large message size.

    To pin a single protocol on host+efa we disable the competing paths via
    env vars rather than relying on default thresholds:

      - FI_EFA_USE_DEVICE_RDMA=0 removes the read-base branch entirely, so a
        size-based choice among EAGER/MEDIUM/LONGCTS is unambiguous.
      - FI_EFA_INTER_MAX_MEDIUM_MESSAGE_SIZE bounds MEDIUM.
      - FI_EFA_INTER_MIN_READ_MESSAGE_SIZE bounds the read-base entry.
      - FI_EFA_RUNT_SIZE selects RUNTREAD (>0) vs LONGREAD (0), and its
        magnitude vs message_size selects runt-only (NOREAD) vs runt+read.

    :param protocol: one of EAGER, MEDIUM, LONGCTS, RUNTREAD-LONGREAD,
                     RUNTREAD-NOREAD
    :param memory_type_symm: symmetric memory type, e.g. "host_to_host",
                     "cuda_to_cuda"
    :param fabric: "efa" or "efa-direct"
    :return: (env_str, message_size) where env_str is a space-separated
             "VAR=val ..." string passed to both peers via additional_env
             (empty for the fallback path), and message_size is the -S value.
    """
    # Host-memory defaults (efa.h): eager_max ~= MTU - headers (~8 KB),
    # max_medium_msg_size = 65536, min_read_msg_size = 1048576.
    EAGER_SIZE = 4096            # < eager_max -> EAGER
    MEDIUM_SIZE = 65536          # 8 MTU-sized packets, == default medium_max -> MEDIUM
    LARGE_SIZE = 1048576         # 1 MiB, > medium_max -> LONGCTS / read-base
    RUNT_ONLY_SIZE = 131072      # <= runt budget so no trailing READ is posted

    # Representative size per protocol, used both for the host+efa pinned
    # path and the fallback path.
    proto_size = {
        "EAGER": EAGER_SIZE,
        "MEDIUM": MEDIUM_SIZE,
        "LONGCTS": LARGE_SIZE,
        "LONGREAD": LARGE_SIZE,
        "RUNTREAD-LONGREAD": LARGE_SIZE,
        "RUNTREAD-NOREAD": RUNT_ONLY_SIZE,
    }
    if protocol not in proto_size:
        raise ValueError(f"unknown protocol: {protocol}")

    # Fallback: HMEM or efa-direct. Protocol pinning via the host RDM
    # thresholds does not apply; return a representative size and no env.
    if fabric != "efa" or memory_type_symm != "host_to_host":
        return "", proto_size[protocol]

    # Host + efa: pin the protocol deterministically via env-var thresholds.
    # The selected protocol can be confirmed manually by running with
    # FI_LOG_LEVEL=debug and looking for the provider's
    # "efa-rdm selecting transfer protocol ..." log line.
    if protocol == "EAGER":
        # 4 KB is below eager_max, and below the 1 MiB host min_read
        # threshold so read-base is never considered. No threshold override.
        proto_env = ""
    elif protocol == "MEDIUM":
        # 64 KB == the default host max_medium_msg_size (65536), which spans
        # ~8 MTU-sized REQ packets. It is above eager_max and below min_read
        # (1 MiB), and the selection test is total_len <= max_medium_msg_size,
        # so the defaults select MEDIUM. No threshold override.
        proto_env = ""
    elif protocol == "LONGCTS":
        # Disable read-base; size above medium_max forces LONGCTS.
        proto_env = ("FI_EFA_USE_DEVICE_RDMA=0 "
                     "FI_EFA_INTER_MAX_MEDIUM_MESSAGE_SIZE=65536")
    elif protocol == "LONGREAD":
        # Pure long read: read-base enabled (USE_DEVICE_RDMA=1) with the
        # runt budget set to 0, so no runt segments ride the RTM and the
        # entire payload is pulled by the receiver via RDMA READ. The
        # LONGREAD RTM is a pure read-request (no source-MR payload), so
        # closing the source MR cannot flush it before the receiver matches:
        # the recv is always matched and its tail READ then fails against
        # the invalidated source rkey, yielding exactly one FI_ECANCELED.
        # This is the only protocol owed a completion under -X (see
        # abort_owes_rx_completion). Run with -H (HOMOGENEOUS_PEERS) so the
        # sender does not stall on a handshake before selecting it.
        proto_env = ("FI_EFA_USE_DEVICE_RDMA=1 "
                     "FI_EFA_RUNT_SIZE=0 "
                     "FI_EFA_INTER_MIN_READ_MESSAGE_SIZE=65536")
    elif protocol == "RUNTREAD-LONGREAD":
        # Read-base enabled with a runt budget smaller than the message
        # (RUNT_SIZE=131072 < 1 MiB). Under the fi_mr_abort flood this
        # exercises BOTH read-base sub-protocols in one run: head-of-line
        # messages select RUNTREAD (runt segments + a trailing RDMA READ)
        # while the per-peer runt budget and num_read_msg_in_flight==0
        # gates pass; once the budget drains / reads are in flight,
        # selection degrades to LONGREAD. Combining them in a single stage
        # mirrors how a real read-base workload mixes the two.
        proto_env = ("FI_EFA_USE_DEVICE_RDMA=1 "
                     "FI_EFA_RUNT_SIZE=131072 "
                     "FI_EFA_INTER_MIN_READ_MESSAGE_SIZE=65536")
    elif protocol == "RUNTREAD-NOREAD":
        # RUNTREAD-NOREAD: runt-only runting read: the whole message fits in
        # the runt budget (total_len <= runt_size), so all data rides REQ
        # packets and no RDMA READ is posted. runt_size >= msg >= min_read.
        proto_env = ("FI_EFA_USE_DEVICE_RDMA=1 "
                     "FI_EFA_RUNT_SIZE=262144 "
                     "FI_EFA_INTER_MIN_READ_MESSAGE_SIZE=65536")
    else:
        raise ValueError(f"unknown protocol: {protocol}")

    return proto_env.strip(), proto_size[protocol]


def abort_owes_rx_completion(protocol):
    """
    Whether an aborted send/tagged message is owed a terminal recv
    completion on the target for this protocol (the -X flag of
    fi_mr_abort).

    Owed: LONGREAD only. The LONGREAD RTM is a pure read-request control
    packet -- it carries the read iov, no inline user data, and its send WR
    uses the internal TX pool MR rather than the user's source MR. So
    closing the source MR cannot flush the RTM: it is always delivered, the
    receiver always matches the recv and posts the tail RDMA READ, and that
    READ then fails against the invalidated source rkey -- driving the
    matched rxe to a clean FI_ECANCELED. Exactly one terminal completion is
    therefore guaranteed.

    Not owed: EAGER, MEDIUM, LONGCTS, RUNTREAD-LONGREAD, RUNTREAD-NOREAD.
    Every one of these carries source-MR user data in its RTM (EAGER/MEDIUM
    full or first segment, LONGCTS first segment, RUNTREAD runt segments),
    so the RTM itself can be flushed or gen-check cancelled before the
    receiver ever matches the recv. When that happens the receiver owes no
    completion, so the recv-completion count is indeterminate and -X (which
    blocks until reaped == required) would hang the target. These use the
    slack path: a stray FI_ECANCELED is tolerated but never required.
    """
    return protocol in ("LONGREAD",)


# --- Test: send and tagged ---
@pytest.mark.functional
@pytest.mark.fabric(params=["efa-direct"]) # TODO add test for efa fabric
@pytest.mark.parametrize("cancel_order", ["reverse", "random"])
@pytest.mark.parametrize("close_side", ["initiator"]) # TODO add target
@pytest.mark.parametrize("ops_per_mr", [1, 4])
@pytest.mark.parametrize("tagged", [True, False])
@pytest.mark.parametrize("protocol", ["EAGER", "MEDIUM", "LONGCTS", "LONGREAD", "RUNTREAD-LONGREAD", "RUNTREAD-NOREAD"])
def test_mr_abort_send(cmdline_args, fabric, cancel_order, close_side,
                       ops_per_mr, tagged, protocol, memory_type_symm,
                       ep_config):

    if fabric == "efa" and cmdline_args.server_id == cmdline_args.client_id:
        pytest.skip("fi_mr_abort not supported with efa with SHM")

    if close_side == "target":
        pytest.skip("efa does not currently support canceling posted RX buffers")

    send_op = "send"
    if tagged:
        if fabric == "efa-direct":
            pytest.skip("efa-direct does not support tagged sends")
        send_op = "tagged"

    env, message_size = determine_settings_for_proto(protocol, memory_type_symm, fabric)

    # efa-direct max message size is 8k
    if fabric == "efa-direct" and message_size > 8192:
        pytest.skip("efa-direct max send size is 8KB")

    owe_flag = " -X" if abort_owes_rx_completion(protocol) else ""
    # LONGREAD is an extra-feature (read-based) protocol. Pass -H to set
    # HOMOGENEOUS_PEERS on the endpoint, which makes it ignore the handshake
    # requirement before selecting a read-based protocol. Without it the
    # first sends are queued pending the peer handshake and the abort race is
    # nondeterministic; skipping the handshake pins LONGREAD from the first
    # send so the target reliably owes -- and can enforce via -X -- exactly
    # one completion per send.
    homogeneous_flag = " -H" if protocol == "LONGREAD" else ""
    command = (f"fi_mr_abort -T {send_op} -C {cancel_order}"
               f" -R {close_side} -N {ops_per_mr} -W {MR_ABORT_NUM_MRS}"
               f" -S {message_size}{owe_flag}{homogeneous_flag}  -A ep_first"
               + ep_config_args(ep_config))
    test = ClientServerTest(cmdline_args, command, timeout=360, fabric=fabric,
                            memory_type=memory_type_symm, additional_env=env)
    test.run()
