/*
 * Copyright (c) 2024 Intel Corporation. All rights reserved.
 *
 * This software is available to you under the BSD license below:
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>

#include <rdma/fabric.h>
#include <rdma/fi_errno.h>
#include <rdma/fi_endpoint.h>

#include "shared.h"
#include "multi_ep_common.h"

static struct fid_ep **server_eps;
static char **send_bufs, **recv_bufs;
static struct fid_mr **send_mrs, **recv_mrs;
static void **send_descs, **recv_descs;
static struct fi_context2 *recv_ctx, *send_ctx;
static struct fid_cq **server_txcqs, **server_rxcqs;
static struct fid_av **server_avs;
static fi_addr_t *remote_fiaddr;
static int num_server_eps = 3;
static bool unexpected_path = false;

static void free_res(void)
{
	int i;

	for (i = 0; i < num_server_eps; i++) {
		FT_CLOSE_FID(send_mrs[i]);
		FT_CLOSE_FID(recv_mrs[i]);

		if (send_bufs[i])
			(void) ft_hmem_free(opts.iface, (void *) send_bufs[i]);
		if (recv_bufs[i])
			(void) ft_hmem_free(opts.iface, (void *) recv_bufs[i]);
	}

	free(send_bufs);
	free(recv_bufs);
	free(send_mrs);
	free(recv_mrs);
	free(send_descs);
	free(recv_descs);
	free(send_ctx);
	free(recv_ctx);
	free(remote_fiaddr);
}

static void free_server_res(void)
{
	int i;

	free_res();

	for (i = 0; i < num_server_eps; i++) {
		FT_CLOSE_FID(server_eps[i]);
		FT_CLOSE_FID(server_txcqs[i]);
		FT_CLOSE_FID(server_rxcqs[i]);
		FT_CLOSE_FID(server_avs[i]);
	}

	free(server_txcqs);
	free(server_rxcqs);
	free(server_eps);
	free(server_avs);
}

static int alloc_bufs(void)
{
	int i, ret;
	size_t alloc_size;

	remote_fiaddr = calloc(num_server_eps, sizeof(*remote_fiaddr));
	send_ctx = calloc(num_server_eps, sizeof(*send_ctx));
	recv_ctx = calloc(num_server_eps, sizeof(*recv_ctx));

	send_bufs = calloc(num_server_eps, sizeof(*send_bufs));
	recv_bufs = calloc(num_server_eps, sizeof(*recv_bufs));

	if (!send_bufs || !recv_bufs || !remote_fiaddr || !send_ctx ||
	    !recv_ctx)
		return -FI_ENOMEM;

	alloc_size = opts.transfer_size < FT_MAX_CTRL_MSG ? FT_MAX_CTRL_MSG : opts.transfer_size;
	for (i = 0; i < num_server_eps; i++) {
		ret = ft_hmem_alloc(opts.iface, opts.device,
				    (void **) &send_bufs[i],
				    alloc_size);
		if (ret)
			return ret;

		ret = ft_hmem_alloc(opts.iface, opts.device,
				    (void **) &recv_bufs[i],
				    alloc_size);
		if (ret)
			return ret;

		ret = ft_fill_buf(send_bufs[i], opts.transfer_size);
		if (ret)
			return ret;
	}

	return 0;
}

static int alloc_server_res(void)
{
	int ret;

	server_eps = calloc(num_server_eps, sizeof(*server_eps));
	server_txcqs = calloc(num_server_eps, sizeof(*server_txcqs));
	server_rxcqs = calloc(num_server_eps, sizeof(*server_rxcqs));
	server_avs = calloc(num_server_eps, sizeof(*server_avs));

	if (!server_eps || !server_txcqs || !server_rxcqs || !server_avs)
		return -FI_ENOMEM;

	ret = alloc_bufs();
	if (ret)
		return ret;

	return 0;
}

static int setup_server_ep(int idx)
{
	int ret;

	ret = fi_endpoint(domain, fi, &server_eps[idx], NULL);
	if (ret) {
		FT_PRINTERR("fi_endpoint", ret);
		return ret;
	}

	ret = ft_alloc_ep_res(fi, &server_txcqs[idx], &server_rxcqs[idx], 
			      NULL, NULL, NULL, &server_avs[idx]);
	if (ret)
		return ret;

	ret = ft_enable_ep(server_eps[idx], eq, server_avs[idx], 
			   server_txcqs[idx], server_rxcqs[idx], 
			   NULL, NULL, NULL);
	if (ret)
		return ret;

	return 0;
}

static int reg_mrs(void)
{
	int i, ret;

	send_mrs = calloc(num_server_eps, sizeof(*send_mrs));
	recv_mrs = calloc(num_server_eps, sizeof(*recv_mrs));
	send_descs = calloc(num_server_eps, sizeof(*send_descs));
	recv_descs = calloc(num_server_eps, sizeof(*recv_descs));

	if (!send_mrs || !recv_mrs || !send_descs ||
	    !recv_descs)
		return -FI_ENOMEM;

	for (i = 0; i < num_server_eps; i++) {
		ret = ft_reg_mr(fi, send_bufs[i], opts.transfer_size,
				ft_info_to_mr_access(fi),
				(FT_MR_KEY + 1) * (i + 1), opts.iface,
				opts.device, &send_mrs[i], 
				&send_descs[i]);
		if (ret)
			return ret;

		ret = ft_reg_mr(fi, recv_bufs[i], opts.transfer_size,
				ft_info_to_mr_access(fi),
				(FT_MR_KEY + 2) * (i + 2), opts.iface,
				opts.device, &recv_mrs[i], 
				&recv_descs[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int server_post_send(int idx)
{
	return ft_post_tx_buf(server_eps[idx], remote_fiaddr[idx],
			      opts.transfer_size, NO_CQ_DATA, &send_ctx[idx],
			      send_bufs[idx], send_descs[idx], ft_tag);
}

static int client_post_recv_unspec(void)
{
	int i, ret = 0;

	for (i = 0; i < num_server_eps; i++) {
		ret = ft_post_rx_buf(ep, FI_ADDR_UNSPEC, opts.transfer_size,
				     &rx_ctx, recv_bufs[i], recv_descs[i],
				     ft_tag);
		if (ret) {
			FT_PRINTERR("client_post_recv_unspec", ret);
			return ret;
		}
	}

	return ret;
}

static int client_post_recv_directed(void)
{
	int i, ret = 0;

	for (i = 0; i < num_server_eps; i++) {
		ret = ft_post_rx_buf(ep, remote_fiaddr[i], opts.transfer_size,
				     &rx_ctx, recv_bufs[i], recv_descs[i],
				     ft_tag);
		if (ret) {
			FT_PRINTERR("client_post_recv_directed", ret);
			return ret;
		}
	}

	return ret;
}

static int run_test(void)
{
	int i, ret;

	printf("Testing message processing before AV insertion\n");

	ret = ft_init_fabric();
	if (ret)
		return ret;

	if (opts.dst_addr) {
		/* Client side - single endpoint */
		printf("Client: Step 1 - Post receive buffers with FI_ADDR_UNSPEC\n");
		
		ret = alloc_bufs();
		if (ret)
			goto cleanup_client;

		ret = reg_mrs();
		if (ret)
			goto cleanup_client;

		if (!unexpected_path) {
			ret = client_post_recv_unspec();
			if (ret)
				goto cleanup_client;
		}

		ft_sync_oob(); // Initial sync

		/* Send client address to server (no AV insertion yet) */
		for (i = 0; i < num_server_eps; i++) {
			ret = ft_send_addr_oob(ep);
			if (ret) {
				FT_PRINTERR("ft_server_insert_addr_oob", ret);
				goto cleanup_client;
			}
		}

		ft_sync_oob(); // Sync after server has sent all messages for step 1

		if (unexpected_path) {
			ret = client_post_recv_unspec();
			if (ret)
				goto cleanup_client;
		}

		printf("Client: Waiting for messages from %d server endpoints\n", num_server_eps);
		
		/* Wait for all receive completions */
		for (i = 0; i < num_server_eps; i++) {
			ret = get_one_comp(rxcq);
			if (ret) {
				FT_PRINTERR("get_client_comp", ret);
				return ret;
			}
		}

		printf("Client: Step 2 - Insert server addresses into AV\n");
		ft_sync_oob(); // Sync for start of AV insertion on client
		
		/* Receive server addresses and insert into AV */
		for (i = 0; i < num_server_eps; i++) {
			ret = ft_recv_addr_oob(av, &remote_fiaddr[i]);
			if (ret)
				goto cleanup_client;
		}

		printf("Client: Step 3 - Repeat with FI_ADDR_UNSPEC after AV insertion\n");
		/* Post receive buffers again with FI_ADDR_UNSPEC */
		if (!unexpected_path) {
			ret = client_post_recv_unspec();
			if (ret)
				goto cleanup_client;
		}

		ft_sync_oob(); // Sync for start of sends on server for step 3

		ft_sync_oob(); // Sync for end of sends on server for step 3

		if (unexpected_path) {
			ret = client_post_recv_unspec();
			if (ret)
				goto cleanup_client;
		}

		/* Wait for all receive completions */
		for (i = 0; i < num_server_eps; i++) {
			ret = get_one_comp(rxcq);
			if (ret)
				goto cleanup_client;
		}

		printf("Client: Step 4 - Directed receives to specific endpoints\n");
		/* Post directed receive buffers */
		if (!unexpected_path) {
			ret = client_post_recv_directed();
			if (ret)
				goto cleanup_client;
		}

		ft_sync_oob(); // Sync for start of sends on server for step 4

		ft_sync_oob(); // Sync for end of sends on server for step 4

		if (unexpected_path) {
			ret = client_post_recv_directed();
			if (ret)
				goto cleanup_client;
		}

		/* Wait for all receive completions */
		for (i = 0; i < num_server_eps; i++) {
			ret = get_one_comp(rxcq);
			if (ret)
				goto cleanup_client;
		}


cleanup_client:
		free_res();
	} else {
		/* Server side - multiple endpoints */
		printf("Server: Creating %d endpoints\n", num_server_eps);
		
		/* Create server endpoints */
		ret = alloc_server_res();
		if (ret)
			return ret;

		for (i = 0; i < num_server_eps; i++) {
			ret = setup_server_ep(i);
			if (ret)
				goto cleanup_server;
		}

		ret = reg_mrs();
		if (ret)
			goto cleanup_server;

		ft_sync_oob(); // Initial sync

		/* Initialize AV for each endpoint using OOB */
		for (i = 0; i < num_server_eps; i++) {
			ret = ft_recv_addr_oob(server_avs[i], &remote_fiaddr[i]);
			if (ret) {
				FT_PRINTERR("ft_server_insert_addr_oob", ret);
				goto cleanup_server;
			}
		}

		printf("Server: Step 1 - Send messages from all endpoints\n");
		/* Send from all endpoints */
		for (i = 0; i < num_server_eps; i++) {
			ret = server_post_send(i);
			if (ret) {
				FT_PRINTERR("server_post_send", ret);
				goto cleanup_server;
			}
		}

		/* Wait for all send completions */
		for (i = 0; i < num_server_eps; i++) {
			ret = get_one_comp(server_txcqs[i]);
			if (ret) {
				FT_PRINTERR("get_server_comp", ret);
				goto cleanup_server;
			}
		}

		ft_sync_oob(); // Sync after server has sent all messages for step 1

		printf("Server: Step 2 - Send addresses to client\n");
		
		ft_sync_oob(); // Sync for start of AV insertion on client

		/* Send server addresses */
		for (i = 0; i < num_server_eps; i++) {
			ret = ft_send_addr_oob(server_eps[i]);
			if (ret) {
				FT_PRINTERR("ft_server_insert_addr_oob", ret);
				goto cleanup_server;
			}
		}

		ft_sync_oob(); // Sync for start of sends on server for step 3

		printf("Server: Step 3 - Send messages again after client AV insertion\n");
		/* Send from all endpoints again */
		for (i = 0; i < num_server_eps; i++) {
			ret = server_post_send(i);
			if (ret) {
				FT_PRINTERR("server_post_send", ret);
				goto cleanup_server;
			}
		}

		/* Wait for all send completions */
		for (i = 0; i < num_server_eps; i++) {
			ret = get_one_comp(server_txcqs[i]);
			if (ret) {
				FT_PRINTERR("get_server_comp", ret);
				goto cleanup_server;
			}
		}

		ft_sync_oob(); // Sync for end of sends on server for step 3

		printf("Server: Step 4 - Send messages for directed receives\n");
		
		ft_sync_oob(); // Sync for start of sends on server for step 4

		/* Send from all endpoints for directed receives */
		for (i = 0; i < num_server_eps; i++) {
			ret = server_post_send(i);
			if (ret) {
				FT_PRINTERR("server_post_send", ret);
				goto cleanup_server;
			}
		}

		/* Wait for all send completions */
		for (i = 0; i < num_server_eps; i++) {
			ret = get_one_comp(server_txcqs[i]);
			if (ret) {
				FT_PRINTERR("get_server_comp", ret);
				goto cleanup_server;
			}
		}

		ft_sync_oob(); // Sync for end of sends on server for step 4

cleanup_server:
		free_server_res();
	}

	printf("Test completed successfully\n");
	return ret;
}

int main(int argc, char **argv)
{
	int op, ret;

	opts = INIT_OPTS;
	opts.transfer_size = 64;
	opts.options |= FT_OPT_OOB_ADDR_EXCH;

	hints = fi_allocinfo();
	if (!hints)
		return EXIT_FAILURE;

	while ((op = getopt(argc, argv, "c:Xh" ADDR_OPTS INFO_OPTS CS_OPTS)) != -1) {
		switch (op) {
		default:
			ft_parse_addr_opts(op, optarg, &opts);
			ft_parseinfo(op, optarg, hints, &opts);
			ft_parsecsopts(op, optarg, &opts);
			break;
		case 'c':
			num_server_eps = atoi(optarg);
			break;
		case 'X':
			unexpected_path = true;
			break;
		case '?':
		case 'h':
			ft_usage(argv[0], "AV message order test");
			FT_PRINT_OPTS_USAGE("-c <int>", 
				"number of server endpoints (default 3)");
			return EXIT_FAILURE;
		}
	}

	if (optind < argc)
		opts.dst_addr = argv[optind];

	hints->caps = FI_MSG;
	hints->mode = FI_CONTEXT | FI_CONTEXT2;
	hints->ep_attr->type = FI_EP_RDM;
	hints->domain_attr->mr_mode = opts.mr_mode;
	hints->addr_format = opts.address_format;

	ret = run_test();

	ft_free_res();
	return ft_exit_code(ret);
}
