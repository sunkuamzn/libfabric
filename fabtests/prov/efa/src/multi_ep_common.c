/* SPDX-License-Identifier: BSD-2-Clause OR GPL-2.0-only */
/* SPDX-FileCopyrightText: Copyright Amazon.com, Inc. or its affiliates. All
 * rights reserved. */

#include "multi_ep_common.h"

int get_one_comp(struct fid_cq *cq)
{
	struct fi_cq_err_entry comp;
	struct fi_cq_err_entry cq_err;

	memset(&cq_err, 0, sizeof(cq_err));
	int ret;

	do {
		ret = fi_cq_read(cq, &comp, 1);
		if (ret > 0)
			break;

		if (ret < 0) {
			if (ret != -FI_EAGAIN) {
				printf("fi_cq_read returns error %d\n", ret);
				(void) fi_cq_readerr(cq, &cq_err, 0);
				return ret;
			}
		}
	} while (1);

	return FI_SUCCESS;
}
