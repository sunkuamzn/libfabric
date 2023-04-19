/*
 * Copyright (c) Amazon.com, Inc. or its affiliates. All rights reserved.
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

#include "sm2_coordination.h"
#include "sm2.h"
#include "sm2_atom.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define NEXT_MULTIPLE_OF(x, mod) x % mod ? ((x / mod) + 1) * mod : x
#define ZOMBIE_ALLOCATION_NAME	 "ZOMBIE"
#define SM2_COORDINATION_DIR	 "/dev/shm"
#define SM2_COORDINATION_FILE	 "/dev/shm/fi_sm2_mmaps"
#define SM2_STARTUP_MAX_TRIES	 1000
#define SM2_MAX_UNIVERSE_SIZE	 4096

static void sm2_file_attempt_shrink(struct sm2_mmap *map);
int sm2_entry_lookup(const char *name, struct sm2_mmap *map);
void *sm2_file_extend_for_region(struct sm2_mmap *map, int last_valid_entry);

static inline bool
sm2_mapping_long_enough_check(struct sm2_mmap *map, int item)
{
	ptrdiff_t entry_offset =
		(char *) sm2_mmap_ep_region(map, item + 1) - map->base;
	return entry_offset <= map->size;
}

/* @return True if pid is still alive. */
static inline bool
pid_lives(int pid)
{
	int err = kill(pid, 0);
	return err == 0;
}

/**
 * @brief take an open file, and mmap its contents
 *
 * @param[in] fd
 * @param[out] map
 */
void *
sm2_mmap_map(int fd, struct sm2_mmap *map)
{
	struct stat st;

	if (OFI_UNLIKELY(fstat(fd, &st))) {
		FI_WARN(&sm2_prov, FI_LOG_EP_CTRL, "Failed fstat");
		goto out;
	}

	map->base =
		mmap(0, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (OFI_UNLIKELY(map->base == MAP_FAILED)) {
		FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
			"Failed mmap, st.st_size=%ld\n", st.st_size);
		goto out;
	}
	map->fd = fd;
	map->size = st.st_size;
	return map->base;
out:
	map->base = NULL;
	return NULL;
}

/*
 * Take a sm2_mmap, and re-map if necessary, ensuring size of at_least
 *
 * If the size of the current map or the size of the file are not sufficient
 * to address "at_least" bytes, then the file will be truncated() (extended)
 * to the required size and the memory munmap()ed and re mmap()'ed
 *
 */
void *
sm2_mmap_remap(struct sm2_mmap *map, size_t at_least)
{
	struct stat st;

	/* return quickly if no need to check the file. */
	assert(at_least > 0);
	if (map->size >= at_least)
		return map->base;

	if (OFI_UNLIKELY(fstat(map->fd, &st))) {
		FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
			"Failed fstat of sm2_mmaps file: %s, at_least: %zu\n",
			strerror(errno), at_least);
		return map->base;
	}

	if (st.st_size < at_least) {
		/* we need to grow the file. */
		if (OFI_UNLIKELY(ftruncate(map->fd, at_least))) {
			FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
				"Failed ftruncate of sm2_mmaps file: %s, "
				"at_least: %zu\n",
				strerror(errno), at_least);
			return map->base;
		}
	} else if (st.st_size >= map->size) {
		/* file has been extended by another process since
		   last we checked.  Re-map the entire file. */
		at_least = st.st_size;
	} else {
		/* This case currently never happens but is here for completness
		 */
		FI_WARN(&sm2_prov, FI_LOG_AV,
			"Shm file has shrunk, re-mapping!\n");
		at_least = st.st_size;
	}

	if (map->size != at_least) {
		/* now un-map and re-map the file */
		if (OFI_UNLIKELY(munmap(map->base, map->size))) {
			FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
				"Failed unmap of sm2_mmaps file: %s, at_least: "
				"%zu\n",
				strerror(errno), at_least);
			return map->base;
		}

		map->base = mmap(0, at_least, PROT_READ | PROT_WRITE,
				 MAP_SHARED, map->fd, 0);
		if (OFI_UNLIKELY(map->base == MAP_FAILED)) {
			FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
				"Failed to remap sm2_maps file when increasing "
				"size to st.st_size=%ld, at_least, %zu, error: "
				"%s\n",
				st.st_size, at_least, strerror(errno));
			abort();
		}
		map->size = at_least;
	}

	return map->base;
}

ssize_t
sm2_mmap_cleanup(struct sm2_mmap *map)
{
	int err1, err2;
	bool first_failed = false;

	err1 = close(map->fd);
	if (err1) {
		FI_WARN(&sm2_prov, FI_LOG_AV,
			"Failed to close fd %d with error code %d", map->fd,
			err1);
		first_failed = true;
	}

	err2 = munmap(map->base, map->size);
	if (err2) {
		FI_WARN(&sm2_prov, FI_LOG_AV,
			"Failed unmap shared memory region with error code %d",
			err2);
	}

	return first_failed ? err1 : err2;
}

enum sm2_file_lock_status {
	SM2_ACQUIRED_LOCK_ON_NEW_FILE = 0,
	SM2_ACQUIRED_LOCK_ON_EXISTING_GLOBAL_FILE,
	SM2_FAILED_TO_AQUIRE_LOCK
};

ssize_t
sm2_file_open_or_create(struct sm2_mmap *map_shared)
{
	pthread_mutexattr_t att;
	struct sm2_mmap map_ours;
	char template[SM2_NAME_MAX];
	struct sm2_coord_file_header *header, *tmp_header;
	struct sm2_ep_allocation_entry *entries;
	int fd, common_fd, err, tries, pid, item;
	enum sm2_file_lock_status lock_status;

	pid = getpid();
	sprintf(template, "%s/fi_sm2_pid%d_XXXXXX", SM2_COORDINATION_DIR, pid);

	/* Assume we are the first.
	   Open a tmpfile file in the shm directory */
	fd = mkostemp(template, O_RDWR);
	assert(fd > 0);
	err = ftruncate(fd, sizeof(*header));
	if (OFI_UNLIKELY(err))
		return -FI_ENOMEM;

	/* mmap the file */
	header = sm2_mmap_map(fd, &map_ours);

	pthread_mutexattr_init(&att);
	pthread_mutexattr_setpshared(&att, PTHREAD_PROCESS_SHARED);
	pthread_mutex_init(&header->write_lock, &att);
	/* since this is currently our own private mutex,
	   this lock is uncontested */
	pthread_mutex_lock(&header->write_lock);

	ofi_atomic_initialize32(&header->pid_lock_hint, pid);
	header->file_version = SM2_VERSION;
	header->ep_region_size = sm2_calculate_size_offsets(NULL, NULL);
	header->ep_entries_max = SM2_MAX_UNIVERSE_SIZE;

	header->ep_enumerations_offset = sizeof(*header);
	header->ep_enumerations_offset = NEXT_MULTIPLE_OF(
		header->ep_enumerations_offset, SM2_MAX_UNIVERSE_SIZE);

	header->ep_regions_offset = header->ep_enumerations_offset +
				    (header->ep_entries_max * sizeof(*entries));

	header->ep_regions_offset = NEXT_MULTIPLE_OF(header->ep_regions_offset,
						     SM2_MAX_UNIVERSE_SIZE);

	/* allocate enough space in the file for all our allocations, but no
	   data exchange regions yet. */
	header = sm2_mmap_remap(&map_ours, header->ep_regions_offset);
	entries = sm2_mmap_entries(&map_ours);
	for (item = 0; item < header->ep_entries_max; item++) {
		entries[item].pid = 0;
	}

	/* Is this memory barrier required for correctness? */
	/* Make sure the header is written before we link the file (flush
	 * file)*/
	atomic_mb();

	lock_status = SM2_FAILED_TO_AQUIRE_LOCK; /* failed */
	tries = SM2_STARTUP_MAX_TRIES;
	do {
		/* create a hardlink to our file with the common name.
		   - on success: we hold the lock to a newly
				 created coordination file.
		   - on failure: the file already exists, we should try opening
		   it.*/
		err = link(template, SM2_COORDINATION_FILE);
		if (0 == err) {
			/* we linked the file we made, we already hold the lock
			 */
			lock_status = SM2_ACQUIRED_LOCK_ON_NEW_FILE;
			break;
		}
		common_fd = open(SM2_COORDINATION_FILE, O_RDWR);
		if (common_fd > 0) {
			int pid_holding = 0;
			/* we've opened some existing file */
			tmp_header = sm2_mmap_map(common_fd, map_shared);
			assert(map_shared->size >= sizeof(*tmp_header));
			err = pthread_mutex_trylock(&tmp_header->write_lock);
			if (err == 0) {
				/* lock acquired! */
				ofi_atomic_set32(&tmp_header->pid_lock_hint,
						 pid);

				lock_status =
					SM2_ACQUIRED_LOCK_ON_EXISTING_GLOBAL_FILE; /* shared */
				break;
			}
			pid_holding =
				ofi_atomic_get32(&tmp_header->pid_lock_hint);

			/* check if pid_holding is alive by issuing it a
			   NULL signal (0).  This will not interrupt
			   the process, but only succeeds if the pid is
			   still running. */
			if (!pid_lives(pid_holding)) {
				/* TODO: unlinking is pretty harsh, but might be
				 * the only good option.*/
				unlink(SM2_COORDINATION_FILE);
				FI_WARN(&sm2_prov, FI_LOG_AV,
					"Unlinked file %s because PID=%d held "
					"the lock, but it has died\n",
					SM2_COORDINATION_FILE, pid_holding);
			}
			sm2_mmap_cleanup(map_shared);
		}
		/* we could not acquire the lock, sleep and try again. */
		/* TODO Consider reducing the amount of time we sleep here */
		usleep(10000);
	} while (tries-- > 0);

	switch (lock_status) {
	case SM2_FAILED_TO_AQUIRE_LOCK:
		FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
			"Failed to acquire the lock to the coordination file. "
			"Aborting\n");
		abort();
	case SM2_ACQUIRED_LOCK_ON_EXISTING_GLOBAL_FILE:
		/* We acquired the lock on the global file.
		 * Attempt to shrink global file.
		 * Unmap the memory that we initialized */
		sm2_file_attempt_shrink(map_shared);
		sm2_mmap_cleanup(&map_ours);
		break;
	case SM2_ACQUIRED_LOCK_ON_NEW_FILE:
		/* We are using the memory we initialized.  Duplicate it and
		   leave the map open. */
		memcpy(map_shared, &map_ours, sizeof(map_ours));
		break;
	default:
		/* catch cosmic ray bit-flips */
		FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
			"Error during sm2_file_open_or_create, unknown "
			"lock state\n");
		abort();
	}

	unlink(template);
	sm2_file_unlock(map_shared);
	return 0;
}

/**
 * @brief insert the name into the ep_enumerations array.  Requires the lock.
 *
 * Note that because of speculative av_insert operations, we may need to
 * assign an index for an endpoint claimed by another peer.
 *
 * @param[in] name	The name/string of the endpoint address
 * @param[inout] map	sm2_mmap object: the global shared file.
 * @param[out] gid	The address we have assigned to the peer.
 * @param[in] self	When true, we will "own" the entry (entry.pid =
 *getpid()) When False, we set pid = -getpid(), allowing owner to claim later
 * @return 0 on success, some negative FI_ERROR on error
 */
ssize_t
sm2_entry_allocate(const char *name, struct sm2_mmap *map, int *gid, bool self)
{
	struct sm2_coord_file_header *header = (void *) map->base;
	struct sm2_ep_allocation_entry *entries;
	struct sm2_region *peer_region = NULL;
	int item;
	int pid = getpid();
	int peer_pid;

	assert(pid == ofi_atomic_get32(&header->pid_lock_hint));

	entries = sm2_mmap_entries(map);

retry_lookup:
	item = sm2_entry_lookup(name, map);
	if (item >= 0) {
		sm2_file_extend_for_region(map, item);
		entries = sm2_mmap_entries(map);
		header = (void *) map->base;

		/* Check if it is dirty */
		if (entries[item].pid && !pid_lives(abs(entries[item].pid))) {
			peer_region = sm2_mmap_ep_region(map, item);
			if (!smr_freestack_isfull(
				    sm2_free_stack(peer_region))) {
				/* Region did not shut down properly, but other
				 * processes might be using it, make it a zombie
				 * region - never use this region for as long as
				 * the file exists */
				FI_WARN(&sm2_prov, FI_LOG_AV,
					"Found region at AV[%d] that did not "
					"shut down correctly, marking it as a "
					"zombie never to be used again (until "
					"all active processes die, and file "
					"size is reset)!\n",
					item);
				strncpy(entries[item].ep_name,
					ZOMBIE_ALLOCATION_NAME, SM2_NAME_MAX);
				goto retry_lookup;
			}
		}

		if (!self) {
			if (!pid_lives(abs(entries[item].pid))) {
				entries[item].pid = 0;
			}
			/* Someone else allocated the entry for us */
			goto found;
		}

		if (entries[item].pid <= 0) {
			if (!pid_lives(abs(entries[item].pid))) {
				FI_WARN(&sm2_prov, FI_LOG_AV,
					"During sm2 allocation of space for "
					"endpoint named %s pid %d "
					"pre-allocated space at AV[%d] and "
					"then died!\n",
					name, -entries[item].pid, item);
			}
			goto found;
		}

		FI_WARN(&sm2_prov, FI_LOG_AV,
			"During sm2 allocation of space for endpoint named %s "
			"an existing conflicting address was found at AV[%d]\n",
			name, item);

		if (!pid_lives(entries[item].pid)) {
			FI_WARN(&sm2_prov, FI_LOG_AV,
				"The pid which allocated the conflicting AV is "
				"dead. Reclaiming as our own.\n");
			/* it is possible that EP's referencing this region are
			 * still alive... don't know how to check (they likely
			 * died if PID died) */
			goto found;
		} else {
			FI_WARN(&sm2_prov, FI_LOG_AV,
				"ERROR: The endpoint (pid: %d) with "
				"conflicting address %s is still alive.\n",
				entries[item].pid, name);
			return -FI_EADDRINUSE;
		}
	}

	/* fine, we could not find the entry, so now look for an empty slot */
	for (item = 0; item < header->ep_entries_max; item++) {
		peer_pid = entries[item].pid;
		if (peer_pid == 0)
			goto found;
		if (peer_pid < 0)
			/* A third peer might have entered this address into
			 * their AV, and there is no current way to check
			 * this... need to keep this entry in the file until we
			 * clean up */
			continue;

		if (!pid_lives(peer_pid)) {
			sm2_file_extend_for_region(map, item);
			entries = sm2_mmap_entries(map);
			header = (void *) map->base;
			struct sm2_region *peer_region =
				sm2_mmap_ep_region(map, item);

			if (entries[item].startup_ready &&
			    smr_freestack_isfull(sm2_free_stack(peer_region))) {
				/* we found a slot with a dead PID and
				 * the freestack is full */
				entries[item].pid = 0;
				goto found;
			}
		}
	}

	FI_WARN(&sm2_prov, FI_LOG_AV,
		"No available entries were found in the coordination file, all "
		"%d were used\n",
		header->ep_entries_max);
	return -FI_EAVAIL;

found:
	if (self) {
		entries[item].startup_ready = 0;
		atomic_wmb();
		entries[item].pid = pid;
	}

	if (!self && entries[item].pid == 0) {
		entries[item].startup_ready = 0;
		atomic_wmb();
		entries[item].pid = -pid;
	}

	FI_WARN(&sm2_prov, FI_LOG_AV, "Using sm2 region at AV[%d] for %s\n",
		item, name);
	strncpy(entries[item].ep_name, name, SM2_NAME_MAX);

	*gid = item;

	/* With the entry allocated, we now need to ensure it's mapped. */
	if (!sm2_mapping_long_enough_check(map, item)) {
		sm2_file_extend_for_region(map, item);
	}
	return 0;
}

/**
 * @brief look-up the name.
 *
 * @param[in] name
 * @param[in] map
 * @return the gid, or -1
 */
int
sm2_entry_lookup(const char *name, struct sm2_mmap *map)
{
	struct sm2_coord_file_header *header = (void *) map->base;
	struct sm2_ep_allocation_entry *entries;
	int item;

	entries = sm2_mmap_entries(map);

	for (item = 0; item < header->ep_entries_max; item++) {
		if (0 == strncmp(name, entries[item].ep_name, SM2_NAME_MAX)) {
			FI_DBG(&sm2_prov, FI_LOG_AV,
			       "Found existing %s in slot %d\n", name, item);
			return item;
		}
		if (entries[item].ep_name[0] != '\0') {
			FI_DBG(&sm2_prov, FI_LOG_EP_CTRL,
			       "Searching for %s. Not yet found in spot "
			       "AV[%d]=%s, PID=%d\n",
			       name, item, entries[item].ep_name,
			       entries[item].pid);
		} else if (!strcmp(entries[item].ep_name,
				   ZOMBIE_ALLOCATION_NAME)) {
			FI_DBG(&sm2_prov, FI_LOG_EP_CTRL,
			       "Found zombie in spot AV[%d], PID= %d\n", item,
			       entries[item].pid);
		}
	}
	return -1;
}

/**
 * @brief clear the pid for this entry.  must already hold lock.
 *
 * @param[in] name
 * @param[inout] map
 * @param[out] gid
 */
ssize_t
sm2_entry_free(struct sm2_mmap *map, int gid)
{
	struct sm2_ep_allocation_entry *entries;

	entries = sm2_mmap_entries(map);
	assert(entries[gid].pid == getpid());
	entries[gid].pid = 0;
	return 0;
}

ssize_t
sm2_file_lock(struct sm2_mmap *map)
{
	struct sm2_coord_file_header *header = (void *) map->base;
	int pid = getpid();

	pthread_mutex_lock(&header->write_lock);
	ofi_atomic_set32(&header->pid_lock_hint, pid);
	return 0;
}

ssize_t
sm2_file_unlock(struct sm2_mmap *map)
{
	struct sm2_coord_file_header *header = (void *) map->base;
	int pid = getpid();
	int hint_pid = ofi_atomic_get32(&header->pid_lock_hint);
	if (pid != hint_pid) {
		FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
			"We (%d) should hold the lock, but pid_lock_hint is "
			"%d!",
			pid, hint_pid);
	}
	pthread_mutex_unlock(&header->write_lock);

	/* use CAS to clear pid because it's possible another process has
	   already acquired the lock.  CAS will only clear it if we hold it. */
	ofi_atomic_cas_bool32(&header->pid_lock_hint, pid, 0);
	return 0;
}

/**
 * @brief ensure the mapping is large enough to address a particular region
 *
 * No lock is required.  This function may be used to grow the file and mapping
 * prior to initializing the memory, or it may be used to grow and re-map a
 * region that another peer already initialized.
 *
 * In either case, we don't need the lock, as we are only growing the file
 * and not modifying any allocations (ep_entries)
 *
 * @param[in,out] map			an sm2_mmap with fid, size and base
 * address
 * @param[in] last_valid_entry		The id of a region we need to be valid.
 * @return pointer to new map->base (typically ignored)
 */
void *
sm2_file_extend_for_region(struct sm2_mmap *map, int last_valid_entry)
{
	size_t new_size;
	assert(last_valid_entry < SM2_MAX_UNIVERSE_SIZE);
	new_size = (char *) sm2_mmap_ep_region(map, last_valid_entry + 1) -
		   map->base;
	return sm2_mmap_remap(map, new_size);
}

/*
 * TODO combine this logic with the remap function while keeping remaps()'s
 * short circuit logic */
static void *
sm2_mmap_shrink_to_size(struct sm2_mmap *map, size_t shrink_size)
{
	struct stat st;
	int err;

	err = fstat(map->fd, &st);
	if (OFI_UNLIKELY(err))
		goto out;

	if (st.st_size > shrink_size) {
		/* we need to shrink the file. */
		FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
			"Shrinking SHM file to be of size %zu\n", shrink_size);
		err = ftruncate(map->fd, shrink_size);
		if (OFI_UNLIKELY(err))
			goto out;

		/* now un-map and re-map the file */
		err = munmap(map->base, map->size);
		if (OFI_UNLIKELY(err))
			goto out;
		map->base = mmap(0, shrink_size, PROT_READ | PROT_WRITE,
				 MAP_SHARED, map->fd, 0);
		if (OFI_UNLIKELY(map->base == MAP_FAILED)) {
			FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
				"Failed to remap when decreasing the map size. "
				"st.st_size=%ld shrink_size=%ld\n",
				st.st_size, shrink_size);
			map->base = NULL;
		}
		map->size = shrink_size;
	}

out:
	if (OFI_UNLIKELY(err)) {
		FI_WARN(&sm2_prov, FI_LOG_EP_CTRL,
			"Failed syscall during sm2_mmap_shrink_to_index()\n");
	}
	return map->base;
}

/*
 * If everything in the file is dead, shrink it to fit 0 entries.
 * This deals with peer pre-allocated entries by making the assumption that
 * my PID must be alive in the file in order for me to hold any of my peers
 * allocations.
 */
static void
sm2_file_attempt_shrink(struct sm2_mmap *map)
{
	struct sm2_coord_file_header *header = (void *) map->base;
	struct sm2_ep_allocation_entry *entries = sm2_mmap_entries(map);
	int item;
	bool shrink_file = true;

	for (item = 0; item < header->ep_entries_max; item++) {
		if (entries[item].pid != 0 &&
		    pid_lives(abs(entries[item].pid))) {
			shrink_file = false;
		}
	}

	if (shrink_file) {
		/* Reset Map */
		for (item = 0; item < header->ep_entries_max; item++) {
			entries[item].pid = 0;
			entries[item].ep_name[0] = '\0';
			entries[item].startup_ready = 0;
		}

		/* Shrink File */
		sm2_mmap_shrink_to_size(map, header->ep_regions_offset);
	}
}
