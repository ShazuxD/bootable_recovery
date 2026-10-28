// mergedbg.hpp - logging-only helpers for debugging TWRP's Virtual A/B merge check.
// Header-only (static inline), no behaviour change. Everything is logged with the
// "[mergedbg]" prefix via LOGINFO, so it ends up in recovery.log.
//
// Place next to partitionmanager.cpp in bootable/recovery/ (twinstall.cpp includes it as
// "../mergedbg.hpp").
#ifndef _MERGEDBG_HPP
#define _MERGEDBG_HPP

#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <fstream>
#include <string>

#include "twcommon.h"

static inline std::string MergeDbg_First_Line(const std::string& path) {
	std::ifstream f(path.c_str());
	std::string line;
	if (f.good())
		std::getline(f, line);
	return line.empty() ? "?" : line;
}

// Every device-mapper node: dm-N, its name, suspended flag and holders (devices stacked on top
// of it - a non-empty holders list means something is using it).
static inline void MergeDbg_Dump_Dm(const char* tag) {
	LOGINFO("[mergedbg] ---- dm devices (%s) ----\n", tag);
	DIR* d = opendir("/sys/block");
	if (!d) {
		LOGINFO("[mergedbg] cannot open /sys/block: %s\n", strerror(errno));
		return;
	}
	struct dirent* e;
	while ((e = readdir(d)) != NULL) {
		if (strncmp(e->d_name, "dm-", 3) != 0)
			continue;
		std::string base = std::string("/sys/block/") + e->d_name;
		std::string holders;
		DIR* h = opendir((base + "/holders").c_str());
		if (h) {
			struct dirent* he;
			while ((he = readdir(h)) != NULL) {
				if (he->d_name[0] == '.')
					continue;
				holders += std::string(he->d_name) + " ";
			}
			closedir(h);
		}
		LOGINFO("[mergedbg] %s name=%s suspended=%s holders=[%s]\n", e->d_name,
				MergeDbg_First_Line(base + "/dm/name").c_str(),
				MergeDbg_First_Line(base + "/dm/suspended").c_str(), holders.c_str());
	}
	closedir(d);
}

// /dev/block/mapper entries and where each symlink points.
static inline void MergeDbg_Dump_Mapper(const char* tag) {
	LOGINFO("[mergedbg] ---- /dev/block/mapper (%s) ----\n", tag);
	DIR* d = opendir("/dev/block/mapper");
	if (!d) {
		LOGINFO("[mergedbg] cannot open /dev/block/mapper: %s\n", strerror(errno));
		return;
	}
	struct dirent* e;
	while ((e = readdir(d)) != NULL) {
		if (e->d_name[0] == '.')
			continue;
		std::string p = std::string("/dev/block/mapper/") + e->d_name;
		char target[PATH_MAX];
		ssize_t n = readlink(p.c_str(), target, sizeof(target) - 1);
		if (n > 0) {
			target[n] = '\0';
			LOGINFO("[mergedbg] %s -> %s\n", e->d_name, target);
		} else {
			LOGINFO("[mergedbg] %s\n", e->d_name);
		}
	}
	closedir(d);
}

// Which dm / mapper devices (and /metadata) are currently mounted.
static inline void MergeDbg_Dump_Mounts(const char* tag) {
	LOGINFO("[mergedbg] ---- mounts of dm/mapper/metadata (%s) ----\n", tag);
	std::ifstream f("/proc/mounts");
	std::string line;
	while (std::getline(f, line)) {
		if (line.find("/dev/block/dm-") != std::string::npos ||
			line.find("/dev/block/mapper/") != std::string::npos ||
			line.find("/metadata") != std::string::npos)
			LOGINFO("[mergedbg] %s\n", line.c_str());
	}
}

// Entries of a directory (used for /metadata/ota, where libsnapshot keeps its state).
static inline void MergeDbg_List_Dir(const char* path) {
	LOGINFO("[mergedbg] ---- ls %s ----\n", path);
	DIR* d = opendir(path);
	if (!d) {
		LOGINFO("[mergedbg] cannot open %s: %s\n", path, strerror(errno));
		return;
	}
	struct dirent* e;
	while ((e = readdir(d)) != NULL) {
		if (e->d_name[0] == '.')
			continue;
		LOGINFO("[mergedbg] %s/%s\n", path, e->d_name);
	}
	closedir(d);
}

// True if 'path' holds an entry whose name contains 'needle'.
static inline bool MergeDbg_Has_Entry(const char* path, const char* needle) {
	bool found = false;
	DIR* d = opendir(path);
	if (!d)
		return false;
	struct dirent* e;
	while ((e = readdir(d)) != NULL) {
		if (strstr(e->d_name, needle) != NULL)
			found = true;
	}
	closedir(d);
	return found;
}

// One-call snapshot of everything relevant: dm devices, mapper links, mounts, snapshot state dir.
static inline void MergeDbg_Snapshot(const char* tag) {
	LOGINFO("[mergedbg] ======== %s ========\n", tag);
	MergeDbg_Dump_Dm(tag);
	MergeDbg_Dump_Mapper(tag);
	MergeDbg_Dump_Mounts(tag);
	MergeDbg_List_Dir("/metadata/ota");
}

#endif // _MERGEDBG_HPP
