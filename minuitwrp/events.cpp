/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/poll.h>
#include <limits.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <thread>
#include <signal.h>
#include <time.h>

#ifdef USE_QTI_HAPTICS
#include <android/hardware/vibrator/1.2/IVibrator.h>
#endif

#ifdef USE_QTI_AIDL_HAPTICS
#include <aidl/android/hardware/vibrator/IVibrator.h>
#include <android/binder_manager.h>
using ::aidl::android::hardware::vibrator::IVibrator;
#ifdef USE_QTI_AIDL_HAPTICS_FQNAME
static const std::string kVibratorInstance = std::string("android.hardware.vibrator.") + USE_QTI_AIDL_HAPTICS_FQNAME;
#else
static const std::string kVibratorInstance = std::string(IVibrator::descriptor) + "/default";
#endif
#ifdef USE_QTI_AIDL_HAPTICS_FIX_OFF
static std::atomic_int vib_on_count = 0;
#endif
#endif

#include "common.h"
#include "twcommon.h"

#include "minuitwrp/minui.h"

//#define _EVENT_LOGGING

#define MAX_DEVICES         32

#define VIBRATOR_TIMEOUT_FILE	"/sys/class/timed_output/vibrator/enable"
#define VIBRATOR_TIME_MS    50

#define LEDS_HAPTICS_DURATION_FILE	"/sys/class/leds/vibrator/duration"
#define LEDS_HAPTICS_ACTIVATE_FILE	"/sys/class/leds/vibrator/activate"

#ifndef SYN_REPORT
#define SYN_REPORT          0x00
#endif
#ifndef SYN_CONFIG
#define SYN_CONFIG          0x01
#endif
#ifndef SYN_MT_REPORT
#define SYN_MT_REPORT       0x02
#endif

#define ABS_MT_POSITION     0x2a /* Group a set of X and Y */
#define ABS_MT_AMPLITUDE    0x2b /* Group a set of Z and W */
#define ABS_MT_SLOT         0x2f
#define ABS_MT_TOUCH_MAJOR  0x30
#define ABS_MT_TOUCH_MINOR  0x31
#define ABS_MT_WIDTH_MAJOR  0x32
#define ABS_MT_WIDTH_MINOR  0x33
#define ABS_MT_ORIENTATION  0x34
#define ABS_MT_POSITION_X   0x35
#define ABS_MT_POSITION_Y   0x36
#define ABS_MT_TOOL_TYPE    0x37
#define ABS_MT_BLOB_ID      0x38
#define ABS_MT_TRACKING_ID  0x39
#define ABS_MT_PRESSURE     0x3a
#define ABS_MT_DISTANCE     0x3b

enum {
    DOWN_NOT,
    DOWN_SENT,
    DOWN_RELEASED,
};

struct virtualkey {
    int scancode;
    int centerx, centery;
    int width, height;
};

struct position {
    int x, y;
    int synced;
    struct input_absinfo xi, yi;
};

struct ev {
    struct pollfd *fd;

    struct virtualkey *vks;
    int vk_count;

    char deviceName[64];

    int ignored;

    struct position p, mt_p;
    int down;
};

static struct pollfd ev_fds[MAX_DEVICES];
static struct ev evs[MAX_DEVICES];
static unsigned ev_count = 0;
static struct timeval lastInputStat;
static time_t lastInputMTime;
static int has_mouse = 0;

static inline int ABS(int x) {
    return x<0?-x:x;
}

int write_to_file(const std::string& fn, const std::string& line) {
	FILE *file;
	file = fopen(fn.c_str(), "w");
	if (file != NULL) {
		fwrite(line.c_str(), line.size(), 1, file);
		fclose(file);
		return 0;
	}
	LOGI("Cannot find file %s\n", fn.c_str());
	return -1;
}

#ifndef TW_NO_HAPTICS
#ifndef TW_HAPTICS_TSPDRV
static timer_t ruby_cont_timer = {};
static bool ruby_cont_timer_ready = false;
static std::string ruby_active_cont_path = "";
static bool ruby_cont_logged = false;

static void ruby_stop_awinic_cont(union sigval) {
    if (!ruby_active_cont_path.empty()) {
        write_to_file(ruby_active_cont_path, "0");
    }
}

static bool ruby_init_cont_timer() {
    if (ruby_cont_timer_ready) return true;

    struct sigevent sev = {};
    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_notify_function = ruby_stop_awinic_cont;
    sev.sigev_value.sival_ptr = nullptr;

    if (timer_create(CLOCK_MONOTONIC, &sev, &ruby_cont_timer) != 0) {
        return false;
    }
    ruby_cont_timer_ready = true;
    return true;
}

static bool ruby_vibrate_awinic_cont(int timeout_ms) {
    if (timeout_ms <= 0) return false;

    if (ruby_active_cont_path.empty()) {
        static const std::string cont_candidates[] = {
            "/sys/devices/platform/11e00000.i2c0/i2c-0/0-005a/cont",
            "/sys/bus/i2c/drivers/awinic_haptic/0-005a/cont",
            "/sys/bus/i2c/drivers/awinic_haptic/1-005a/cont",
            "/sys/bus/i2c/drivers/awinic_haptic/2-005a/cont",
            "/sys/bus/i2c/drivers/awinic_haptic/2-0058/cont",
            "/sys/bus/i2c/drivers/awinic_haptic/3-005a/cont",
            "/sys/bus/i2c/drivers/awinic_haptic/7-005a/cont",
            "/sys/bus/i2c/drivers/aw8697_haptic/0-005a/cont",
            "/sys/bus/i2c/drivers/aw8697_haptic/1-005a/cont",
            "/sys/bus/i2c/drivers/aw8697_haptic/2-005a/cont",
            "/sys/bus/i2c/drivers/aw8697_haptic/3-005a/cont",
            "/sys/bus/i2c/drivers/aw8697_haptic/7-005a/cont",
            "/sys/bus/i2c/drivers/aw8697_haptic/9-005a/cont",
            "/sys/class/timed_output/vibrator/cont"
        };
        for (const auto& p : cont_candidates) {
            if (access(p.c_str(), W_OK) == 0) {
                ruby_active_cont_path = p;
                std::string parent = p.substr(0, p.rfind('/'));
                std::string state_file = parent + "/state";
                if (access(state_file.c_str(), W_OK) == 0) {
                    write_to_file(state_file, "1");
                }
                break;
            }
        }
        if (ruby_active_cont_path.empty()) {
            return false;
        }
    }

    const int duration_ms = timeout_ms > 1000 ? 1000 : timeout_ms;

    if (write_to_file(ruby_active_cont_path, "1") != 0) {
        return false;
    }

    if (!ruby_init_cont_timer()) {
        std::string path = ruby_active_cont_path;
        std::thread([path, duration_ms]() {
            usleep(duration_ms * 1000);
            write_to_file(path, "0");
        }).detach();
        return true;
    }

    struct itimerspec its = {};
    its.it_value.tv_sec = duration_ms / 1000;
    its.it_value.tv_nsec = static_cast<long>(duration_ms % 1000) * 1000000L;
    if (its.it_value.tv_sec == 0 && its.it_value.tv_nsec == 0) {
        its.it_value.tv_nsec = 1000000L;
    }

    if (timer_settime(ruby_cont_timer, 0, &its, nullptr) != 0) {
        write_to_file(ruby_active_cont_path, "0");
        return false;
    }

    if (!ruby_cont_logged) {
        LOGI("VIBRATOR: Using Awinic continuous haptics at %s\n", ruby_active_cont_path.c_str());
        ruby_cont_logged = true;
    }
    return true;
}

static int ruby_ff_fd = -2;
static int ruby_ff_effect_id = -1;
static int ruby_ff_effect_type = -1;

static bool ruby_ff_test_bit(unsigned int bit, const unsigned long* bits) {
    const unsigned int bits_per_long = sizeof(unsigned long) * 8U;
    return (bits[bit / bits_per_long] >> (bit % bits_per_long)) & 1UL;
}

static int ruby_open_input_ff_haptics() {
    if (ruby_ff_fd >= 0) return ruby_ff_fd;
    ruby_ff_fd = -2;

    DIR* dir = opendir("/dev/input");
    if (dir == nullptr) return -1;

    struct dirent* de = nullptr;
    while ((de = readdir(dir)) != nullptr) {
        if (strncmp(de->d_name, "event", 5) != 0) continue;

        char path[PATH_MAX] = {};
        snprintf(path, sizeof(path), "/dev/input/%s", de->d_name);
        int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;

        char name[256] = {};
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
            close(fd);
            continue;
        }

        const unsigned int bits_per_long = sizeof(unsigned long) * 8U;
        unsigned long ev_bits[(EV_MAX + bits_per_long) / bits_per_long] = {};
        if (ioctl(fd, EVIOCGBIT(0, sizeof(ev_bits)), ev_bits) < 0 ||
            !ruby_ff_test_bit(EV_FF, ev_bits)) {
            close(fd);
            continue;
        }

        unsigned long ff_bits[(FF_MAX + bits_per_long) / bits_per_long] = {};
        if (ioctl(fd, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
            close(fd);
            continue;
        }

        if (ruby_ff_test_bit(FF_RUMBLE, ff_bits)) {
            ruby_ff_effect_type = FF_RUMBLE;
        } else if (ruby_ff_test_bit(FF_CONSTANT, ff_bits)) {
            ruby_ff_effect_type = FF_CONSTANT;
        } else {
            close(fd);
            continue;
        }

        ruby_ff_fd = fd;
        LOGI("VIBRATOR: Using input FF device '%s' at %s (effect=%s)\n",
             name, path, ruby_ff_effect_type == FF_RUMBLE ? "FF_RUMBLE" : "FF_CONSTANT");
        break;
    }

    closedir(dir);
    return ruby_ff_fd >= 0 ? ruby_ff_fd : -1;
}

static void ruby_reset_ff_backend(int fd) {
    if (ruby_ff_effect_id >= 0) {
        struct input_event stop = {};
        stop.type = EV_FF;
        stop.code = static_cast<__u16>(ruby_ff_effect_id);
        stop.value = 0;
        (void)write(fd, &stop, sizeof(stop));
        (void)ioctl(fd, EVIOCRMFF, ruby_ff_effect_id);
    }
    ruby_ff_effect_id = -1;
    ruby_ff_effect_type = -1;
    if (fd >= 0) close(fd);
    ruby_ff_fd = -2;
}

static bool ruby_vibrate_input_ff(int timeout_ms) {
    int fd = ruby_open_input_ff_haptics();
    if (fd < 0 || timeout_ms <= 0) return false;

    struct ff_effect effect = {};
    effect.type = static_cast<__u16>(ruby_ff_effect_type);
    effect.id = static_cast<__s16>(ruby_ff_effect_id);
    effect.replay.length = static_cast<__u16>(timeout_ms > 1000 ? 1000 : timeout_ms);
    effect.replay.delay = 0;

    if (ruby_ff_effect_type == FF_CONSTANT) {
        effect.u.constant.level = 0x3fff;
        effect.u.constant.envelope.attack_length = 0;
        effect.u.constant.envelope.fade_length = 0;
    } else {
        effect.u.rumble.strong_magnitude = 0x3fff;
        effect.u.rumble.weak_magnitude = 0x2fff;
    }

    if (ioctl(fd, EVIOCSFF, &effect) < 0) {
        ruby_reset_ff_backend(fd);
        return false;
    }
    ruby_ff_effect_id = effect.id;

    struct input_event play = {};
    play.type = EV_FF;
    play.code = static_cast<__u16>(ruby_ff_effect_id);
    play.value = 1;

    if (write(fd, &play, sizeof(play)) != static_cast<ssize_t>(sizeof(play))) {
        ruby_reset_ff_backend(fd);
        return false;
    }
    return true;
}

static bool ruby_vibrate_sysfs(int timeout_ms) {
    if (timeout_ms <= 0) return false;
    char tout[16];
    snprintf(tout, sizeof(tout), "%d", timeout_ms);

    static const std::string possible_haptic_paths[] = {
         "/sys/devices/platform/11e00000.i2c0/i2c-0/0-005a",
         "/sys/bus/i2c/drivers/awinic_haptic/0-005a",
         "/sys/bus/i2c/drivers/awinic_haptic/1-005a",
         "/sys/bus/i2c/drivers/awinic_haptic/2-005a",
         "/sys/bus/i2c/drivers/awinic_haptic/2-0058",
         "/sys/bus/i2c/drivers/awinic_haptic/3-005a",
         "/sys/bus/i2c/drivers/awinic_haptic/7-005a",
         "/sys/bus/i2c/drivers/aw8697_haptic/0-005a",
         "/sys/bus/i2c/drivers/aw8697_haptic/1-005a",
         "/sys/bus/i2c/drivers/aw8697_haptic/2-005a",
         "/sys/bus/i2c/drivers/aw8697_haptic/3-005a",
         "/sys/bus/i2c/drivers/aw8697_haptic/7-005a",
         "/sys/bus/i2c/drivers/aw8697_haptic/9-005a",
         "/sys/bus/i2c/drivers/aw8624_haptic/2-005a",
         "/sys/class/leds/vibrator"
    };

    for (const auto& hpath : possible_haptic_paths) {
        std::string act = hpath + "/activate";
        std::string dur = hpath + "/duration";
        if (access(act.c_str(), W_OK) == 0) {
            std::string state_file = hpath + "/state";
            if (access(state_file.c_str(), W_OK) == 0) {
                write_to_file(state_file, "1");
            }
            std::string idx = hpath + "/index";
            if (access(idx.c_str(), W_OK) == 0) {
                write_to_file(idx, "1");
            }
            std::string gain = hpath + "/gain";
            if (access(gain.c_str(), W_OK) == 0) {
                write_to_file(gain, "80");
            }
            write_to_file(dur, tout);
            write_to_file(act, "1");
            return true;
        }
    }
    return false;
}

int vibrate(int timeout_ms)
{
    if (timeout_ms > 70) timeout_ms = 70;
    char tout[16];
    snprintf(tout, sizeof(tout), "%d", timeout_ms);

#ifdef USE_QTI_HAPTICS
    android::sp<android::hardware::vibrator::V1_2::IVibrator> vib = android::hardware::vibrator::V1_2::IVibrator::getService();
    if (vib != nullptr) {
        vib->on((uint32_t)timeout_ms);
        return 0;
    }
#elif defined(USE_QTI_AIDL_HAPTICS)
    std::shared_ptr<IVibrator> vib = IVibrator::fromBinder(ndk::SpAIBinder(AServiceManager_checkService(kVibratorInstance.c_str())));
    if (vib != nullptr) {
#ifdef USE_QTI_AIDL_HAPTICS_FIX_OFF
        std::thread([vib, timeout_ms] {
            if (vib->on((uint32_t)timeout_ms, nullptr).isOk()) {
                vib_on_count++;
                usleep(timeout_ms * 1000);
                vib_on_count--;
                if (!vib_on_count) vib->off();
            }
        }).detach();
#else
        vib->on((uint32_t)timeout_ms, nullptr);
#endif
        return 0;
    }
#elif defined(USE_SAMSUNG_HAPTICS)
    if (std::ifstream(VIBRATOR_TIMEOUT_FILE).good()) {
        write_to_file(VIBRATOR_TIMEOUT_FILE, tout);
        return 0;
    }
#endif

    // 1. Awinic hardware continuous mode (AW869x / MT6877)
    if (ruby_vibrate_awinic_cont(timeout_ms)) {
        return 0;
    }

    // 2. Linux Input Force Feedback subsystem (/dev/input/event*)
    if (ruby_vibrate_input_ff(timeout_ms)) {
        return 0;
    }

    // 3. Sysfs activate / duration (with index & gain setup)
    if (ruby_vibrate_sysfs(timeout_ms)) {
        return 0;
    }

    // 4. Legacy timed_output enable node
    write_to_file(VIBRATOR_TIMEOUT_FILE, tout);
    return 0;
}
#endif
#endif

/* Returns empty tokens */
static char *vk_strtok_r(char *str, const char *delim, char **save_str)
{
    if(!str)
    {
        if(!*save_str)
            return NULL;

        str = (*save_str) + 1;
    }
    *save_str = strpbrk(str, delim);

    if (*save_str)
        **save_str = '\0';

    return str;
}

static int vk_init(struct ev *e)
{
    char vk_path[PATH_MAX] = "/sys/board_properties/virtualkeys.";
    char vks[2048], *ts = NULL;
    ssize_t len;
    int vk_fd;
    int i;

    e->vk_count = 0;

    len = strlen(vk_path);
    len = ioctl(e->fd->fd, EVIOCGNAME(sizeof(e->deviceName)), e->deviceName);
    if (len <= 0)
    {
        LOGE("Unable to query event object.\n");
        return -1;
    }
#ifdef _EVENT_LOGGING
    printf("Event object: %s\n", e->deviceName);
#endif

#ifdef WHITELIST_INPUT
    if (strcmp(e->deviceName, EXPAND(WHITELIST_INPUT)) != 0)
    {
        e->ignored = 1;
    }
#else
#ifndef TW_INPUT_BLACKLIST
    // Blacklist these "input" devices, use TW_INPUT_BLACKLIST := "accelerometer\x0atest1\x0atest2" using the \x0a as a separator between input devices
    if (strcmp(e->deviceName, "bma250") == 0 || strcmp(e->deviceName, "bma150") == 0)
    {
        LOGI("Blacklisting input device: %s\n", e->deviceName);
        e->ignored = 1;
    }
#else
    char* bl = strdup(EXPAND(TW_INPUT_BLACKLIST));
    char* blacklist = strtok(bl, "\n");

    while (blacklist != NULL) {
        if (strcmp(e->deviceName, blacklist) == 0) {
            LOGI("Blacklisting input device: %s\n", blacklist);
            e->ignored = 1;
        }
        blacklist = strtok(NULL, "\n");
    }
    free(bl);
#endif
#endif

    strcat(vk_path, e->deviceName);

    // Some devices split the keys from the touchscreen
    e->vk_count = 0;
    vk_fd = open(vk_path, O_RDONLY);
    if (vk_fd >= 0)
    {
        len = read(vk_fd, vks, sizeof(vks)-1);
        close(vk_fd);
        if (len <= 0)
            return -1;

        vks[len] = '\0';

        /* Parse a line like:
            keytype:keycode:centerx:centery:width:height:keytype2:keycode2:centerx2:...
        */
        for (ts = vks, e->vk_count = 1; *ts; ++ts) {
            if (*ts == ':')
                ++e->vk_count;
        }

        if (e->vk_count % 6) {
            LOGI("minui: %s is %d %% 6\n", vk_path, e->vk_count % 6);
        }
        e->vk_count /= 6;
        if (e->vk_count <= 0)
            return -1;

        e->down = DOWN_NOT;
    }

    ioctl(e->fd->fd, EVIOCGABS(ABS_X), &e->p.xi);
    ioctl(e->fd->fd, EVIOCGABS(ABS_Y), &e->p.yi);
    e->p.synced = 0;
#ifdef _EVENT_LOGGING
    printf("EV: ST minX: %d  maxX: %d  minY: %d  maxY: %d\n", e->p.xi.minimum, e->p.xi.maximum, e->p.yi.minimum, e->p.yi.maximum);
#endif

    ioctl(e->fd->fd, EVIOCGABS(ABS_MT_POSITION_X), &e->mt_p.xi);
    ioctl(e->fd->fd, EVIOCGABS(ABS_MT_POSITION_Y), &e->mt_p.yi);
    e->mt_p.synced = 0;
#ifdef _EVENT_LOGGING
    printf("EV: MT minX: %d  maxX: %d  minY: %d  maxY: %d\n", e->mt_p.xi.minimum, e->mt_p.xi.maximum, e->mt_p.yi.minimum, e->mt_p.yi.maximum);
#endif

    e->vks = (virtualkey *)malloc(sizeof(*e->vks) * e->vk_count);

    for (i = 0; i < e->vk_count; ++i) {
        char *token[6];
        int j;

        for (j = 0; j < 6; ++j) {
            token[j] = vk_strtok_r((i||j)?NULL:vks, ":", &ts);
        }

        if (strcmp(token[0], "0x01") != 0) {
            /* Java does string compare, so we do too. */
            LOGI("minui: %s: ignoring unknown virtual key type %s\n", vk_path, token[0]);
            continue;
        }

        e->vks[i].scancode = strtol(token[1], NULL, 0);
        e->vks[i].centerx = strtol(token[2], NULL, 0);
        e->vks[i].centery = strtol(token[3], NULL, 0);
        e->vks[i].width = strtol(token[4], NULL, 0);
        e->vks[i].height = strtol(token[5], NULL, 0);
    }

    return 0;
}

#define BITS_PER_LONG (sizeof(long) * 8)
#define NBITS(x) ((((x)-1)/BITS_PER_LONG)+1)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)	((array[LONG(bit)] >> OFF(bit)) & 1)

// Check for EV_REL (REL_X and REL_Y) and, because touchscreens can have those too,
// check also for EV_KEY (BTN_LEFT and BTN_RIGHT)
static void check_mouse(int fd, const char* deviceName)
{
	if(has_mouse)
		return;

	unsigned long bit[EV_MAX][NBITS(KEY_MAX)];
	memset(bit, 0, sizeof(bit));
	ioctl(fd, EVIOCGBIT(0, EV_MAX), bit[0]);

	if(!test_bit(EV_REL, bit[0]) || !test_bit(EV_KEY, bit[0]))
		return;

	ioctl(fd, EVIOCGBIT(EV_REL, KEY_MAX), bit[EV_REL]);
	if(!test_bit(REL_X, bit[EV_REL]) || !test_bit(REL_Y, bit[EV_REL]))
		return;

	ioctl(fd, EVIOCGBIT(EV_KEY, KEY_MAX), bit[EV_KEY]);
	if(!test_bit(BTN_LEFT, bit[EV_KEY]) || !test_bit(BTN_RIGHT, bit[EV_KEY]))
		return;

	LOGI("Found mouse '%s'\n", deviceName);
	has_mouse = 1;
}

int ev_has_mouse(void)
{
	return has_mouse;
}

int ev_init(void)
{
    DIR *dir;
    struct dirent *de;
    int fd;

    has_mouse = 0;

	dir = opendir("/dev/input");
    if(dir != 0) {
        while((de = readdir(dir))) {
#ifdef _EVENT_LOGGING
            fprintf(stderr,"/dev/input/%s\n", de->d_name);
#endif
            if(strncmp(de->d_name,"event",5)) continue;
            fd = openat(dirfd(dir), de->d_name, O_RDONLY);
            if(fd < 0) continue;

			ev_fds[ev_count].fd = fd;
            ev_fds[ev_count].events = POLLIN;
            evs[ev_count].fd = &ev_fds[ev_count];

            /* Load virtualkeys if there are any */
            vk_init(&evs[ev_count]);

            if (!evs[ev_count].ignored)
                check_mouse(fd, evs[ev_count].deviceName);

            ev_count++;
            if(ev_count == MAX_DEVICES) break;
        }
        closedir(dir);
    }

    struct stat st;
    if(stat("/dev/input", &st) >= 0)
        lastInputMTime = st.st_mtime;
    gettimeofday(&lastInputStat, NULL);

    return 0;
}

void ev_exit(void)
{
	while (ev_count-- > 0) {
		if (evs[ev_count].vk_count) {
			free(evs[ev_count].vks);
			evs[ev_count].vk_count = 0;
		}
		close(ev_fds[ev_count].fd);
	}
	ev_count = 0;
}

/*static int vk_inside_display(__s32 value, struct input_absinfo *info, int screen_size)
{
    int screen_pos;

    if (info->minimum == info->maximum)
        return 0;

    screen_pos = (value - info->minimum) * (screen_size - 1) / (info->maximum - info->minimum);
    return (screen_pos >= 0 && screen_pos < screen_size);
}*/

static int vk_tp_to_screen(struct position *p, int *x, int *y)
{
    if (p->xi.minimum == p->xi.maximum || p->yi.minimum == p->yi.maximum)
    {
        // In this case, we assume the screen dimensions are the same.
        *x = p->x;
        *y = p->y;
        return 0;
    }

#ifdef _EVENT_LOGGING
    printf("EV: p->x=%d  x-range=%d,%d  fb-width=%d\n", p->x, p->xi.minimum, p->xi.maximum, gr_fb_width());
#endif

#ifndef RECOVERY_TOUCHSCREEN_SWAP_XY
    int fb_width = gr_fb_width();
    int fb_height = gr_fb_height();
#else
    // We need to swap the scaling sizes, too
    int fb_width = gr_fb_height();
    int fb_height = gr_fb_width();
#endif

    *x = (p->x - p->xi.minimum) * (fb_width - 1) / (p->xi.maximum - p->xi.minimum);
    *y = (p->y - p->yi.minimum) * (fb_height - 1) / (p->yi.maximum - p->yi.minimum);

    if (*x >= 0 && *x < fb_width &&
        *y >= 0 && *y < fb_height)
    {
        return 0;
    }

    return 1;
}

/* Translate a virtual key in to a real key event, if needed */
/* Returns non-zero when the event should be consumed */
static int vk_modify(struct ev *e, struct input_event *ev)
{
    static int downX = -1, downY = -1;
    static int discard = 0;
    static int last_virt_key = 0;
    static int lastWasSynReport = 0;
    static int touchReleaseOnNextSynReport = 0;
	static int use_tracking_id_negative_as_touch_release = 0; // On some devices, type: 3  code: 39  value: -1, aka EV_ABS ABS_MT_TRACKING_ID -1 indicates a true touch release
    int i;
    int x, y;

    // This is used to ditch useless event handlers, like an accelerometer
    if (e->ignored)     return 1;

    if (ev->type == EV_REL && ev->code == REL_Z)
    {
        // This appears to be an accelerometer or another strange input device. It's not the touchscreen.
#ifdef _EVENT_LOGGING
        printf("EV: Device disabled due to non-touchscreen messages.\n");
#endif
        e->ignored = 1;
        return 1;
    }

#ifdef _EVENT_LOGGING
    printf("EV: %s => type: %x  code: %x  value: %d\n", e->deviceName, ev->type, ev->code, ev->value);
#endif

	// Handle keyboard events, value of 1 indicates key down, 0 indicates key up
	if (ev->type == EV_KEY) {
		return 0;
	}

    if (ev->type == EV_ABS) {
        switch (ev->code) {

        case ABS_X: //00
            e->p.synced |= 0x01;
            e->p.x = ev->value;
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS  ABS_X  %d\n", e->deviceName, ev->value);
#endif
            break;

        case ABS_Y: //01
            e->p.synced |= 0x02;
            e->p.y = ev->value;
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS  ABS_Y  %d\n", e->deviceName, ev->value);
#endif
            break;

        case ABS_MT_POSITION: //2a
            e->mt_p.synced = 0x03;
            if (ev->value == (1 << 31))
            {
#ifndef TW_IGNORE_MT_POSITION_0
                e->mt_p.x = 0;
                e->mt_p.y = 0;
                lastWasSynReport = 1;
#endif
#ifdef _EVENT_LOGGING
#ifndef TW_IGNORE_MT_POSITION_0
                printf("EV: %s => EV_ABS  ABS_MT_POSITION  %d, set x and y to 0 and lastWasSynReport to 1\n", e->deviceName, ev->value);
#else
                printf("Ignoring ABS_MT_POSITION 0\n", e->deviceName, ev->value);
#endif
#endif
            }
            else
            {
                lastWasSynReport = 0;
                e->mt_p.x = (ev->value & 0x7FFF0000) >> 16;
                e->mt_p.y = (ev->value & 0xFFFF);
#ifdef _EVENT_LOGGING
                printf("EV: %s => EV_ABS  ABS_MT_POSITION  %d, set x: %d and y: %d and lastWasSynReport to 0\n", e->deviceName, ev->value, (ev->value & 0x7FFF0000) >> 16, (ev->value & 0xFFFF));
#endif
            }
            break;

        case ABS_MT_TOUCH_MAJOR: //30
            if (ev->value == 0)
            {
#ifndef TW_IGNORE_MAJOR_AXIS_0
                // We're in a touch release, although some devices will still send positions as well
                e->mt_p.x = 0;
                e->mt_p.y = 0;
                touchReleaseOnNextSynReport = 1;
#endif
            }
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS  ABS_MT_TOUCH_MAJOR  %d\n", e->deviceName, ev->value);
#endif
            break;

		case ABS_MT_PRESSURE: //3a
                    if (ev->value == 0)
            {
                // We're in a touch release, although some devices will still send positions as well
                e->mt_p.x = 0;
                e->mt_p.y = 0;
                touchReleaseOnNextSynReport = 1;
            }
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS  ABS_MT_PRESSURE  %d\n", e->deviceName, ev->value);
#endif
            break;

		case ABS_MT_POSITION_X: //35
            e->mt_p.synced |= 0x01;
            e->mt_p.x = ev->value;
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS  ABS_MT_POSITION_X  %d\n", e->deviceName, ev->value);
#endif
            break;

        case ABS_MT_POSITION_Y: //36
            e->mt_p.synced |= 0x02;
            e->mt_p.y = ev->value;
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS  ABS_MT_POSITION_Y  %d\n", e->deviceName, ev->value);
#endif
            break;

        case ABS_MT_TOUCH_MINOR: //31
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS ABS_MT_TOUCH_MINOR %d\n", e->deviceName, ev->value);
#endif
            break;

        case ABS_MT_WIDTH_MAJOR: //32
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS ABS_MT_WIDTH_MAJOR %d\n", e->deviceName, ev->value);
#endif
            break;

        case ABS_MT_WIDTH_MINOR: //33
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS ABS_MT_WIDTH_MINOR %d\n", e->deviceName, ev->value);
#endif
            break;

        case ABS_MT_TRACKING_ID: //39
#ifdef TW_IGNORE_ABS_MT_TRACKING_ID
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS ABS_MT_TRACKING_ID %d ignored\n", e->deviceName, ev->value);
#endif
            return 1;
#endif
            if (ev->value < 0) {
                e->mt_p.x = 0;
                e->mt_p.y = 0;
                touchReleaseOnNextSynReport = 2;
                use_tracking_id_negative_as_touch_release = 1;
#ifdef _EVENT_LOGGING
                if (use_tracking_id_negative_as_touch_release)
                    printf("using ABS_MT_TRACKING_ID value -1 to indicate touch releases\n");
#endif
            }
#ifdef _EVENT_LOGGING
            printf("EV: %s => EV_ABS ABS_MT_TRACKING_ID %d\n", e->deviceName, ev->value);
#endif
            break;

#ifdef _EVENT_LOGGING
        // These are for touch logging purposes only
        case ABS_MT_ORIENTATION: //34
            printf("EV: %s => EV_ABS ABS_MT_ORIENTATION %d\n", e->deviceName, ev->value);
			return 1;
            break;

		case ABS_MT_TOOL_TYPE: //37
            LOGI("EV: %s => EV_ABS ABS_MT_TOOL_TYPE %d\n", e->deviceName, ev->value);
			return 1;
            break;

        case ABS_MT_BLOB_ID: //38
            printf("EV: %s => EV_ABS ABS_MT_BLOB_ID %d\n", e->deviceName, ev->value);
			return 1;
            break;

		case ABS_MT_DISTANCE: //3b
            printf("EV: %s => EV_ABS ABS_MT_DISTANCE %d\n", e->deviceName, ev->value);
			return 1;
            break;
        case ABS_MT_SLOT:
            printf("EV: %s => ABS_MT_SLOT %d\n", e->deviceName, ev->value);
			return 1;
			break;
#endif

        default:
            // This is an unhandled message, just skip it
            return 1;
        }

        if (ev->code != ABS_MT_POSITION)
        {
            lastWasSynReport = 0;
            return 1;
        }
    }

    // Check if we should ignore the message
    if (ev->code != ABS_MT_POSITION && (ev->type != EV_SYN || (ev->code != SYN_REPORT && ev->code != SYN_MT_REPORT)))
    {
        lastWasSynReport = 0;
        return 0;
    }

#ifdef _EVENT_LOGGING
    if (ev->type == EV_SYN && ev->code == SYN_REPORT)
        printf("EV: %s => EV_SYN  SYN_REPORT\n", e->deviceName);
    if (ev->type == EV_SYN && ev->code == SYN_MT_REPORT)
        printf("EV: %s => EV_SYN  SYN_MT_REPORT\n", e->deviceName);
#endif

    // Discard the MT versions
    if (ev->code == SYN_MT_REPORT)      return 0;

    if (((lastWasSynReport == 1 || touchReleaseOnNextSynReport == 1) && !use_tracking_id_negative_as_touch_release) || (use_tracking_id_negative_as_touch_release && touchReleaseOnNextSynReport == 2))
    {
        // Reset the value
        touchReleaseOnNextSynReport = 0;

        // We are a finger-up state
        if (!discard)
        {
            // Report the key up
            ev->type = EV_ABS;
            ev->code = 0;
            ev->value = (downX << 16) | downY;
        }
        downX = -1;
        downY = -1;
        if (discard)
        {
            discard = 0;

            // Send the keyUp event
            ev->type = EV_KEY;
            ev->code = last_virt_key;
            ev->value = 0;
        }
        return 0;
    }
    lastWasSynReport = 1;

    // Retrieve where the x,y position is
    if (e->p.synced & 0x03)
    {
        vk_tp_to_screen(&e->p, &x, &y);
    }
    else if (e->mt_p.synced & 0x03)
    {
        vk_tp_to_screen(&e->mt_p, &x, &y);
    }
    else
    {
        // We don't have useful information to convey
        return 1;
    }

#ifdef RECOVERY_TOUCHSCREEN_SWAP_XY
    x ^= y;
    y ^= x;
    x ^= y;
#endif
#ifdef RECOVERY_TOUCHSCREEN_FLIP_X
    x = gr_fb_width() - x;
#endif
#ifdef RECOVERY_TOUCHSCREEN_FLIP_Y
    y = gr_fb_height() - y;
#endif

#ifdef _EVENT_LOGGING
    printf("EV: x: %d  y: %d\n", x, y);
#endif

    // Clear the current sync states
    e->p.synced = e->mt_p.synced = 0;

    // If we have nothing useful to report, skip it
    if (x == -1 || y == -1)     return 1;

    // Special case, we'll ignore touches on 0,0 because it usually means
    // that we received extra data after our last sync and x and y were
    // reset to 0. We should not be using 0,0 anyway.
    if (x == 0 && y == 0)
        return 1;

    // On first touch, see if we're at a virtual key
    if (downX == -1)
    {
        // Attempt mapping to virtual key
        for (i = 0; i < e->vk_count; ++i)
        {
            int xd = ABS(e->vks[i].centerx - x);
            int yd = ABS(e->vks[i].centery - y);

            if (xd < e->vks[i].width/2 && yd < e->vks[i].height/2)
            {
                ev->type = EV_KEY;
                ev->code = e->vks[i].scancode;
                ev->value = 1;

                last_virt_key = e->vks[i].scancode;

#ifndef TW_NO_HAPTICS
                vibrate(VIBRATOR_TIME_MS);
#endif

                // Mark that all further movement until lift is discard,
                // and make sure we don't come back into this area
                discard = 1;
                downX = 0;
                return 0;
            }
        }
    }

    // If we were originally a button press, discard this event
    if (discard)
    {
        return 1;
    }

    // Record where we started the touch for deciding if this is a key or a scroll
    downX = x;
    downY = y;

    ev->type = EV_ABS;
    ev->code = 1;
    ev->value = (x << 16) | y;
    return 0;
}

int ev_get(struct input_event *ev, int timeout_ms)
{
    int r;
    unsigned n;
    struct timeval curr;

    gettimeofday(&curr, NULL);
    if(curr.tv_sec - lastInputStat.tv_sec >= 2)
    {
        struct stat st;
        stat("/dev/input", &st);
        if (st.st_mtime > lastInputMTime)
        {
            LOGI("Reloading input devices\n");
            ev_exit();
            ev_init();
            lastInputMTime = st.st_mtime;
        }
        lastInputStat = curr;
    }

    r = poll(ev_fds, ev_count, timeout_ms);

    if(r > 0) {
        for(n = 0; n < ev_count; n++) {
            if(ev_fds[n].revents & POLLIN) {
                r = read(ev_fds[n].fd, ev, sizeof(*ev));
                if(r == sizeof(*ev)) {
                    if (!vk_modify(&evs[n], ev))
                        return 0;
                }
            }
        }
        return -1;
    }

    return -2;
}

int ev_wait(int timeout __unused)
{
    return -1;
}

void ev_dispatch(void)
{
    return;
}

int ev_get_input(int fd __unused, short revents __unused, struct input_event *ev __unused)
{
    return -1;
}
