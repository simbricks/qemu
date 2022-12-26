/*
 * SimBricks Memory Device
 *
 * Copyright (c) 2020-2022 Max Planck Institute for Software Systems
 * Copyright (c) 2020-2022 National University of Singapore
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "simbricks_mem.h"

#include <simbricks/mem/if.h>

// clang-format off
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/qdev-properties.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "chardev/char-fe.h"
#include "qapi/error.h"
#include "sysemu/cpus.h"
#include "hw/core/cpu.h"
#include "hw/boards.h"
// clang-format on

// #define DEBUG_PRINTS
// #define DEBUG_PRINTS_VERBOSE
bool verbose_debug_prints = false; /* skip verbose prints during startup */

#define SIMBRICKS_CLOCK QEMU_CLOCK_VIRTUAL

#define SIMBRICKS_MEM(obj) \
  OBJECT_CHECK(SimbricksMemState, obj, TYPE_MEM_SIMBRICKS_DEVICE)

typedef struct SimbricksMemRequest {
  CPUState *cpu; /* CPU associated with this request */
  QemuCond cond;
  uint64_t addr;
  uint64_t value; /* value read/to be written */
  unsigned size;

  /* store result of successful read request when resuming CPU with different
  instruction */
  uint64_t cached_addr;
  uint64_t cached_value;
  unsigned cached_size;

  bool processing;
  bool requested;
} SimbricksMemRequest;

typedef struct SimbricksMemState {
  SysBusDevice sbd;

  QemuThread thread;
  volatile bool stopping;

  /* config parameters */
  char *socket_path; /* path to ux socket to connect to */
  uint64_t mem_latency;
  uint64_t sync_period;
  uint64_t base_address;
  uint64_t size;

  MemoryRegion mr;

  struct SimbricksMemIf memif;
  struct SimbricksProtoMemMemIntro dev_intro;

  bool sync_ts_bumped;

  /* communication bewteen main io thread and worker thread
   * (protected by thr_mutex). */
  size_t reqs_len;
  SimbricksMemRequest *reqs;

  /* timers for synchronization etc. */
  bool sync;
  int sync_mode;
  int64_t ts_base;
  QEMUTimer *timer_dummy;
  QEMUTimer *timer_sync;
  QEMUTimer *timer_poll;
} SimbricksMemState;

void QEMU_NORETURN cpu_loop_exit(CPUState *cpu);

static void panic(const char *msg, ...) __attribute__((noreturn));

static void panic(const char *msg, ...) {
  va_list ap;

  va_start(ap, msg);
  error_vreport(msg, ap);
  va_end(ap);

  abort();
}

static inline uint64_t ts_to_proto(SimbricksMemState *simbricks,
                                  int64_t qemu_ts) {
  return (qemu_ts - simbricks->ts_base) * 1000;
}

static inline int64_t ts_from_proto(SimbricksMemState *simbricks,
                                    uint64_t proto_ts) {
  return (proto_ts / 1000) + simbricks->ts_base;
}

static inline volatile union SimbricksProtoMemH2M *simbricks_comm_h2m_alloc(
    SimbricksMemState *simbricks, uint64_t ts) {
  volatile union SimbricksProtoMemH2M *msg;
  while (!(msg = SimbricksMemIfH2MOutAlloc(&simbricks->memif,
                                           ts_to_proto(simbricks, ts))))
    ;

  // whenever we send a message, we need to reschedule our sync timer
  simbricks->sync_ts_bumped = true;
  return msg;
}

/******************************************************************************/
/* Worker thread */

static void simbricks_comm_m2h_rcomp(SimbricksMemState *simbricks,
                                     uint64_t cur_ts, uint64_t req_id,
                                     const void *data) {
  SimbricksMemRequest *req = simbricks->reqs + req_id;
  CPUState *cpu;

  assert(req_id <= simbricks->reqs_len);

  if (!req->processing) {
    panic("simbricks_comm_m2h_rcomp: no request currently processing");
  }

  /* copy read value from message */
  req->value = 0;
  memcpy(&req->value, data, req->size);

  req->processing = false;

  if (simbricks->sync) {
    cpu = req->cpu;

#ifdef DEBUG_PRINTS
    warn_report("simbricks_comm_m2h_rcomp: kicking cpu %lu ts=%lu", req_id,
                cur_ts);
#endif

    cpu->stopped = 0;
    // qemu_cpu_kick(cpu);
  } else {
    qemu_cond_broadcast(&req->cond);
  }
}

/* process and complete message */
static void simbricks_comm_m2h_process(
    SimbricksMemState *simbricks, int64_t ts,
    volatile union SimbricksProtoMemM2H *msg) {
  uint8_t type;

  type = SimbricksMemIfM2HInType(&simbricks->memif, msg);
#ifdef DEBUG_PRINTS_VERBOSE
  if (verbose_debug_prints) {
    warn_report("simbricks_comm_m2h_process: ts=%ld type=%u", ts, type);
  }
#endif

  switch (type) {
    case SIMBRICKS_PROTO_MSG_TYPE_SYNC:
      /* nop */
      break;
    case SIMBRICKS_PROTO_MEM_M2H_MSG_READCOMP:
      simbricks_comm_m2h_rcomp(simbricks, ts, msg->readcomp.req_id,
                               (void *)msg->readcomp.data);
      break;
    case SIMBRICKS_PROTO_MEM_M2H_MSG_WRITECOMP:
      /* we treat writes as posted, so nothing we need to do here */
      break;
    default:
      panic("simbricks_comm_poll_m2h: unhandled type");
  }

  SimbricksMemIfM2HInDone(&simbricks->memif, msg);
}

static void simbricks_timer_dummy(void *data) {
}

static void simbricks_timer_poll(void *data) {
  SimbricksMemState *simbricks = data;
  volatile union SimbricksProtoMemM2H *msg;
  volatile union SimbricksProtoMemM2H *next_msg;
  int64_t cur_ts, next_ts, proto_ts;

  cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
  proto_ts = ts_to_proto(
      simbricks,
      cur_ts +
          1); /* +1 to avoid getting stuck on off by ones due to rounding */
#ifdef DEBUG_PRINTS_VERBOSE
  if (verbose_debug_prints) {
    uint64_t poll_ts = SimbricksMemIfM2HInTimestamp(&simbricks->memif);
    if (proto_ts > poll_ts + 1 || proto_ts < poll_ts) {
      warn_report("simbricks_timer_poll: expected_pts=%lu cur_pts=%lu", poll_ts,
                  proto_ts);
    }
    warn_report("simbricks_timer_poll: ts=%ld sync_ts=%ld", cur_ts,
                timer_expire_time_ns(simbricks->timer_sync));
  }
#endif

  /* poll until we have a message (should not usually spin) */
  do {
    msg = SimbricksMemIfM2HInPoll(&simbricks->memif, proto_ts);
  } while (msg == NULL);

  /* wait for next message so we know its timestamp and when to schedule the
   * timer. */
  do {
    next_msg = SimbricksMemIfM2HInPeek(&simbricks->memif, proto_ts);
    next_ts = SimbricksMemIfM2HInTimestamp(&simbricks->memif);
  } while (!next_msg && next_ts <= proto_ts);

  /* set timer for next message */
  /* we need to do this before actually processing the message, in order to
   * have a timer set to prevent the clock from running away from us. We set a
   * dummy timer with the current ts to prevent the clock from jumping */
  timer_mod_ns(simbricks->timer_dummy, cur_ts);
  timer_mod_ns(simbricks->timer_poll, ts_from_proto(simbricks, next_ts));
  if (simbricks->sync_ts_bumped) {
    timer_mod_ns(simbricks->timer_sync,
                 ts_from_proto(simbricks, SimbricksBaseIfOutNextSync(
                                              &simbricks->memif.base)));
    simbricks->sync_ts_bumped = false;
  }

  /* now process the message */
  simbricks_comm_m2h_process(simbricks, cur_ts, msg);

#ifdef DEBUG_PRINTS_VERBOSE
  if (verbose_debug_prints) {
    int64_t now_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    if (cur_ts != now_ts) {
      warn_report("simbricks_timer_poll: time advanced from %lu to %lu", cur_ts,
                  now_ts);
    }

    warn_report("simbricks_timer_poll: done, next=%ld", next_ts);
  }
#endif
}

static void simbricks_timer_sync(void *data) {
  SimbricksMemState *simbricks = data;
  int64_t cur_ts;
  uint64_t proto_ts;

  cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
  proto_ts = ts_to_proto(simbricks, cur_ts);

#ifdef DEBUG_PRINTS_VERBOSE
  if (verbose_debug_prints) {
    uint64_t sync_ts = SimbricksMemIfH2MOutNextSync(&simbricks->memif);
    if (proto_ts > sync_ts + 1) {
      warn_report("simbricks_timer_sync: expected_ts=%lu cur_ts=%lu", sync_ts,
                  proto_ts);
    }

    warn_report("simbricks_timer_sync: ts=%lu pts=%lu npts=%lu", cur_ts,
                proto_ts, sync_ts);
  }
#endif

  while (SimbricksMemIfH2MOutSync(&simbricks->memif, proto_ts))
    ;

#ifdef DEBUG_PRINTS_VERBOSE
  if (verbose_debug_prints) {
    int64_t now_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    if (cur_ts != now_ts) {
      warn_report("simbricks_timer_poll: time advanced from %lu to %lu", cur_ts,
                  now_ts);
    }
  }
#endif
  uint64_t next_sync_pts = SimbricksMemIfH2MOutNextSync(&simbricks->memif);
  uint64_t next_sync_ts = ts_from_proto(simbricks, next_sync_pts);
#ifdef DEBUG_PRINTS_VERBOSE
  if (verbose_debug_prints) {
    warn_report("simbricks_timer_sync: next pts=%lu ts=%lu", next_sync_pts,
                next_sync_ts);
  }
#endif
  timer_mod_ns(simbricks->timer_sync, next_sync_ts);
}

static void *simbricks_poll_thread(void *opaque) {
  SimbricksMemState *simbricks = opaque;
  volatile union SimbricksProtoMemM2H *msg;

  assert(!simbricks->sync);

  while (!simbricks->stopping) {
    msg = SimbricksMemIfM2HInPoll(&simbricks->memif, 0);
    if (!msg)
      continue;

    /* actually process the operation. this needs to be done with the I/O
     * lock held. */
    qemu_mutex_lock_iothread();
    simbricks_comm_m2h_process(simbricks, 0, msg);
    qemu_mutex_unlock_iothread();
  }

  return NULL;
}

/******************************************************************************/
/* MMIO interface */

static bool logging_active = false;

/* submit a read or write to the worker thread and wait for it to complete */
static void simbricks_mmio_rw(SimbricksMemState *simbricks, hwaddr addr,
                              unsigned size, uint64_t *val, bool is_write) {
  CPUState *cpu = current_cpu;
  SimbricksMemRequest *req;
  volatile union SimbricksProtoMemH2M *msg;
  volatile struct SimbricksProtoMemH2MRead *read;
  volatile struct SimbricksProtoMemH2MWrite *write;
  int64_t cur_ts;

  assert(simbricks->reqs_len > cpu->cpu_index);
  req = simbricks->reqs + cpu->cpu_index;
  assert(req->cpu == cpu);

  cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);

#ifdef DEBUG_PRINTS
  verbose_debug_prints = true;
#endif

  if (req->requested) {
    /* a request from this CPU has been started */
    /* note that address might not match if an interrupt occurs, which in
     * turn triggers another read. */
    if (req->processing) {
      /* request in progress, we have to wait */
      cpu->stopped = 1;
      cpu_loop_exit(cpu);
    } else if (req->addr == addr && req->size == size) {
/* request finished */
#ifdef DEBUG_PRINTS
      warn_report("simbricks_mmio_rw: done (%lu) addr=0x%lx size=%u val=0x%lx",
                  cur_ts, addr, size, req->value);
#endif
      *val = req->value;
      req->requested = false;
      return;
    } else {
      /* Request is done processing, but for a different address. Cache the
      result because the CPU is probably retrying an instruction that was split
      into multiple ones.*/
      req->requested = false;
      req->cached_addr = req->addr;
      req->cached_size = req->size;
      req->cached_value = req->value;
#ifdef DEBUG_PRINTS
      warn_report(
          "simbricks_mmio_rw: caching (%lu) req_addr=0x%lx req_size=%u "
          "cached_addr=0x%lx cached_size=%u cached_val=0x%lx",
          cur_ts, addr, size, req->cached_addr, req->cached_size,
          req->cached_value);
#endif
    }
  }

  assert(!req->processing);

  /* prepare operation */
  if (is_write) {
    /* clear cache */
    req->cached_addr = 0;
    req->cached_size = 0;

    msg = simbricks_comm_h2m_alloc(
        simbricks, cur_ts); /* allocate host-to-device queue entry */
    write = &msg->write;

    write->req_id = cpu->cpu_index;
    write->addr = addr;
    write->len = size;

    assert(size <=
           SimbricksMemIfH2MOutMsgLen(&simbricks->memif) - sizeof(*write));
    /* FIXME: this probably only works for LE */
    memcpy((void *)write->data, val, size);

    SimbricksMemIfH2MOutSend(&simbricks->memif, msg,
                             SIMBRICKS_PROTO_MEM_H2M_MSG_WRITE);

#ifdef DEBUG_PRINTS
    warn_report(
        "simbricks_mmio_rw: finished write (%lu) addr=0x%lx size=%u "
        "val=0x%lx",
        cur_ts, addr, size, *val);
#endif

    /* we treat writes as posted and don't wait for completion */
    return;
  }

  /* handle read request */
  if (req->cached_addr == addr && req->cached_size == size) {
    /* request handled by cache */
    *val = req->cached_value;
    req->cached_addr = 0;
    req->cached_size = 0;

#ifdef DEBUG_PRINTS
    warn_report(
        "simbricks_mmio_rw: done from cache (%lu) addr=0x%lx size=%u "
        "val=0x%lx",
        cur_ts, addr, size, req->cached_value);
#endif
    return;
  }

  msg = simbricks_comm_h2m_alloc(
      simbricks, cur_ts); /* allocate host-to-device queue entry */
  read = &msg->read;

  read->req_id = cpu->cpu_index;
  read->addr = addr;
  read->len = size;

  SimbricksMemIfH2MOutSend(&simbricks->memif, msg,
                           SIMBRICKS_PROTO_MEM_H2M_MSG_READ);

  /* start processing request */
  req->processing = true;
  req->requested = true;

  req->addr = addr;
  req->size = size;

#ifdef DEBUG_PRINTS
  warn_report(
      "simbricks_mmio_rw: starting wait for read (%lu) addr=0x%lx "
      "size=%u",
      cur_ts, addr, size);
#endif

  if (simbricks->sync) {
    cpu->stopped = 1;
    cpu_loop_exit(cpu);
  } else {
    while (req->processing)
      qemu_cond_wait_iothread(&req->cond);

    *val = req->value;
    req->requested = false;
  }
}

static uint64_t simbricks_mmio_read(void *opaque, hwaddr addr, unsigned size) {
  SimbricksMemState *simbricks = SIMBRICKS_MEM(opaque);
  uint64_t ret = 0;

  simbricks_mmio_rw(simbricks, addr, size, &ret, false);

  return ret;
}

static void simbricks_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size) {
  SimbricksMemState *simbricks = SIMBRICKS_MEM(opaque);
  simbricks_mmio_rw(simbricks, addr, size, &val, true);
}

static const MemoryRegionOps simbricks_mmio_ops = {
    .read = simbricks_mmio_read,
    .write = simbricks_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.max_access_size = 64,
    .impl.max_access_size = 64,
    .valid.unaligned = true,
    .impl.unaligned = true};

/******************************************************************************/
/* Initialization */

static int simbricks_connect(SimbricksMemState *simbricks, Error **errp) {
  struct SimbricksProtoMemMemIntro *d_i = &simbricks->dev_intro;
  struct SimbricksProtoMemHostIntro host_intro;
  struct SimbricksBaseIfParams params;
  size_t len;
  CPUState *cpu;
  uint64_t first_sync_ts = 0, first_msg_ts = 0;
  volatile union SimbricksProtoMemM2H *msg;
  struct SimbricksBaseIf *base_if = &simbricks->memif.base;

  if (!simbricks->socket_path) {
    error_setg(errp, "socket path not set but required");
    return 0;
  }

  if (!simbricks->base_address) {
    error_setg(errp, "base_address not set but required");
    return 0;
  }

  if (!simbricks->size) {
    error_setg(errp, "size not set but required");
    return 0;
  }

  SimbricksMemIfDefaultParams(&params);
  params.link_latency = simbricks->mem_latency * 1000;
  params.sync_interval = simbricks->sync_period * 1000;
  params.blocking_conn = true;
  params.sock_path = simbricks->socket_path;
  params.sync_mode = (simbricks->sync ? kSimbricksBaseIfSyncRequired
                                      : kSimbricksBaseIfSyncDisabled);

  if (SimbricksBaseIfInit(base_if, &params)) {
    error_setg(errp, "SimbricksBaseIfInit failed");
    return 0;
  }

  if (SimbricksBaseIfConnect(base_if)) {
    error_setg(errp, "SimbricksBaseIfConnect failed");
    return 0;
  }

  if (SimbricksBaseIfConnected(base_if)) {
    error_setg(errp, "SimbricksBaseIfConnected indicates unconnected");
    return 0;
  }

  /* prepare & send host intro */
  memset(&host_intro, 0, sizeof(host_intro));
  if (SimbricksBaseIfIntroSend(base_if, &host_intro, sizeof(host_intro))) {
    error_setg(errp, "SimbricksBaseIfIntroSend failed");
    return 0;
  }

  /* receive device intro */
  len = sizeof(*d_i);
  if (SimbricksBaseIfIntroRecv(base_if, d_i, &len)) {
    error_setg(errp, "SimbricksBaseIfIntroRecv failed");
    return 0;
  }
  if (len != sizeof(*d_i)) {
    error_setg(errp, "rx dev intro: length is not as expected");
    return 0;
  }

  if (simbricks->sync) {
    /* send a first sync */
    if (SimbricksMemIfH2MOutSync(&simbricks->memif, 0)) {
      error_setg(errp, "sending initial sync failed");
      return 0;
    }
    first_sync_ts = SimbricksMemIfH2MOutNextSync(&simbricks->memif);

    /* wait for first message so we know its timestamp */
    do {
      msg = SimbricksMemIfM2HInPeek(&simbricks->memif, 0);
      first_msg_ts = SimbricksMemIfM2HInTimestamp(&simbricks->memif);
    } while (!msg && !first_msg_ts);
  }
  simbricks->reqs_len = 0;
  CPU_FOREACH(cpu) {
    simbricks->reqs_len++;
  }
  simbricks->reqs = calloc(simbricks->reqs_len, sizeof(*simbricks->reqs));
  CPU_FOREACH(cpu) {
    simbricks->reqs[cpu->cpu_index].cpu = cpu;
    qemu_cond_init(&simbricks->reqs[cpu->cpu_index].cond);
  }

  if (simbricks->sync) {
    simbricks->timer_dummy =
        timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_dummy, simbricks);

    simbricks->ts_base = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    simbricks->timer_sync =
        timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_sync, simbricks);
    timer_mod_ns(simbricks->timer_sync,
                 ts_from_proto(simbricks, first_sync_ts));
    simbricks->timer_poll =
        timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_poll, simbricks);
    timer_mod_ns(simbricks->timer_poll, ts_from_proto(simbricks, first_msg_ts));
  } else {
    qemu_thread_create(&simbricks->thread, "simbricks-poll",
                       simbricks_poll_thread, simbricks, QEMU_THREAD_JOINABLE);
  }

  return 1;
}

static void mem_simbricks_realize(DeviceState *ds, Error **errp) {
  SimbricksMemState *simbricks = SIMBRICKS_MEM(ds);

  if (!simbricks_connect(simbricks, errp)) {
    return;
  }

  memory_region_init_io(&simbricks->mr, OBJECT(simbricks), &simbricks_mmio_ops,
                        simbricks, TYPE_MEM_SIMBRICKS_DEVICE, simbricks->size);
  sysbus_init_mmio(SYS_BUS_DEVICE(ds), &simbricks->mr);
  sysbus_mmio_map(SYS_BUS_DEVICE(ds), 0, simbricks->base_address);
}

static void mem_simbricks_unrealize(DeviceState *ds) {
  SimbricksMemState *simbricks = SIMBRICKS_MEM(ds);
  CPUState *cpu;

  if (!simbricks->sync) {
    simbricks->stopping = true;
    qemu_thread_join(&simbricks->thread);
  }

  CPU_FOREACH(cpu) {
    qemu_cond_destroy(&simbricks->reqs[cpu->cpu_index].cond);
  }
  free(simbricks->reqs);

  if (simbricks->sync) {
    timer_del(simbricks->timer_dummy);
    timer_free(simbricks->timer_dummy);
    timer_del(simbricks->timer_sync);
    timer_free(simbricks->timer_sync);
    timer_del(simbricks->timer_poll);
    timer_free(simbricks->timer_poll);
  }

  SimbricksBaseIfClose(&simbricks->memif.base);
}

static Property simbricks_mem_dev_properties[] = {
    DEFINE_PROP_STRING("socket", SimbricksMemState, socket_path),
    DEFINE_PROP_BOOL("sync", SimbricksMemState, sync, false),
    DEFINE_PROP_INT32("sync-mode", SimbricksMemState, sync_mode,
                      SIMBRICKS_PROTO_SYNC_SIMBRICKS),
    DEFINE_PROP_UINT64("mem-latency", SimbricksMemState, mem_latency, 500),
    DEFINE_PROP_UINT64("sync-period", SimbricksMemState, sync_period, 500),
    DEFINE_PROP_UINT64("base-address", SimbricksMemState, base_address, 0),
    DEFINE_PROP_UINT64("size", SimbricksMemState, size, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void simbricks_mem_class_init(ObjectClass *class, void *data) {
  DeviceClass *dc = DEVICE_CLASS(class);

  device_class_set_props(dc, simbricks_mem_dev_properties);

  dc->realize = mem_simbricks_realize;
  dc->unrealize = mem_simbricks_unrealize;
  dc->hotpluggable = true;
  dc->user_creatable = true;

  dc->desc = "SimBricks Memory adapter";
}

static void mem_simbricks_register_types(void) {
  static const TypeInfo simbricks_info = {
      .name = TYPE_MEM_SIMBRICKS_DEVICE,
      .parent = TYPE_SYS_BUS_DEVICE,
      .instance_size = sizeof(SimbricksMemState),
      .class_init = simbricks_mem_class_init,
  };

  type_register_static(&simbricks_info);
}
type_init(mem_simbricks_register_types)
