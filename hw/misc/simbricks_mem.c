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

#define DEBUG_PRINTS
// #define DEBUG_PRINTS_VERBOSE

#ifdef DEBUG_PRINTS_VERBOSE
bool verbose_debug_prints = false; /* skip verbose prints during startup */
#endif

#define SIMBRICKS_CLOCK QEMU_CLOCK_VIRTUAL

#define SIMBRICKS_MEM(obj) \
  OBJECT_CHECK(SimbricksMemState, obj, TYPE_MEM_SIMBRICKS_DEVICE)

struct SimbricksMemRequest;
typedef struct CacheEntry {
  hwaddr addr;
  struct SimbricksMemRequest *waiters;
  uint64_t last_access;
  bool valid;
  bool requested;
  uint16_t lock;
  uint8_t data[];
} CacheEntry;

typedef struct SimbricksMemRequest {
  CPUState *cpu; /* CPU associated with this request */
  QemuCond cond;
  bool pending;
  CacheEntry *ce;
  int32_t old_exception_index;
  struct SimbricksMemRequest *next_waiter;
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
  uint64_t cache_size;
  uint64_t cache_line_size;

  MemoryRegion mr;

  struct SimbricksMemIf memif;
  struct SimbricksProtoMemMemIntro dev_intro;

  bool sync_ts_bumped;

  /* communication bewteen main io thread and worker thread
   * (protected by thr_mutex). */
  size_t reqs_len;
  SimbricksMemRequest *reqs;
  CacheEntry **cache;

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

/* Hack: Qemu doesn't allow stalling memory operations so instead we abort the
 * CPU's current instruction. This works because the clock will keep
 * advancing. */
static inline void suspend_cpu(SimbricksMemState *simbricks, CPUState *cpu) {
  SimbricksMemRequest *our_mr = simbricks->reqs + cpu->cpu_index;
  cpu->stopped = 1;
  our_mr->old_exception_index = cpu->exception_index;
  cpu->exception_index = 0x10000; /* prevent Qemu from handling exceptions,
                                     which will cause an infinite loop */
  cpu_loop_exit(cpu);
}

static inline void resume_cpu(SimbricksMemState *simbricks, CPUState *cpu) {
  SimbricksMemRequest *our_mr = simbricks->reqs + cpu->cpu_index;
  cpu->stopped = 0;
  cpu->exception_index = our_mr->old_exception_index;
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
  SimbricksMemRequest *mr, *next_mr;
  CacheEntry *ce;
  CPUState *cpu;

  assert(req_id <= simbricks->reqs_len);

  if (!req->pending) {
    panic("simbricks_comm_m2h_rcomp: no request currently processing");
  }

  assert(req->ce != NULL);
  ce = req->ce;
  assert(!ce->valid);
  assert(ce->requested);

#ifdef DEBUG_PRINTS
  warn_report("simbricks-mem: simbricks_comm_m2h_rcomp (%lu) %p completed",
              cur_ts, ce);
#endif
  /* copy read value from message */
  memcpy(&ce->data, data, simbricks->cache_line_size);
  ce->requested = false;
  ce->valid = true;
  mr = ce->waiters;
  ce->waiters = NULL;

  for (; mr != NULL; mr = next_mr) {
    next_mr = mr->next_waiter;
    mr->next_waiter = NULL;
    mr->pending = false;

    if (simbricks->sync) {
      cpu = mr->cpu;

  #ifdef DEBUG_PRINTS
      warn_report("simbricks_comm_m2h_rcomp: kicking cpu %lu ts=%lu", req_id,
                  cur_ts);
  #endif

      resume_cpu(simbricks, cpu);
    } else {
      qemu_cond_broadcast(&mr->cond);
    }
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
      panic(
          "simbricks_comm_m2h_process: writes are treated as posted, so there "
          "shouldn't be a write completion message.");

      break;
    default:
      panic("simbricks_comm_m2h_process: unhandled type");
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
/* Cache management */

static CacheEntry *cache_entry_get(SimbricksMemState *simbricks, hwaddr addr,
                                   bool alloc, uint64_t ts)
{
  hwaddr cl_num = addr / simbricks->cache_line_size;
  uint64_t key = cl_num;
  uint64_t i;
  CacheEntry *ce, *min_ce = NULL;

  addr &= ~((uint64_t) simbricks->cache_line_size - 1);

  // very simple hash mixer
  key ^= key >> 33;
  key *= 0xff51afd7ed558ccd;
  key ^= key >> 33;
  key *= 0xc4ceb9fe1a85ec53;
  key ^= key >> 33;

  for (i = 0; i < 8; i++) {
    ce = simbricks->cache[(key + i) % simbricks->cache_size];
    if (ce->addr == addr) {
#ifdef DEBUG_PRINTS
      warn_report("simbricks-mem: cache_entry_get (%lu) addr=0x%lx found %p l=%u",
              ts, addr, ce, ce->lock);
#endif
      ce->last_access = ts;
      ce->lock++;
      return ce;
    }

    if (!ce->waiters && !ce->lock) {
      if (min_ce == NULL || ce->last_access < min_ce->last_access)
        min_ce = ce;
    }
  }

  if (!alloc) {
#ifdef DEBUG_PRINTS
    warn_report("simbricks-mem: cache_entry_get (%lu) addr=0x%lx not found",
            ts, addr);
#endif
    return NULL;
  }

  if (!min_ce) {
    for (i = 0; i < 8; i++) {
      ce = simbricks->cache[(key + i) % simbricks->cache_size];
      warn_report("simbricks-mem: cache_entry_get (%lu) ci[%lu]=%p w=%p l=%u a=%lx",
            ts, i, ce, ce->waiters, ce->lock, ce->addr);
    }
    panic("found no cache entries to allocate");
  }

#ifdef DEBUG_PRINTS
  warn_report("simbricks-mem: cache_entry_get (%lu) addr=0x%lx allocated %p",
          ts, addr, min_ce);
#endif
  min_ce->last_access = ts;
  min_ce->valid = false;
  min_ce->requested = false;
  min_ce->addr = addr;
  min_ce->lock++;
  return min_ce;
}

static void cache_entry_release(SimbricksMemState *simbricks,
                                CacheEntry *ce, uint64_t cur_ts)
{
#ifdef DEBUG_PRINTS
  warn_report("simbricks-mem: cache_entry_release (%lu) %p l=%u",
              cur_ts, ce, ce->lock);
#endif
  ce->lock--;
}

static int cache_entry_ensure_valid(SimbricksMemState *simbricks,
                                    CacheEntry *ce,
                                    uint64_t cur_ts)
{
  CPUState *cpu = current_cpu;
  SimbricksMemRequest *our_mr, *mr;

  if (ce->valid)
    return 0;

  assert(simbricks->reqs_len > cpu->cpu_index);
  our_mr = simbricks->reqs + cpu->cpu_index;
  assert(our_mr->cpu == cpu);

  /* mem request slot for this CPU is busy */
  if (our_mr->pending) {
#ifdef DEBUG_PRINTS
    warn_report("simbricks-mem: cache_entry_ensure_valid (%lu) %p CPU pending",
                cur_ts, ce);
#endif
    return 1;
  }

  our_mr->pending = true;
  our_mr->ce = ce;

  if (!ce->requested) {
    /* need to issue a request for this cache line */
#ifdef DEBUG_PRINTS
    warn_report("simbricks-mem: cache_entry_ensure_valid (%lu) %p issuing req l=%u",
                cur_ts, ce, ce->lock);
#endif
    volatile union SimbricksProtoMemH2M *msg;
    volatile struct SimbricksProtoMemH2MRead *read;

    msg = simbricks_comm_h2m_alloc(
        simbricks, cur_ts); /* allocate host-to-device queue entry */
    read = &msg->read;

    read->req_id = cpu->cpu_index;
    read->addr = ce->addr;
    read->len = simbricks->cache_line_size;

    SimbricksMemIfH2MOutSend(&simbricks->memif, msg,
                            SIMBRICKS_PROTO_MEM_H2M_MSG_READ);

    ce->requested = true;

    /* append mr to CE's waiters list (which is empty at this point) */
    our_mr->ce = ce;
    assert(ce->waiters == NULL);
    our_mr->next_waiter = NULL;
    ce->waiters = our_mr;
  } else {
    /* otherwise just add ourselves to waiting list */
#ifdef DEBUG_PRINTS
    warn_report("simbricks-mem: cache_entry_ensure_valid (%lu) %p waiting l=%u",
                cur_ts, ce, ce->lock);
#endif
    assert(ce->waiters != NULL);
    for (mr = ce->waiters; mr->next_waiter != NULL; mr = mr->next_waiter);
    our_mr->next_waiter = NULL;
    mr->next_waiter = our_mr;
  }

  /* wait for result */
  if (simbricks->sync) {
#ifdef DEBUG_PRINTS
    warn_report("simbricks-mem: cache_entry_ensure_valid (%lu) %p suspending l=%u",
                cur_ts, ce, ce->lock);
#endif
    cache_entry_release(simbricks, ce, cur_ts);
    suspend_cpu(simbricks, cpu);
    panic("suspend_cpu returned");
    return 1;
  } else {
    /* in un-synchronized mode we just block until completion*/
#ifdef DEBUG_PRINTS
    warn_report("simbricks-mem: cache_entry_ensure_valid (%lu) %p blocking l=%u",
                cur_ts, ce, ce->lock);
#endif
    while (!our_mr->pending)
      qemu_cond_wait_iothread(&our_mr->cond);
#ifdef DEBUG_PRINTS
    warn_report("simbricks-mem: cache_entry_ensure_valid (%lu) %p ready (l=%u)",
                cur_ts, ce, ce->lock);
#endif
    assert(ce->valid);
    return 0;
  }
}


static void issue_write(SimbricksMemState *simbricks, hwaddr addr, uint64_t val,
                        unsigned size, uint64_t cur_ts)
{
  CPUState *cpu = current_cpu;
  volatile union SimbricksProtoMemH2M *msg;
  volatile struct SimbricksProtoMemH2MWrite *write;

#ifdef DEBUG_PRINTS
  warn_report("simbricks-mem: issue_write (%lu) addr=0x%lx size=%u val=0x%lx",
              cur_ts, addr, size, val);
#endif

  msg = simbricks_comm_h2m_alloc(simbricks, cur_ts);
  write = &msg->write;

  write->req_id = cpu->cpu_index;
  write->addr = addr;
  write->len = size;

  assert(size <=
          SimbricksMemIfH2MOutMsgLen(&simbricks->memif) - sizeof(*write));
  /* FIXME: this probably only works for LE */
  memcpy((void *)write->data, &val, size);

  SimbricksMemIfH2MOutSend(&simbricks->memif, msg,
                            SIMBRICKS_PROTO_MEM_H2M_MSG_WRITE_POSTED);
}

/******************************************************************************/
/* MMIO interface */

#if 0
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

#ifdef DEBUG_PRINTS_VERBOSE
  verbose_debug_prints = true;
#endif

  if (req->requested) {
    /* a request from this CPU has been started */
    /* note that address might not match if an interrupt occurs, which in
     * turn triggers another read. */
    if (req->processing) {
      /* request in progress, we have to wait */
      suspend_cpu(cpu);
    } else if (req->addr == addr && req->size == size) {
      /* request finished */
      *val = req->value;
      req->requested = false;
#ifdef DEBUG_PRINTS
      warn_report("simbricks_mmio_rw: done (%lu) addr=0x%lx size=%u val=0x%lx",
                  cur_ts, addr, size, *val);
#endif
      return;
    } else {
      /* Request is done processing, but for a different address.*/
      req->requested = false;
#ifdef DEBUG_PRINTS
      warn_report(
          "simbricks_mmio_rw: done, but received different request (%lu) "
          "addr=0x%lx size=%u is_write=%u",
          cur_ts, addr, size, is_write);
#endif
    }
  }

  assert(!req->processing);

  /* handle write request */
  if (is_write) {
    fifo8_reset(&req->read_cache); /* clear read cache */

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
                             SIMBRICKS_PROTO_MEM_H2M_MSG_WRITE_POSTED);

#ifdef DEBUG_PRINTS
    warn_report(
        "simbricks_mmio_rw: finished write (%lu) addr=0x%lx size=%u "
        "val=0x%lx",
        cur_ts, addr, size, *val);
#endif
    return;
  }

  /* Handle read request. First check whether we can already find the result in
   * the cache. This is necessary to make progress when handling faults in the
   * TLB, which cause multiple read operations to be replayed when we stop the
   * CPU.*/
  struct ReadCacheEntry *cached_entry =
      read_cache_get_entry(&req->read_cache, addr, size);
  if (cached_entry) {
    /* request handled by cache */
    *val = cached_entry->value;

#ifdef DEBUG_PRINTS
    warn_report(
        "simbricks_mmio_rw: read done from cache (%lu) addr=0x%lx size=%u "
        "val=0x%lx",
        cur_ts, addr, size, cached_entry->value);
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

  /* wait for result */
  if (simbricks->sync) {
    suspend_cpu(cpu);
  } else {
    while (req->processing)
      qemu_cond_wait_iothread(&req->cond);

    *val = req->value;
    req->requested = false;
  }
}
#endif

static uint64_t simbricks_mmio_read(void *opaque, hwaddr addr, unsigned size) {
  SimbricksMemState *simbricks = SIMBRICKS_MEM(opaque);
  CacheEntry *ce;
  uint64_t cache_line_off;
  uint64_t ret = 0;
  uint64_t cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);

#ifdef DEBUG_PRINTS
  warn_report("simbricks-mem: simbricks_mmio_read (%lu) addr=0x%lx size=%u",
          cur_ts, addr, size);
#endif

  if (addr / simbricks->cache_line_size !=
      (addr + size - 1) / simbricks->cache_line_size) {
    panic("Do not support split-cache-line reads");
  }

  ce = cache_entry_get(simbricks, addr, true, cur_ts);
  if (cache_entry_ensure_valid(simbricks, ce, cur_ts)) {
    // not ready yet, CPU suspended
#ifdef DEBUG_PRINTS
    warn_report("simbricks-mem: simbricks_mmio_read (%lu) not ready",
            cur_ts);
#endif
    cache_entry_release(simbricks, ce, cur_ts);
    return 0;
  }

  cache_line_off = addr % simbricks->cache_line_size;
  memcpy(&ret, ce->data + cache_line_off, size);
  cache_entry_release(simbricks, ce, cur_ts);

#ifdef DEBUG_PRINTS
  warn_report("simbricks-mem: simbricks_mmio_read (%lu) finished ret=%lx",
          cur_ts, ret);
#endif
  return ret;
}

static void simbricks_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size) {
  SimbricksMemState *simbricks = SIMBRICKS_MEM(opaque);
  CacheEntry *ce;
  uint64_t cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);

#ifdef DEBUG_PRINTS
  warn_report("simbricks-mem: simbricks_mmio_write (%lu) addr=0x%lx size=%u "
              "val=%lx", cur_ts, addr, size, val);
#endif

  if (addr / simbricks->cache_line_size !=
      (addr + size - 1) / simbricks->cache_line_size) {
    panic("Do not support split-cache-line writes");
  }

  // get cache entry, but do not allocate if none exists yet
  ce = cache_entry_get(simbricks, addr, false, cur_ts);
  if (ce) {
    // if there is one, we need to make sure to wait for it to be valid
    if (cache_entry_ensure_valid(simbricks, ce, cur_ts)) {
      // not ready yet, CPU suspended
      cache_entry_release(simbricks, ce, cur_ts);
#ifdef DEBUG_PRINTS
      warn_report("simbricks-mem: simbricks_mmio_write (%lu) suspend",
                  cur_ts);
#endif
      return;
    }

    hwaddr cache_line_off = addr % simbricks->cache_line_size;
    memcpy(ce->data + cache_line_off, &val, size);
    cache_entry_release(simbricks, ce, cur_ts);
  }

  issue_write(simbricks, addr, val, size, cur_ts);
#ifdef DEBUG_PRINTS
  warn_report("simbricks-mem: simbricks_mmio_write (%lu) completed",
              cur_ts);
#endif
}

static const MemoryRegionOps simbricks_mmio_ops = {
    .read = simbricks_mmio_read,
    .write = simbricks_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.max_access_size = 8,
    .impl.max_access_size = 8,
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

  simbricks->cache = calloc(simbricks->cache_size, sizeof(*simbricks->cache));
  uint64_t i;
  for (i = 0; i < simbricks->cache_size; i++) {
    simbricks->cache[i] = calloc(
        1, sizeof(CacheEntry) + simbricks->cache_line_size);
  }

  simbricks->reqs_len = 0;
  CPU_FOREACH(cpu) {
    simbricks->reqs_len++;
  }
  simbricks->reqs = calloc(simbricks->reqs_len, sizeof(*simbricks->reqs));
  if (simbricks->reqs_len > 1) {
    panic(
        "simbricks_mem: The read cache does not track the memory operations of "
        "other CPUs, which means read operations could yield wrong results");
  }
  CPU_FOREACH(cpu) {
    simbricks->reqs[cpu->cpu_index].pending = false;
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
  uint64_t i;
  for (i = 0; i < simbricks->cache_size; i++) {
    free(simbricks->cache[i]);
  }
  free(simbricks->cache);

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
    DEFINE_PROP_UINT64("cache-size", SimbricksMemState, cache_size, 1024),
    DEFINE_PROP_UINT64("cache-line-size", SimbricksMemState, cache_line_size,
                       64),
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
