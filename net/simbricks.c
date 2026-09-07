#include "qemu/osdep.h"
#include "net/net.h"
#include "net/clients.h"
#include "qapi/error.h"
#include "qapi/qapi-types-net.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qemu/error-report.h"

#include <simbricks/network/proto.h>
#include <simbricks/network/if.h>
#include <simbricks/parser/parser.h>

// #define SIMBRICKS_ETH_DEBUG

#define SIMBRICKS_CLOCK QEMU_CLOCK_VIRTUAL

typedef struct SimbricksEthState
{
    NetClientState nc;
    struct SimbricksBaseIfSHMPool pool;
    struct SimbricksNetIf netif;
    bool sync;
    int64_t ts_base;
    QEMUTimer *poll_timer;
    QEMUTimer *sync_timer;
    QEMUTimer *timer_dummy;
} SimbricksEthState;

static inline uint64_t ts_to_proto(SimbricksEthState *simbricks,
                                   int64_t qemu_ts)
{
    return (qemu_ts - simbricks->ts_base) * 1000;
}

static inline int64_t ts_from_proto(SimbricksEthState *simbricks,
                                    uint64_t proto_ts)
{
    return (proto_ts / 1000) + simbricks->ts_base;
}

static void simbricks_timer_sync_cb(void *opaque)
{
    SimbricksEthState *simbricks = opaque;

    int64_t cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    uint64_t proto_ts = ts_to_proto(simbricks, cur_ts);

    while (SimbricksNetIfOutSync(&simbricks->netif, proto_ts))
        ;

    uint64_t next_sync_pts = SimbricksNetIfOutNextSync(&simbricks->netif);
    uint64_t next_sync_ts = ts_from_proto(simbricks, next_sync_pts);

    timer_mod_ns(simbricks->sync_timer, next_sync_ts);
}

/* 
 * QEMU -> Network
 * Called when the Guest NIC (within QEMU) transmits a packet.
 */
static ssize_t simbricks_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    SimbricksEthState *simbricks = DO_UPCAST(SimbricksEthState, nc, nc);

    int64_t cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    uint64_t proto_ts = ts_to_proto(simbricks, cur_ts);

    /* Allocate space in the SimBricks TX queue */
    volatile union SimbricksProtoNetMsg *msg = SimbricksNetIfOutAlloc(&simbricks->netif, proto_ts);
    if (!msg)
    {
        warn_report("simbricks-eth simbricks_receive: could not allocate messsage");
        return 0;
    }

    /* Copy payload into SimBricks shared memory */
    volatile struct SimbricksProtoNetMsgPacket *pkt = &msg->packet;
    pkt->len = size;
    pkt->port = 0;
    memcpy((void *)pkt->data, buf, size);

    /* Commit the packet to the queue */
    SimbricksNetIfOutSend(&simbricks->netif, msg, SIMBRICKS_PROTO_NET_MSG_PACKET);

    if (simbricks->sync) {
        // re-schedule sync timer if we send out a message
        timer_mod_ns(simbricks->sync_timer, ts_from_proto(simbricks, SimbricksNetIfOutNextSync(&simbricks->netif)));
    }

    return size;
}

static void handle_and_free_n2q_msg(SimbricksEthState *simbricks, volatile union SimbricksProtoNetMsg *msg)
{
    if (!msg)
    {
        return;
    }

    uint8_t type = SimbricksNetIfInType(&simbricks->netif, msg);

    switch (type) {
        case SIMBRICKS_PROTO_MSG_TYPE_SYNC:
            /* nop */
            break;
        case SIMBRICKS_PROTO_NET_MSG_PACKET:
            volatile struct SimbricksProtoNetMsgPacket *pkt = &msg->packet;
            qemu_send_packet(&simbricks->nc, (const uint8_t *)pkt->data, pkt->len);
            break;
        default:
            fprintf(stderr, "simbricks-eth: handle_and_free_n2q_msg: unhandled type");
            return;
    }

    SimbricksNetIfInDone(&simbricks->netif, msg);
}

/* 
 * Network -> QEMU
 * Polling loop to pull packets from network into the Guest NIC (within QEMU).
 */
static void simbricks_poll(void *opaque)
{
    SimbricksEthState *simbricks = opaque;
    volatile union SimbricksProtoNetMsg *msg = NULL;
    volatile union SimbricksProtoNetMsg *next_msg = NULL;
    int64_t cur_ts, next_ts, proto_ts;

    cur_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    proto_ts = ts_to_proto(simbricks, cur_ts + 1); /* + 1 to avoid rounding lockups */

    /* ==========================================
     * UNSYNCHRONIZED MODE
     * ========================================== */
    if (!simbricks->sync)
    {
        /* Drain the queue without spinning/blocking */
        while ((msg = SimbricksNetIfInPoll(&simbricks->netif, proto_ts)) != NULL)
        {
            handle_and_free_n2q_msg(simbricks, msg);
        }

        timer_mod_ns(simbricks->poll_timer, cur_ts + 100000);
        return;
    }

    /* ==========================================
     * SYNCHRONIZED MODE
     * ========================================== */

#ifdef SIMBRICKS_ETH_DEBUG
    fprintf(stderr, "simbricks-eth: [poll] Entering wait for msg at ts %ld...\n", cur_ts);
#endif
    while (msg == NULL)
    {

        msg = SimbricksNetIfInPoll(&simbricks->netif, proto_ts);
    }
#ifdef SIMBRICKS_ETH_DEBUG
    fprintf(stderr, "simbricks-eth: [poll] Got msg! Entering wait for next_msg...\n");
#endif

    /* wait for next message so we know its timestamp and when to schedule the timer. */
    do
    {
        next_msg = SimbricksNetIfInPeek(&simbricks->netif, proto_ts);
        next_ts = SimbricksNetIfInTimestamp(&simbricks->netif);
    } while (!next_msg && next_ts <= proto_ts);

#ifdef SIMBRICKS_ETH_DEBUG
    fprintf(stderr, "simbricks-eth: [poll] Wait complete! Rescheduling...\n");
#endif

    timer_mod_ns(simbricks->timer_dummy, cur_ts);
    /* set timer for next message */
    timer_mod_ns(simbricks->poll_timer, ts_from_proto(simbricks, next_ts));

    /* now process the message */
    handle_and_free_n2q_msg(simbricks, msg);

#ifdef SIMBRICKS_ETH_DEBUG
    int64_t now_ts = qemu_clock_get_ns(SIMBRICKS_CLOCK);
    if (cur_ts != now_ts)
        fprintf(stderr, "\n\n\nsimbricks_timer_poll: time advanced from %lu to %lu\n\n\n",
                    cur_ts, now_ts);
#endif

    return;
}

/* Cleanup routine when QEMU shuts down */
static void simbricks_cleanup(NetClientState *nc)
{
    SimbricksEthState *simbricks = DO_UPCAST(SimbricksEthState, nc, nc);

    if (simbricks->poll_timer)
    {
        timer_free(simbricks->poll_timer);
    }
    if (simbricks->sync_timer)
    {
        timer_free(simbricks->sync_timer);
    }
    if (simbricks->timer_dummy)
    {
        timer_free(simbricks->timer_dummy);
    }
}

static void simbricks_timer_dummy(void *opaque) {}

/* Define the NetClientInfo interface */
static NetClientInfo net_simbricks_info = {
    .type = NET_CLIENT_DRIVER_SIMBRICKS_ETH,
    .size = sizeof(SimbricksEthState),
    .receive = simbricks_receive,
    .cleanup = simbricks_cleanup,
};

/* 
 * Initialization Routine
 * Called when parsing `-netdev simbricks-eth`
 */
int net_init_simbricks(const Netdev *netdev, const char *name,
                       NetClientState *peer, Error **errp)
{
    const NetdevSimbricksOptions *options = &netdev->u.simbricks_eth;
    NetClientState *nc;
    SimbricksEthState *simbricks;

    uint64_t first_sync_ts = 0, first_msg_ts = 0;
    volatile union SimbricksProtoNetMsg *msg;

    if (!options->sock_path)
    {
        error_setg(errp, "simbricks: 'sock-path' parameter string is required");
        return -1;
    }

    nc = qemu_new_net_client(&net_simbricks_info, peer, "simbricks-eth", name);
    simbricks = DO_UPCAST(SimbricksEthState, nc, nc);
    
    /* Initialize the SimBricks Interface */
    struct SimBricksBaseIfEstablishData est;
    struct SimbricksProtoNetIntro intro_msg;
    memset(&intro_msg, 0, sizeof(intro_msg));
    est.base_if = &simbricks->netif.base;
    est.rx_intro = &intro_msg;
    est.rx_intro_len = sizeof(intro_msg);
    est.tx_intro = &intro_msg;
    est.tx_intro_len = sizeof(intro_msg);

    SimbricksNetIfDefaultParams(&simbricks->netif.base.params);
    if (SimbricksParametersEstablish(&est, &(options->sock_path), 1, &(simbricks->pool), NULL))
    {
        error_setg(errp, "simbricks: Failed to initialize interface at %s", options->sock_path);
        qemu_del_net_client(nc);
        return -1;
    }

    simbricks->sync = simbricks->netif.base.sync;

    if (simbricks->sync)
    {
        /* send a first sync */
        if (SimbricksNetIfOutSync(&simbricks->netif, 0))
        {
            error_setg(errp, "sending initial sync failed");
            qemu_del_net_client(nc);
            return -1;
        }
        first_sync_ts = SimbricksNetIfOutNextSync(&simbricks->netif);

        /* wait for first message so we know its timestamp */
        do
        {
            msg = SimbricksNetIfInPeek(&simbricks->netif, 0);
            first_msg_ts = SimbricksNetIfInTimestamp(&simbricks->netif);
        } while (!msg && !first_msg_ts);
        
        /* Set up the virtual timer for polling */
        simbricks->ts_base = qemu_clock_get_ns(SIMBRICKS_CLOCK);

        simbricks->timer_dummy =
            timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_dummy, simbricks);
    
        simbricks->sync_timer = timer_new_ns(SIMBRICKS_CLOCK, simbricks_timer_sync_cb, simbricks);
        timer_mod_ns(simbricks->sync_timer, ts_from_proto(simbricks, first_sync_ts));
    
        simbricks->poll_timer = timer_new_ns(SIMBRICKS_CLOCK, simbricks_poll, simbricks);
        timer_mod_ns(simbricks->poll_timer, ts_from_proto(simbricks, first_msg_ts));

    } else {
        simbricks->poll_timer = timer_new_ns(SIMBRICKS_CLOCK, simbricks_poll, simbricks);
        timer_mod_ns(simbricks->poll_timer, ts_from_proto(simbricks, first_msg_ts));
    }

    return 0;
}