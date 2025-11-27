/*
 * stp_sim.c
 *
 * Simplified STP / RSTP-like module for learning / prototyping.
 *
 * Build:
 *   gcc -O2 -o stp_sim stp_sim.c
 *
 * This code is not a full production stack. It's intended to show a clear,
 * implementable algorithm for STP core components and a simple simulation.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <time.h>

/* ---------- Basic Types ---------- */

typedef struct {
    uint16_t priority;   // 2 bytes
    uint8_t mac[6];      // 6 bytes
} BridgeID;

typedef enum { BETTER = -1, SAME = 0, WORSE = 1 } cmp_t;

/* BPDU flags bits (very simplified) */
#define BPDU_FLAG_TC        0x01  // topology change
#define BPDU_FLAG_PROPOSAL  0x02  // proposal in RSTP
#define BPDU_FLAG_AGREEMENT 0x04  // agreement in RSTP

typedef struct {
    BridgeID root_id;
    uint32_t root_path_cost;
    BridgeID sender_id;
    uint16_t sender_port_id;
    uint8_t flags;
    uint16_t msg_age;      // in sec
    uint16_t max_age;
    uint16_t hello_time;
    uint16_t forward_delay;
} BPDU;

/* Simple port & bridge structures */

#define MAX_PORTS 8
#define MAX_BRIDGES 8

typedef enum { ROOT_PORT, DESIGNATED_PORT, ALTERNATE_PORT, BACKUP_PORT } PortRole;
typedef enum { DISCARDING, LEARNING, FORWARDING } PortState;

typedef struct Port Port;
struct Port {
    uint16_t port_no;
    PortRole role;
    PortState state;

    uint32_t path_cost;               // cost of this port/link
    BPDU best_received;               // best BPDU seen on this port
    bool have_bpdu;                   // whether best_received is valid

    uint16_t msg_age_timer;           // increments each tick when we have BPDU
    uint16_t forward_delay_timer;     // counts down when entering forwarding path
    uint16_t hello_timer;             // hello interval timer for tx

    bool oper_edge;                   // edge port (no BPDUs expected)
    bool proposed;                    // whether we've sent proposal recently
    bool syncing;                     // used in RSTP handshake
    // Link simulation: pointer to remote port for local simulation only
    Port *remote;
    struct Bridge *parent;
};

/* Bridge structure */
typedef struct Bridge {
    BridgeID self_id;
    BridgeID root_id;
    uint32_t root_path_cost;

    uint16_t hello_time;
    uint16_t max_age;
    uint16_t forward_delay;

    Port ports[MAX_PORTS];
    int num_ports;

    // convenience: human name for logs
    char name[32];
} Bridge;

/* ---------- Utility functions ---------- */

static void print_bid(const BridgeID *b, char *buf, size_t sz) {
    snprintf(buf, sz, "%u:%02x%02x%02x%02x%02x%02x",
        b->priority,
        b->mac[0], b->mac[1], b->mac[2], b->mac[3], b->mac[4], b->mac[5]);
}

static int bridgeid_compare(const BridgeID *a, const BridgeID *b) {
    if (a->priority < b->priority) return -1;
    if (a->priority > b->priority) return 1;
    for (int i = 0; i < 6; ++i) {
        if (a->mac[i] < b->mac[i]) return -1;
        if (a->mac[i] > b->mac[i]) return 1;
    }
    return 0;
}

/* BPDU comparison tuple: (root_id, root_path_cost, sender_id, sender_port_id) */
static cmp_t compare_bpdus(const BPDU *a, const BPDU *b) {
    int r = bridgeid_compare(&a->root_id, &b->root_id);
    if (r < 0) return BETTER;
    if (r > 0) return WORSE;

    if (a->root_path_cost < b->root_path_cost) return BETTER;
    if (a->root_path_cost > b->root_path_cost) return WORSE;

    r = bridgeid_compare(&a->sender_id, &b->sender_id);
    if (r < 0) return BETTER;
    if (r > 0) return WORSE;

    if (a->sender_port_id < b->sender_port_id) return BETTER;
    if (a->sender_port_id > b->sender_port_id) return WORSE;

    return SAME;
}

/* Copy BPDU utility */
static void copy_bpdu(BPDU *dst, const BPDU *src) {
    memcpy(dst, src, sizeof(BPDU));
}

/* ---------- Logging helpers ---------- */
static void log_bridge(const Bridge *br, const char *fmt, ...) {
    char bidbuf[48];
    print_bid(&br->self_id, bidbuf, sizeof(bidbuf));
    va_list ap;
    va_start(ap, fmt);
    printf("[%s|%s] ", br->name, bidbuf);
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

/* ---------- BPDU transmission hooks (platform integration points) ---------- */

/* send_bpdu_on_link: called to send BPDU out of a port.
 * In a real system: encode BPDU into frame and write to NIC.
 * Here: we simulate by directly calling the remote port's receive function.
 */
static void send_bpdu_on_link(Port *p, const BPDU *bpdu)
{
    Bridge *br = p->parent;
    char buf_local[64], buf_root[64];
    print_bid(&br->self_id, buf_local, sizeof(buf_local));
    print_bid(&bpdu->root_id, buf_root, sizeof(buf_root));

    printf("  TX on %s:%u -> root=%s cost=%u flags=0x%02x\n",
           br->name, p->port_no, buf_root, (unsigned)bpdu->root_path_cost, bpdu->flags);

    if (p->remote) {
        // simulate transmission latency by direct call
        // incoming side will treat msg_age appropriately
        Port *remote = p->remote;
        // create a local copy to simulate independent packet
        BPDU copy;
        copy_bpdu(&copy, bpdu);
        // When sending, increment msg_age by 1 for the remote side's perspective
        if (copy.msg_age + 1 > copy.max_age) {
            // dropped due to max_age - simulate no receive
            return;
        }
        copy.msg_age += 1;
        // remote receives it
        // In a real system, this would be delivered by NIC interrupt
        // Here: call receive function
        // Simulate as an asynchronous arrival; for simplicity call it directly
        // But ensure remote->parent exists
        if (remote->parent) {
            // We'll push packet into remote's best_received processing
            // The receive routine will be called from the main loop to mimic real event handling
            // To keep this example simple, we'll store it in the remote's best_received candidate
            // Actually call a direct process function for clarity:
            // (But we need to ensure ordering semantics: this is acceptable for sim)
            // Implemented below as process_received_bpdu(remote, &copy)
            // However, to avoid forward declarations, call a function pointer later; use a global handler
            extern void process_received_bpdu(Bridge *bridge, Port *port, const BPDU *bpdu);
            process_received_bpdu(remote->parent, remote, &copy);
        }
    }
}

/* ---------- Core STP algorithm components ---------- */

/* Forward declarations */
void recompute_roles(Bridge *br);
void update_port_state(Port *p);

/* Initialize a bridge with simple params */
static void bridge_init(Bridge *br, const char *name, uint16_t priority, const uint8_t mac[6]) {
    memset(br, 0, sizeof(*br));
    br->self_id.priority = priority;
    memcpy(br->self_id.mac, mac, 6);
    br->root_id = br->self_id; // assume self as root initially
    br->root_path_cost = 0;
    br->hello_time = 2;       // default
    br->max_age = 20;
    br->forward_delay = 15;
    br->num_ports = 0;
    strncpy(br->name, name, sizeof(br->name)-1);
}

/* Add a port to bridge */
static Port *bridge_add_port(Bridge *br, uint16_t port_no, uint32_t path_cost) {
    if (br->num_ports >= MAX_PORTS) return NULL;
    Port *p = &br->ports[br->num_ports++];
    memset(p, 0, sizeof(*p));
    p->port_no = port_no;
    p->role = DESIGNATED_PORT; // default
    p->state = DISCARDING;
    p->path_cost = path_cost;
    p->parent = br;
    p->hello_timer = br->hello_time;
    p->forward_delay_timer = br->forward_delay;
    p->msg_age_timer = 0;
    return p;
}

/* Simple function to build a BPDU reflecting bridge's current view, for a given port */
static void build_config_bpdu(const Bridge *br, const Port *p, BPDU *out) {
    memset(out, 0, sizeof(*out));
    out->root_id = br->root_id;
    out->root_path_cost = br->root_path_cost;
    out->sender_id = br->self_id;
    out->sender_port_id = p->port_no;
    out->flags = 0;
    if (p->proposed) out->flags |= BPDU_FLAG_PROPOSAL;
    if (p->syncing) out->flags |= BPDU_FLAG_AGREEMENT;
    out->msg_age = 0;
    out->max_age = br->max_age;
    out->hello_time = br->hello_time;
    out->forward_delay = br->forward_delay;
}

/* Determine if a received BPDU is 'better' than current best on port */
static bool is_better_than_current(Port *p, const BPDU *r) {
    if (!p->have_bpdu) return true;
    cmp_t c = compare_bpdus(r, &p->best_received);
    return (c == BETTER);
}

/* Process a received BPDU on a port (called from simulated RX path) */
void process_received_bpdu(Bridge *bridge, Port *port, const BPDU *bpdu) {
    // Basic sanity
    if (!bridge || !port || !bpdu) return;

    char rbuf[64];
    char sbuf[64];
    print_bid(&bpdu->root_id, rbuf, sizeof(rbuf));
    print_bid(&bpdu->sender_id, sbuf, sizeof(sbuf));
    printf("  RX on %s:%u <- root=%s cost=%u sender=%s port=%u flags=0x%02x age=%u\n",
           bridge->name, port->port_no,
           rbuf, (unsigned)bpdu->root_path_cost, sbuf, bpdu->sender_port_id, bpdu->flags, bpdu->msg_age);

    // If msg_age >= max_age, drop
    if (bpdu->msg_age >= bpdu->max_age) {
        printf("    Dropped due to msg_age >= max_age\n");
        return;
    }

    // If this BPDU is better than current best for this port, update
    if (is_better_than_current(port, bpdu)) {
        copy_bpdu(&port->best_received, bpdu);
        port->have_bpdu = true;
        port->msg_age_timer = bpdu->msg_age;

        // Compute candidate root path cost via this port
        uint32_t candidate_cost = bpdu->root_path_cost + port->path_cost;

        // If this BPDU indicates a better root than we currently know, update bridge root
        cmp_t croot = compare_bpdus(bpdu, &(BPDU){ .root_id = bridge->root_id,
                                                   .root_path_cost = bridge->root_path_cost,
                                                   .sender_id = bridge->self_id,
                                                   .sender_port_id = 0 });
        // Above compare_bpdus trick: compare root_id then cost then sender id etc.
        // Simpler test: compare root IDs, then compare candidate_cost
        if (bridge->root_id.priority != bpdu->root_id.priority ||
            memcmp(bridge->root_id.mac, bpdu->root_id.mac, 6) != 0 ||
            candidate_cost < bridge->root_path_cost) {
            // Adopt new root
            bridge->root_id = bpdu->root_id;
            bridge->root_path_cost = candidate_cost;
            log_bridge(bridge, "New root learned via port %u: root is now", port->port_no);
            char tmp[64]; print_bid(&bridge->root_id, tmp, sizeof(tmp)); printf("    %s cost=%u\n", tmp, (unsigned)bridge->root_path_cost);
            // This port becomes candidate for root port
            // We'll recompute roles globally
        }

        // If bpdu has proposal flag (RSTP), respond with agreement if we can
        if (bpdu->flags & BPDU_FLAG_PROPOSAL) {
            // If this port is eligible (we are not an alternate blocking root), sync and reply agreement
            // For this simplified logic, if this bridge has no other ports blocking forwarding, accept
            // In full RSTP you must ensure all designated ports are synchronized before agreement.
            port->syncing = true;
            port->proposed = false;
            // Immediate state change for fast convergence: move to forwarding if allowed
            port->state = FORWARDING;
            port->role = ROOT_PORT; // if we accepted proposal, logically we become downstream design
            // send agreement back: set flags and transmit
            BPDU reply;
            build_config_bpdu(bridge, port, &reply);
            reply.flags |= BPDU_FLAG_AGREEMENT;
            send_bpdu_on_link(port, &reply);
        }

        // After processing, recompute roles for the whole bridge
        recompute_roles(bridge);
    } else {
        // If received BPDU is not better, might still contain useful flags like TC or Agreement
        if (bpdu->flags & BPDU_FLAG_AGREEMENT) {
            // remote side agreed; mark remote port as synced (for simplicity)
            port->syncing = true;
            port->state = FORWARDING;
            recompute_roles(bridge);
        }
    }
}

/* Decide root port and designated ports */
void recompute_roles(Bridge *br) {
    // Find best root port candidate: port with best (lowest) received BPDU tuple
    Port *best_rp = NULL;
    for (int i = 0; i < br->num_ports; ++i) {
        Port *p = &br->ports[i];
        if (!p->have_bpdu) continue;
        if (best_rp == NULL) {
            best_rp = p;
            continue;
        }
        cmp_t c = compare_bpdus(&p->best_received, &best_rp->best_received);
        if (c == BETTER) best_rp = p;
    }

    // Reset roles to default designated (we will select alternates explicitly)
    for (int i = 0; i < br->num_ports; ++i) {
        br->ports[i].role = DESIGNATED_PORT;
    }

    if (best_rp) {
        // That port becomes root port
        best_rp->role = ROOT_PORT;
        // Any other port that has a received BPDU better than what we'd advertise on that segment becomes alternate
    }

    // For each port, determine if we're designated for that LAN (i.e., our own config BPDU is better than remote's)
    for (int i = 0; i < br->num_ports; ++i) {
        Port *p = &br->ports[i];
        if (!p->remote) continue;
        Port *r = p->remote;
        // Build our own would-be BPDU
        BPDU ourbp;
        build_config_bpdu(br, p, &ourbp);
        // Compare our BPDU vs remote's best_received (which remote would advertise)
        // But remote->best_received describes root as seen by remote. Simpler: compare (our root id + cost + sender id + port id)
        BPDU remote_advertised = r->have_bpdu ? r->best_received : (BPDU){0};
        // If remote has no BPDU, we assume we are designated
        if (!r->have_bpdu) {
            p->role = DESIGNATED_PORT;
            continue;
        }
        cmp_t c = compare_bpdus(&ourbp, &remote_advertised);
        if (c == BETTER) {
            p->role = DESIGNATED_PORT;
        } else if (c == WORSE) {
            // remote is designated -> we are alternate/backups
            p->role = ALTERNATE_PORT;
        } else {
            // tie -> break by port number
            if (p->port_no < r->port_no) p->role = DESIGNATED_PORT;
            else p->role = ALTERNATE_PORT;
        }
    }

    // Update states according to roles
    for (int i = 0; i < br->num_ports; ++i) {
        update_port_state(&br->ports[i]);
    }

    // Logging for debugging
    log_bridge(br, "Role recompute:");
    for (int i = 0; i < br->num_ports; ++i) {
        Port *p = &br->ports[i];
        const char *rstr = (p->role==ROOT_PORT)?"ROOT":(p->role==DESIGNATED_PORT)?"DESG":"ALT";
        const char *sstr = (p->state==DISCARDING)?"DIS":(p->state==LEARNING)?"LEA":"FWD";
        printf("    port %u role=%s state=%s have_bpdu=%d\n",
               p->port_no, rstr, sstr, p->have_bpdu?1:0);
    }
}

/* Port state transitions (simplified) */
void update_port_state(Port *p) {
    switch (p->role) {
        case DESIGNATED_PORT:
            // Designated ports eventually forward (after forward delay) unless edge
            if (p->oper_edge) {
                p->state = FORWARDING;
            } else {
                // If previously discarding, start forward delay
                if (p->state == DISCARDING) {
                    p->forward_delay_timer = p->parent->forward_delay;
                    p->state = LEARNING; // move into learning quickly in this simplified model
                }
                // If timer expired -> forwarding
                if (p->forward_delay_timer == 0) p->state = FORWARDING;
            }
            break;
        case ROOT_PORT:
            // Root ports should forward after checking learning; simplified to immediate forward
            p->state = FORWARDING;
            break;
        case ALTERNATE_PORT:
            // Alternate ports should discard (blocking)
            p->state = DISCARDING;
            break;
        default:
            p->state = DISCARDING;
            break;
    }
}

/* Periodic transmit and timer handling for a bridge (call every 1 second) */
void stp_tick(Bridge *br) {
    // For each port, decrement timers and maybe transmit BPDU if hello timer expired
    for (int i = 0; i < br->num_ports; ++i) {
        Port *p = &br->ports[i];

        // Decrement hello timer
        if (p->hello_timer > 0) p->hello_timer--;
        if (p->hello_timer == 0) {
            // Build BPDU representing our current view
            BPDU bpdu;
            build_config_bpdu(br, p, &bpdu);
            // Advertise our root path cost (if we are designated, include our root path cost)
            bpdu.root_path_cost = br->root_path_cost;
            // transmit
            send_bpdu_on_link(p, &bpdu);
            // reset hello timer
            p->hello_timer = br->hello_time;
        }

        // Decrement forward delay timer if > 0
        if (p->forward_delay_timer > 0) p->forward_delay_timer--;

        // Message age processing: simulate frames aging - if a port hasn't seen fresh BPDU,
        // the port->msg_age_timer increments and when exceeds max_age, clear best_received.
        if (p->have_bpdu) {
            p->msg_age_timer++;
            if (p->msg_age_timer >= br->max_age) {
                p->have_bpdu = false;
                // this may cause role recalculation
                recompute_roles(br);
            }
        }
    }
}

/* ---------- Simple simulation / test harness ---------- */

/* Helper to connect two ports together (bidirectional) */
static void connect_ports(Port *a, Port *b) {
    a->remote = b;
    b->remote = a;
}

/* Utility to set MAC addresses easily */
static void mac_from_int(uint8_t mac[6], unsigned int x) {
    mac[0] = (x >> 40) & 0xff;
    mac[1] = (x >> 32) & 0xff;
    mac[2] = (x >> 24) & 0xff;
    mac[3] = (x >> 16) & 0xff;
    mac[4] = (x >> 8) & 0xff;
    mac[5] = x & 0xff;
}

/* Demo: create 3 bridges in triangle topology */
int main(void) {
    // Create three bridges
    Bridge br1, br2, br3;
    uint8_t m1[6] = {0x00,0x0a,0x95,0x00,0x01,0x01};
    uint8_t m2[6] = {0x00,0x0a,0x95,0x00,0x01,0x02};
    uint8_t m3[6] = {0x00,0x0a,0x95,0x00,0x01,0x03};

    bridge_init(&br1, "BR1", 32768, m1);
    bridge_init(&br2, "BR2", 32768, m2);
    bridge_init(&br3, "BR3", 32768, m3);

    // Add ports: make triangle BR1-BR2, BR2-BR3, BR3-BR1
    Port *p1_1 = bridge_add_port(&br1, 1, 19);
    Port *p1_2 = bridge_add_port(&br1, 2, 19);

    Port *p2_1 = bridge_add_port(&br2, 1, 19);
    Port *p2_2 = bridge_add_port(&br2, 2, 19);

    Port *p3_1 = bridge_add_port(&br3, 1, 19);
    Port *p3_2 = bridge_add_port(&br3, 2, 19);

    // Connect: BR1:1 <-> BR2:1 ; BR2:2 <-> BR3:1 ; BR3:2 <-> BR1:2
    connect_ports(p1_1, p2_1);
    connect_ports(p2_2, p3_1);
    connect_ports(p3_2, p1_2);

    // Initial TX: each bridge should advertise itself as root at start
    for (int tick = 0; tick < 30; ++tick) {
        printf("=== tick %d ===\n", tick);
        stp_tick(&br1);
        stp_tick(&br2);
        stp_tick(&br3);
        // small sleep to slow output (comment out on embedded)
        // usleep(200000); // optional
    }

    printf("Simulation complete\n");
    return 0;
}
