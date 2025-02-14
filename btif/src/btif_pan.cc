/******************************************************************************
 *
 *  Copyright 2009-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/*******************************************************************************
 *
 *  Filename:      btif_pan.c
 *
 *  Description:   PAN Profile Bluetooth Interface
 *
 *
 ******************************************************************************/

#define LOG_TAG "bt_btif_pan"

#include <base/bind.h>
#include <base/logging.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/if_ether.h>
#include <linux/if_tun.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/prctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <hardware/bluetooth.h>
#include <hardware/bt_pan.h>

#include "bt_common.h"
#include "bta_api.h"
#include "bta_pan_api.h"
#include "btif_common.h"
#include "btif_pan_internal.h"
#include "btif_sock_thread.h"
#include "btif_sock_util.h"
#include "btif_util.h"
#include "btm_api.h"
#include "device/include/controller.h"
#include "osi/include/log.h"
#include "osi/include/osi.h"
#include "stack/include/btu.h"

#if (BTA_PAN_INCLUDED == TRUE)

#define FORWARD_IGNORE 1
#define FORWARD_SUCCESS 0
#define FORWARD_FAILURE (-1)
#define FORWARD_CONGEST (-2)

#define asrt(s)                                                          \
  do {                                                                   \
    if (!(s))                                                            \
      BTIF_TRACE_ERROR("btif_pan: ## %s assert %s failed at line:%d ##", \
                       __func__, #s, __LINE__)                           \
  } while (0)

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

btpan_cb_t btpan_cb;

static bool jni_initialized;
static bool stack_initialized;

static bt_status_t btpan_jni_init(const btpan_callbacks_t* callbacks);
static void btpan_jni_cleanup();
static bt_status_t btpan_connect(const RawAddress* bd_addr, int local_role,
                                 int remote_role);
static bt_status_t btpan_disconnect(const RawAddress* bd_addr);
static bt_status_t btpan_enable(int local_role);
static int btpan_get_local_role(void);

static void btpan_tap_fd_signaled(int fd, int type, int flags,
                                  uint32_t user_id);
static void btpan_cleanup_conn(btpan_conn_t* conn);
static void bta_pan_callback(tBTA_PAN_EVT event, tBTA_PAN* p_data);
static void btu_exec_tap_fd_read(const int fd);

static btpan_interface_t pan_if = {
    sizeof(pan_if), btpan_jni_init,   nullptr,          btpan_get_local_role,
    btpan_connect,  btpan_disconnect, btpan_jni_cleanup};

const btpan_interface_t* btif_pan_get_interface() { return &pan_if; }

/*******************************************************************************
 **
 ** Function        btif_pan_init
 **
 ** Description     initializes the pan interface
 **
 ** Returns         bt_status_t
 **
 ******************************************************************************/
void btif_pan_init() {
  BTIF_TRACE_DEBUG("jni_initialized = %d, btpan_cb.enabled:%d", jni_initialized,
                   btpan_cb.enabled);
  stack_initialized = true;

  if (jni_initialized && !btpan_cb.enabled) {
    BTIF_TRACE_DEBUG("Enabling PAN....");
    memset(&btpan_cb, 0, sizeof(btpan_cb));
    btpan_cb.tap_fd = INVALID_FD;
    btpan_cb.flow = 1;
    for (int i = 0; i < MAX_PAN_CONNS; i++)
      btpan_cleanup_conn(&btpan_cb.conns[i]);
    BTA_PanEnable(bta_pan_callback);
    btpan_cb.enabled = 1;

    int role = BTPAN_ROLE_NONE;
#if PAN_NAP_DISABLED == FALSE
    role |= BTPAN_ROLE_PANNAP;
#endif
#if PANU_DISABLED == FALSE
    role |= BTPAN_ROLE_PANU;
#endif
    btpan_enable(role);
  }
}

static void pan_disable() {
  if (btpan_cb.enabled) {
    btpan_cb.enabled = 0;
    BTA_PanDisable();
    if (btpan_cb.tap_fd != INVALID_FD) {
      btpan_tap_close(btpan_cb.tap_fd);
      btpan_cb.tap_fd = INVALID_FD;
    }
  }
}

void btif_pan_cleanup() {
  if (!stack_initialized) return;

  // Bluetooth is shuting down, invalidate all BTA PAN handles
  for (int i = 0; i < MAX_PAN_CONNS; i++)
    btpan_cleanup_conn(&btpan_cb.conns[i]);

  pan_disable();
  stack_initialized = false;
}

static btpan_callbacks_t callback;
static bt_status_t btpan_jni_init(const btpan_callbacks_t* callbacks) {
  BTIF_TRACE_DEBUG("stack_initialized = %d, btpan_cb.enabled:%d",
                   stack_initialized, btpan_cb.enabled);
  callback = *callbacks;
  jni_initialized = true;
  if (stack_initialized && !btpan_cb.enabled) btif_pan_init();
  return BT_STATUS_SUCCESS;
}

static void btpan_jni_cleanup() {
  pan_disable();
  jni_initialized = false;
}

static inline int bta_role_to_btpan(int bta_pan_role) {
  int btpan_role = 0;
  BTIF_TRACE_DEBUG("bta_pan_role:0x%x", bta_pan_role);
  if (bta_pan_role & PAN_ROLE_NAP_SERVER) btpan_role |= BTPAN_ROLE_PANNAP;
  if (bta_pan_role & PAN_ROLE_CLIENT) btpan_role |= BTPAN_ROLE_PANU;
  return btpan_role;
}

static inline int btpan_role_to_bta(int btpan_role) {
  int bta_pan_role = PAN_ROLE_INACTIVE;
  BTIF_TRACE_DEBUG("btpan_role:0x%x", btpan_role);
  if (btpan_role & BTPAN_ROLE_PANNAP) bta_pan_role |= PAN_ROLE_NAP_SERVER;
  if (btpan_role & BTPAN_ROLE_PANU) bta_pan_role |= PAN_ROLE_CLIENT;
  return bta_pan_role;
}

static volatile int btpan_dev_local_role;
#if (BTA_PAN_INCLUDED == TRUE)
static tBTA_PAN_ROLE_INFO bta_panu_info = {PANU_SERVICE_NAME, 0};
static tBTA_PAN_ROLE_INFO bta_pan_nap_info = {PAN_NAP_SERVICE_NAME, 1};
#endif

static bt_status_t btpan_enable(int local_role) {
#if (BTA_PAN_INCLUDED == TRUE)
  BTIF_TRACE_DEBUG("%s - local_role: %d", __func__, local_role);
  int bta_pan_role = btpan_role_to_bta(local_role);
  BTA_PanSetRole(bta_pan_role, &bta_panu_info, &bta_pan_nap_info);
  btpan_dev_local_role = local_role;
  return BT_STATUS_SUCCESS;
#else
  return BT_STATUS_FAIL;
#endif
}

static int btpan_get_local_role() {
  BTIF_TRACE_DEBUG("btpan_dev_local_role:%d", btpan_dev_local_role);
  return btpan_dev_local_role;
}

static bt_status_t btpan_connect(const RawAddress* bd_addr, int local_role,
                                 int remote_role) {
  BTIF_TRACE_DEBUG("local_role:%d, remote_role:%d", local_role, remote_role);
  int bta_local_role = btpan_role_to_bta(local_role);
  int bta_remote_role = btpan_role_to_bta(remote_role);
  btpan_new_conn(-1, *bd_addr, bta_local_role, bta_remote_role);
  BTA_PanOpen(*bd_addr, bta_local_role, bta_remote_role);
  return BT_STATUS_SUCCESS;
}

static void btif_in_pan_generic_evt(uint16_t event, char* p_param) {
  BTIF_TRACE_EVENT("%s: event=%d", __func__, event);
  switch (event) {
    case BTIF_PAN_CB_DISCONNECTING: {
      RawAddress* bd_addr = (RawAddress*)p_param;
      btpan_conn_t* conn = btpan_find_conn_addr(*bd_addr);
      int btpan_conn_local_role;
      int btpan_remote_role;
      asrt(conn != NULL);
      if (conn) {
        btpan_conn_local_role = bta_role_to_btpan(conn->local_role);
        btpan_remote_role = bta_role_to_btpan(conn->remote_role);
        callback.connection_state_cb(BTPAN_STATE_DISCONNECTING,
                                     BT_STATUS_SUCCESS, &conn->peer,
                                     btpan_conn_local_role, btpan_remote_role);
      }
    } break;
    default: {
      BTIF_TRACE_WARNING("%s : Unknown event 0x%x", __func__, event);
    } break;
  }
}

static bt_status_t btpan_disconnect(const RawAddress* bd_addr) {
  btpan_conn_t* conn = btpan_find_conn_addr(*bd_addr);
  if (conn && conn->handle >= 0) {
    /* Inform the application that the disconnect has been initiated
     * successfully */
    btif_transfer_context(btif_in_pan_generic_evt, BTIF_PAN_CB_DISCONNECTING,
                          (char*)bd_addr, sizeof(RawAddress), NULL);
    BTA_PanClose(conn->handle);
    return BT_STATUS_SUCCESS;
  }
  return BT_STATUS_FAIL;
}

static int pan_pth = -1;
void create_tap_read_thread(int tap_fd) {
  if (pan_pth < 0) pan_pth = btsock_thread_create(btpan_tap_fd_signaled, NULL);
  if (pan_pth >= 0)
    btsock_thread_add_fd(pan_pth, tap_fd, 0, SOCK_THREAD_FD_RD, 0);
}

void destroy_tap_read_thread(void) {
  if (pan_pth >= 0) {
    btsock_thread_exit(pan_pth);
    pan_pth = -1;
  }
}

static int tap_if_up(const char* devname, const RawAddress* addr) {
  struct ifreq ifr;
  int sk, err;

  sk = socket(AF_INET, SOCK_DGRAM, 0);
  if (sk < 0) return -1;

  // set mac addr
  memset(&ifr, 0, sizeof(ifr));
  strlcpy(ifr.ifr_name, devname, IFNAMSIZ);
  err = ioctl(sk, SIOCGIFHWADDR, &ifr);
  if (err < 0) {
    BTIF_TRACE_ERROR(
        "Could not get network hardware for interface:%s, errno:%s", devname,
        strerror(errno));
    close(sk);
    return -1;
  }

  strlcpy(ifr.ifr_name, devname, IFNAMSIZ);
  memcpy(ifr.ifr_hwaddr.sa_data, addr->address, 6);

  /* The IEEE has specified that the most significant bit of the most
   * significant byte is used to
   * determine a multicast address. If its a 1, that means multicast, 0 means
   * unicast.
   * Kernel returns an error if we try to set a multicast address for the
   * tun-tap ethernet interface.
   * Mask this bit to avoid any issue with auto generated address.
   */
  if (ifr.ifr_hwaddr.sa_data[0] & 0x01) {
    BTIF_TRACE_WARNING(
        "Not a unicast MAC address, force multicast bit flipping");
    ifr.ifr_hwaddr.sa_data[0] &= ~0x01;
  }

  err = ioctl(sk, SIOCSIFHWADDR, (caddr_t)&ifr);

  if (err < 0) {
    BTIF_TRACE_ERROR("Could not set bt address for interface:%s, errno:%s",
                     devname, strerror(errno));
    close(sk);
    return -1;
  }

  // bring it up
  memset(&ifr, 0, sizeof(ifr));
  strlcpy(ifr.ifr_name, devname, IF_NAMESIZE);

  ifr.ifr_flags |= IFF_UP;
  ifr.ifr_flags |= IFF_MULTICAST;

  err = ioctl(sk, SIOCSIFFLAGS, (caddr_t)&ifr);

  if (err < 0) {
    BTIF_TRACE_ERROR("Could not bring up network interface:%s, errno:%d",
                     devname, errno);
    close(sk);
    return -1;
  }
  close(sk);
  BTIF_TRACE_DEBUG("network interface: %s is up", devname);
  return 0;
}

static int tap_if_down(const char* devname) {
  struct ifreq ifr;
  int sk;

  sk = socket(AF_INET, SOCK_DGRAM, 0);
  if (sk < 0) return -1;

  memset(&ifr, 0, sizeof(ifr));
  strlcpy(ifr.ifr_name, devname, IF_NAMESIZE);

  ifr.ifr_flags &= ~IFF_UP;

  ioctl(sk, SIOCSIFFLAGS, (caddr_t)&ifr);

  close(sk);

  return 0;
}

void btpan_set_flow_control(bool enable) {
  if (btpan_cb.tap_fd == -1) return;

  btpan_cb.flow = enable;
  if (enable) {
    btsock_thread_add_fd(pan_pth, btpan_cb.tap_fd, 0, SOCK_THREAD_FD_RD, 0);
    do_in_main_thread(FROM_HERE,
                      base::Bind(btu_exec_tap_fd_read, btpan_cb.tap_fd));
  }
}

int btpan_tap_open() {
  struct ifreq ifr;
  int fd, err;
  const char* clonedev = "/dev/tun";

  /* open the clone device */

  fd = open(clonedev, O_RDWR);
  if (fd < 0) {
    BTIF_TRACE_DEBUG("could not open %s, err:%d", clonedev, errno);
    return fd;
  }

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_TAP | IFF_NO_PI;

  strlcpy(ifr.ifr_name, TAP_IF_NAME, IFNAMSIZ);

  /* try to create the device */
  err = ioctl(fd, TUNSETIFF, (void*)&ifr);
  if (err < 0) {
    BTIF_TRACE_DEBUG("ioctl error:%d, errno:%s", err, strerror(errno));
    close(fd);
    return err;
  }
  if (tap_if_up(TAP_IF_NAME, controller_get_interface()->get_address()) == 0) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
  }
  BTIF_TRACE_ERROR("can not bring up tap interface:%s", TAP_IF_NAME);
  close(fd);
  return INVALID_FD;
}

int btpan_tap_send(int tap_fd, const RawAddress& src, const RawAddress& dst,
                   uint16_t proto, const char* buf, uint16_t len,
                   UNUSED_ATTR bool ext, UNUSED_ATTR bool forward) {
  if (tap_fd != INVALID_FD) {
    tETH_HDR eth_hdr;
    eth_hdr.h_dest = dst;
    eth_hdr.h_src = src;
    eth_hdr.h_proto = htons(proto);
    char packet[TAP_MAX_PKT_WRITE_LEN + sizeof(tETH_HDR)];
    memcpy(packet, &eth_hdr, sizeof(tETH_HDR));
    if (len > TAP_MAX_PKT_WRITE_LEN) {
      LOG_ERROR("btpan_tap_send eth packet size:%d is exceeded limit!", len);
      return -1;
    }
    memcpy(packet + sizeof(tETH_HDR), buf, len);

    /* Send data to network interface */
    ssize_t ret;
    OSI_NO_INTR(ret = write(tap_fd, packet, len + sizeof(tETH_HDR)));
    BTIF_TRACE_DEBUG("ret:%d", ret);
    return (int)ret;
  }
  return -1;
}

int btpan_tap_close(int fd) {
  if (tap_if_down(TAP_IF_NAME) == 0) close(fd);
  if (pan_pth >= 0) btsock_thread_wakeup(pan_pth);
  return 0;
}

btpan_conn_t* btpan_find_conn_handle(uint16_t handle) {
  for (int i = 0; i < MAX_PAN_CONNS; i++) {
    if (btpan_cb.conns[i].handle == handle) return &btpan_cb.conns[i];
  }
  return NULL;
}

btpan_conn_t* btpan_find_conn_addr(const RawAddress& addr) {
  for (int i = 0; i < MAX_PAN_CONNS; i++) {
    if (btpan_cb.conns[i].peer == addr) return &btpan_cb.conns[i];
  }
  return NULL;
}

static void btpan_open_conn(btpan_conn_t* conn, tBTA_PAN* p_data) {
  BTIF_TRACE_API(
      "btpan_open_conn: local_role:%d, peer_role: %d,  handle:%d, conn: %p",
      p_data->open.local_role, p_data->open.peer_role, p_data->open.handle,
      conn);

  if (conn == NULL)
    conn = btpan_new_conn(p_data->open.handle, p_data->open.bd_addr,
                          p_data->open.local_role, p_data->open.peer_role);
  if (conn) {
    BTIF_TRACE_DEBUG(
        "btpan_open_conn:tap_fd:%d, open_count:%d, "
        "conn->handle:%d should = handle:%d, local_role:%d, remote_role:%d",
        btpan_cb.tap_fd, btpan_cb.open_count, conn->handle, p_data->open.handle,
        conn->local_role, conn->remote_role);

    btpan_cb.open_count++;
    conn->handle = p_data->open.handle;
    if (btpan_cb.tap_fd < 0) {
      btpan_cb.tap_fd = btpan_tap_open();
      if (btpan_cb.tap_fd >= 0) create_tap_read_thread(btpan_cb.tap_fd);
    }

    if (btpan_cb.tap_fd >= 0) {
      btpan_cb.flow = 1;
      conn->state = PAN_STATE_OPEN;
    }
  }
}

static void btpan_close_conn(btpan_conn_t* conn) {
  BTIF_TRACE_API("btpan_close_conn: %p", conn);

  if (conn && conn->state == PAN_STATE_OPEN) {
    BTIF_TRACE_DEBUG("btpan_close_conn: PAN_STATE_OPEN");

    conn->state = PAN_STATE_CLOSE;
    btpan_cb.open_count--;

    if (btpan_cb.open_count == 0) {
      destroy_tap_read_thread();
      if (btpan_cb.tap_fd != INVALID_FD) {
        btpan_tap_close(btpan_cb.tap_fd);
        btpan_cb.tap_fd = INVALID_FD;
      }
    }
  }
}

static void btpan_cleanup_conn(btpan_conn_t* conn) {
  if (conn) {
    conn->handle = -1;
    conn->state = -1;
    memset(&conn->peer, 0, sizeof(conn->peer));
    memset(&conn->eth_addr, 0, sizeof(conn->eth_addr));
    conn->local_role = conn->remote_role = 0;
  }
}

btpan_conn_t* btpan_new_conn(int handle, const RawAddress& addr, int local_role,
                             int remote_role) {
  for (int i = 0; i < MAX_PAN_CONNS; i++) {
    BTIF_TRACE_DEBUG("conns[%d]:%d", i, btpan_cb.conns[i].handle);
    if (btpan_cb.conns[i].handle == -1) {
      BTIF_TRACE_DEBUG("handle:%d, local_role:%d, remote_role:%d", handle,
                       local_role, remote_role);

      btpan_cb.conns[i].handle = handle;
      btpan_cb.conns[i].peer = addr;
      btpan_cb.conns[i].local_role = local_role;
      btpan_cb.conns[i].remote_role = remote_role;
      return &btpan_cb.conns[i];
    }
  }
  BTIF_TRACE_DEBUG("MAX_PAN_CONNS:%d exceeded, return NULL as failed",
                   MAX_PAN_CONNS);
  return NULL;
}

void btpan_close_handle(btpan_conn_t* p) {
  BTIF_TRACE_DEBUG("btpan_close_handle : close handle %d", p->handle);
  p->handle = -1;
  p->local_role = -1;
  p->remote_role = -1;
  memset(&p->peer, 0, 6);
}

static inline bool should_forward(tETH_HDR* hdr) {
  uint16_t proto = ntohs(hdr->h_proto);
  if (proto == ETH_P_IP || proto == ETH_P_ARP || proto == ETH_P_IPV6)
    return true;
  BTIF_TRACE_DEBUG("unknown proto:%x", proto);
  return false;
}

static int forward_bnep(tETH_HDR* eth_hdr, BT_HDR* hdr) {
  int broadcast = eth_hdr->h_dest.address[0] & 1;

  // Find the right connection to send this frame over.
  for (int i = 0; i < MAX_PAN_CONNS; i++) {
    uint16_t handle = btpan_cb.conns[i].handle;
    if (handle != (uint16_t)-1 &&
        (broadcast || btpan_cb.conns[i].eth_addr == eth_hdr->h_dest ||
         btpan_cb.conns[i].peer == eth_hdr->h_dest)) {
      int result = PAN_WriteBuf(handle, eth_hdr->h_dest, eth_hdr->h_src,
                                ntohs(eth_hdr->h_proto), hdr, 0);
      switch (result) {
        case PAN_Q_SIZE_EXCEEDED:
          return FORWARD_CONGEST;
        case PAN_SUCCESS:
          return FORWARD_SUCCESS;
        default:
          return FORWARD_FAILURE;
      }
    }
  }
  osi_free(hdr);
  return FORWARD_IGNORE;
}

static void bta_pan_callback_transfer(uint16_t event, char* p_param) {
  tBTA_PAN* p_data = (tBTA_PAN*)p_param;

  switch (event) {
    case BTA_PAN_ENABLE_EVT:
      BTIF_TRACE_DEBUG("BTA_PAN_ENABLE_EVT");
      break;
    case BTA_PAN_SET_ROLE_EVT: {
      int btpan_role = bta_role_to_btpan(p_data->set_role.role);
      bt_status_t status = p_data->set_role.status == BTA_PAN_SUCCESS
                               ? BT_STATUS_SUCCESS
                               : BT_STATUS_FAIL;
      btpan_control_state_t state =
          btpan_role == 0 ? BTPAN_STATE_DISABLED : BTPAN_STATE_ENABLED;
      callback.control_state_cb(state, btpan_role, status, TAP_IF_NAME);
      break;
    }
    case BTA_PAN_OPENING_EVT: {
      btpan_conn_t* conn;
      BTIF_TRACE_DEBUG("BTA_PAN_OPENING_EVT handle %d, addr: %s",
                       p_data->opening.handle,
                       p_data->opening.bd_addr.ToString().c_str());
      conn = btpan_find_conn_addr(p_data->opening.bd_addr);

      asrt(conn != NULL);
      if (conn) {
        conn->handle = p_data->opening.handle;
        int btpan_conn_local_role = bta_role_to_btpan(conn->local_role);
        int btpan_remote_role = bta_role_to_btpan(conn->remote_role);
        callback.connection_state_cb(BTPAN_STATE_CONNECTING, BT_STATUS_SUCCESS,
                                     &p_data->opening.bd_addr,
                                     btpan_conn_local_role, btpan_remote_role);
      } else
        BTIF_TRACE_ERROR("connection not found");
      break;
    }
    case BTA_PAN_OPEN_EVT: {
      btpan_connection_state_t state;
      bt_status_t status;
      btpan_conn_t* conn = btpan_find_conn_handle(p_data->open.handle);

      LOG_VERBOSE("%s pan connection open status: %d", __func__,
                  p_data->open.status);
      if (p_data->open.status == BTA_PAN_SUCCESS) {
        state = BTPAN_STATE_CONNECTED;
        status = BT_STATUS_SUCCESS;
        btpan_open_conn(conn, p_data);
      } else {
        state = BTPAN_STATE_DISCONNECTED;
        status = BT_STATUS_FAIL;
        btpan_cleanup_conn(conn);
      }
      /* debug("BTA_PAN_OPEN_EVT handle:%d, conn:%p",  p_data->open.handle,
       * conn); */
      /* debug("conn bta local_role:%d, bta remote role:%d", conn->local_role,
       * conn->remote_role); */
      int btpan_conn_local_role = bta_role_to_btpan(p_data->open.local_role);
      int btpan_remote_role = bta_role_to_btpan(p_data->open.peer_role);
      callback.connection_state_cb(state, status, &p_data->open.bd_addr,
                                   btpan_conn_local_role, btpan_remote_role);
      break;
    }
    case BTA_PAN_CLOSE_EVT: {
      LOG_INFO("%s: event = BTA_PAN_CLOSE_EVT handle %d", __func__,
               p_data->close.handle);
      btpan_conn_t* conn = btpan_find_conn_handle(p_data->close.handle);
      btpan_close_conn(conn);

      if (conn && conn->handle >= 0) {
        int btpan_conn_local_role = bta_role_to_btpan(conn->local_role);
        int btpan_remote_role = bta_role_to_btpan(conn->remote_role);
        callback.connection_state_cb(BTPAN_STATE_DISCONNECTED, (bt_status_t)0,
                                     &conn->peer, btpan_conn_local_role,
                                     btpan_remote_role);
        btpan_cleanup_conn(conn);
      } else
        BTIF_TRACE_ERROR("pan handle not found (%d)", p_data->close.handle);
      break;
    }
    default:
      BTIF_TRACE_WARNING("Unknown pan event %d", event);
      break;
  }
}

static void bta_pan_callback(tBTA_PAN_EVT event, tBTA_PAN* p_data) {
  btif_transfer_context(bta_pan_callback_transfer, event, (char*)p_data,
                        sizeof(tBTA_PAN), NULL);
}

#define IS_EXCEPTION(e) ((e) & (POLLHUP | POLLRDHUP | POLLERR | POLLNVAL))
static void btu_exec_tap_fd_read(int fd) {
  struct pollfd ufd;

  if (fd == INVALID_FD || fd != btpan_cb.tap_fd) return;

  // Don't occupy BTU context too long, avoid buffer overruns and
  // give other profiles a chance to run by limiting the amount of memory
  // PAN can use.
  for (int i = 0; i < PAN_BUF_MAX && btif_is_enabled() && btpan_cb.flow; i++) {
    BT_HDR* buffer = (BT_HDR*)osi_malloc(PAN_BUF_SIZE);
    buffer->offset = PAN_MINIMUM_OFFSET;
    buffer->len = PAN_BUF_SIZE - sizeof(BT_HDR) - buffer->offset;

    uint8_t* packet = (uint8_t*)buffer + sizeof(BT_HDR) + buffer->offset;

    // If we don't have an undelivered packet left over, pull one from the TAP
    // driver.
    // We save it in the congest_packet right away in case we can't deliver it
    // in this
    // attempt.
    if (!btpan_cb.congest_packet_size) {
      ssize_t ret;
      OSI_NO_INTR(ret = read(fd, btpan_cb.congest_packet,
                             sizeof(btpan_cb.congest_packet)));
      switch (ret) {
        case -1:
          BTIF_TRACE_ERROR("%s unable to read from driver: %s", __func__,
                           strerror(errno));
          osi_free(buffer);
          // add fd back to monitor thread to try it again later
          btsock_thread_add_fd(pan_pth, fd, 0, SOCK_THREAD_FD_RD, 0);
          return;
        case 0:
          BTIF_TRACE_WARNING("%s end of file reached.", __func__);
          osi_free(buffer);
          // add fd back to monitor thread to process the exception
          btsock_thread_add_fd(pan_pth, fd, 0, SOCK_THREAD_FD_RD, 0);
          return;
        default:
          btpan_cb.congest_packet_size = ret;
          break;
      }
    }

    memcpy(packet, btpan_cb.congest_packet,
           MIN(btpan_cb.congest_packet_size, buffer->len));
    buffer->len = MIN(btpan_cb.congest_packet_size, buffer->len);

    if (buffer->len > sizeof(tETH_HDR) && should_forward((tETH_HDR*)packet)) {
      // Extract the ethernet header from the buffer since the PAN_WriteBuf
      // inside
      // forward_bnep can't handle two pointers that point inside the same GKI
      // buffer.
      tETH_HDR hdr;
      memcpy(&hdr, packet, sizeof(tETH_HDR));

      // Skip the ethernet header.
      buffer->len -= sizeof(tETH_HDR);
      buffer->offset += sizeof(tETH_HDR);
      if (forward_bnep(&hdr, buffer) != FORWARD_CONGEST)
        btpan_cb.congest_packet_size = 0;
    } else {
      BTIF_TRACE_WARNING("%s dropping packet of length %d", __func__,
                         buffer->len);
      btpan_cb.congest_packet_size = 0;
      osi_free(buffer);
    }

    // Bail out of the loop if reading from the TAP fd would block.
    ufd.fd = fd;
    ufd.events = POLLIN;
    ufd.revents = 0;

    int ret;
    OSI_NO_INTR(ret = poll(&ufd, 1, 0));
    if (ret <= 0 || IS_EXCEPTION(ufd.revents)) break;
  }

  if (btpan_cb.flow) {
    // add fd back to monitor thread when the flow is on
    btsock_thread_add_fd(pan_pth, fd, 0, SOCK_THREAD_FD_RD, 0);
  }
}

static void btif_pan_close_all_conns() {
  if (!stack_initialized) return;

  for (int i = 0; i < MAX_PAN_CONNS; ++i) {
    if (btpan_cb.conns[i].handle != -1) BTA_PanClose(btpan_cb.conns[i].handle);
  }
}

static void btpan_tap_fd_signaled(int fd, int type, int flags,
                                  uint32_t user_id) {
  CHECK(btpan_cb.tap_fd == INVALID_FD || btpan_cb.tap_fd == fd);

  if (btpan_cb.tap_fd != fd) {
    BTIF_TRACE_WARNING("%s Signaled on mismatched fds exp:%d act:%d\n",
                       __func__, btpan_cb.tap_fd, fd);
    return;
  }

  if (flags & SOCK_THREAD_FD_EXCEPTION) {
    btpan_cb.tap_fd = INVALID_FD;
    btpan_tap_close(fd);
    btif_pan_close_all_conns();
  } else if (flags & SOCK_THREAD_FD_RD) {
    do_in_main_thread(FROM_HERE, base::Bind(btu_exec_tap_fd_read, fd));
  }
}

#endif
