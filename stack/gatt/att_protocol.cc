/******************************************************************************
 *
 *  Copyright 2008-2014 Broadcom Corporation
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

/******************************************************************************
 *
 *  this file contains ATT protocol functions
 *
 ******************************************************************************/

#include "bt_target.h"

#include "gatt_int.h"
#include "l2c_api.h"
#include "osi/include/log.h"

#define GATT_HDR_FIND_TYPE_VALUE_LEN 21
#define GATT_OP_CODE_SIZE 1
#define GATT_START_END_HANDLE_SIZE 4

using base::StringPrintf;
using bluetooth::Uuid;
/**********************************************************************
 *   ATT protocl message building utility                              *
 **********************************************************************/
/*******************************************************************************
 *
 * Function         attp_build_mtu_exec_cmd
 *
 * Description      Build a exchange MTU request
 *
 * Returns          None.
 *
 ******************************************************************************/
BT_HDR* attp_build_mtu_cmd(uint8_t op_code, uint16_t rx_mtu) {
  uint8_t* p;
  BT_HDR* p_buf =
      (BT_HDR*)osi_malloc(sizeof(BT_HDR) + GATT_HDR_SIZE + L2CAP_MIN_OFFSET);

  p = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;
  UINT8_TO_STREAM(p, op_code);
  UINT16_TO_STREAM(p, rx_mtu);

  p_buf->offset = L2CAP_MIN_OFFSET;
  p_buf->len = GATT_HDR_SIZE; /* opcode + 2 bytes mtu */

  return p_buf;
}
/*******************************************************************************
 *
 * Function         attp_build_exec_write_cmd
 *
 * Description      Build a execute write request or response.
 *
 * Returns          None.
 *
 ******************************************************************************/
BT_HDR* attp_build_exec_write_cmd(uint8_t op_code, uint8_t flag) {
  BT_HDR* p_buf = (BT_HDR*)osi_malloc(GATT_DATA_BUF_SIZE);
  uint8_t* p;

  p = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;

  p_buf->offset = L2CAP_MIN_OFFSET;
  p_buf->len = GATT_OP_CODE_SIZE;

  UINT8_TO_STREAM(p, op_code);

  if (op_code == GATT_REQ_EXEC_WRITE) {
    flag &= GATT_PREP_WRITE_EXEC;
    UINT8_TO_STREAM(p, flag);
    p_buf->len += 1;
  }

  return p_buf;
}

/*******************************************************************************
 *
 * Function         attp_build_err_cmd
 *
 * Description      Build a exchange MTU request
 *
 * Returns          None.
 *
 ******************************************************************************/
BT_HDR* attp_build_err_cmd(uint8_t cmd_code, uint16_t err_handle,
                           uint8_t reason) {
  uint8_t* p;
  BT_HDR* p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR) + L2CAP_MIN_OFFSET + 5);

  p = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;
  UINT8_TO_STREAM(p, GATT_RSP_ERROR);
  UINT8_TO_STREAM(p, cmd_code);
  UINT16_TO_STREAM(p, err_handle);
  UINT8_TO_STREAM(p, reason);

  p_buf->offset = L2CAP_MIN_OFFSET;
  /* GATT_HDR_SIZE (1B ERR_RSP op code+ 2B handle) + 1B cmd_op_code  + 1B status
   */
  p_buf->len = GATT_HDR_SIZE + 1 + 1;

  return p_buf;
}
/*******************************************************************************
 *
 * Function         attp_build_browse_cmd
 *
 * Description      Build a read information request or read by type request
 *
 * Returns          None.
 *
 ******************************************************************************/
BT_HDR* attp_build_browse_cmd(uint8_t op_code, uint16_t s_hdl, uint16_t e_hdl,
                              const bluetooth::Uuid& uuid) {
  const size_t payload_size =
      (GATT_OP_CODE_SIZE) + (GATT_START_END_HANDLE_SIZE) + (Uuid::kNumBytes128);
  BT_HDR* p_buf =
      (BT_HDR*)osi_malloc(sizeof(BT_HDR) + payload_size + L2CAP_MIN_OFFSET);

  uint8_t* p = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;
  /* Describe the built message location and size */
  p_buf->offset = L2CAP_MIN_OFFSET;
  p_buf->len = GATT_OP_CODE_SIZE + 4;

  UINT8_TO_STREAM(p, op_code);
  UINT16_TO_STREAM(p, s_hdl);
  UINT16_TO_STREAM(p, e_hdl);
  p_buf->len += gatt_build_uuid_to_stream(&p, uuid);

  return p_buf;
}

/*******************************************************************************
 *
 * Function         attp_build_read_handles_cmd
 *
 * Description      Build a read by type and value request.
 *
 * Returns          pointer to the command buffer.
 *
 ******************************************************************************/
BT_HDR* attp_build_read_by_type_value_cmd(uint16_t payload_size,
                                          tGATT_FIND_TYPE_VALUE* p_value_type) {
  uint8_t* p;
  uint16_t len = p_value_type->value_len;
  BT_HDR* p_buf =
      (BT_HDR*)osi_malloc(sizeof(BT_HDR) + payload_size + L2CAP_MIN_OFFSET);

  p = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;
  p_buf->offset = L2CAP_MIN_OFFSET;
  p_buf->len = 5; /* opcode + s_handle + e_handle */

  UINT8_TO_STREAM(p, GATT_REQ_FIND_TYPE_VALUE);
  UINT16_TO_STREAM(p, p_value_type->s_handle);
  UINT16_TO_STREAM(p, p_value_type->e_handle);

  p_buf->len += gatt_build_uuid_to_stream(&p, p_value_type->uuid);

  if (p_value_type->value_len + p_buf->len > payload_size)
    len = payload_size - p_buf->len;

  memcpy(p, p_value_type->value, len);
  p_buf->len += len;

  return p_buf;
}

/*******************************************************************************
 *
 * Function         attp_build_read_multi_cmd
 *
 * Description      Build a read multiple request
 *
 * Returns          None.
 *
 ******************************************************************************/
BT_HDR* attp_build_read_multi_cmd(uint8_t op_code, uint16_t payload_size,
                                  uint16_t num_handle, uint16_t* p_handle) {
  uint8_t *p, i = 0;
  BT_HDR* p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR) + num_handle * 2 + 1 +
                                      L2CAP_MIN_OFFSET);

  p = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;
  p_buf->offset = L2CAP_MIN_OFFSET;
  p_buf->len = 1;

  UINT8_TO_STREAM(p, op_code);

  for (i = 0; i < num_handle && p_buf->len + 2 <= payload_size; i++) {
    UINT16_TO_STREAM(p, *(p_handle + i));
    p_buf->len += 2;
  }

  return p_buf;
}
/*******************************************************************************
 *
 * Function         attp_build_handle_cmd
 *
 * Description      Build a read /read blob request
 *
 * Returns          None.
 *
 ******************************************************************************/
BT_HDR* attp_build_handle_cmd(uint8_t op_code, uint16_t handle,
                              uint16_t offset) {
  uint8_t* p;
  BT_HDR* p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR) + 5 + L2CAP_MIN_OFFSET);

  p = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;
  p_buf->offset = L2CAP_MIN_OFFSET;

  UINT8_TO_STREAM(p, op_code);
  p_buf->len = 1;

  UINT16_TO_STREAM(p, handle);
  p_buf->len += 2;

  if (op_code == GATT_REQ_READ_BLOB) {
    UINT16_TO_STREAM(p, offset);
    p_buf->len += 2;
  }

  return p_buf;
}

/*******************************************************************************
 *
 * Function         attp_build_opcode_cmd
 *
 * Description      Build a  request/response with opcode only.
 *
 * Returns          None.
 *
 ******************************************************************************/
BT_HDR* attp_build_opcode_cmd(uint8_t op_code) {
  uint8_t* p;
  BT_HDR* p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR) + 1 + L2CAP_MIN_OFFSET);

  p = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;
  p_buf->offset = L2CAP_MIN_OFFSET;

  UINT8_TO_STREAM(p, op_code);
  p_buf->len = 1;

  return p_buf;
}

/*******************************************************************************
 *
 * Function         attp_build_value_cmd
 *
 * Description      Build a attribute value request
 *
 * Returns          None.
 *
 ******************************************************************************/
BT_HDR* attp_build_value_cmd(uint16_t payload_size, uint8_t op_code,
                             uint16_t handle, uint16_t offset, uint16_t len,
                             uint8_t* p_data) {
  uint8_t *p, *pp, pair_len, *p_pair_len;
  BT_HDR* p_buf =
      (BT_HDR*)osi_malloc(sizeof(BT_HDR) + payload_size + L2CAP_MIN_OFFSET);

  p = pp = (uint8_t*)(p_buf + 1) + L2CAP_MIN_OFFSET;
  UINT8_TO_STREAM(p, op_code);
  p_buf->offset = L2CAP_MIN_OFFSET;
  p_buf->len = 1;

  if (op_code == GATT_RSP_READ_BY_TYPE) {
    p_pair_len = p;
    pair_len = len + 2;
    UINT8_TO_STREAM(p, pair_len);
    p_buf->len += 1;
  }
  if (op_code != GATT_RSP_READ_BLOB && op_code != GATT_RSP_READ) {
    UINT16_TO_STREAM(p, handle);
    p_buf->len += 2;
  }

  if (op_code == GATT_REQ_PREPARE_WRITE || op_code == GATT_RSP_PREPARE_WRITE) {
    UINT16_TO_STREAM(p, offset);
    p_buf->len += 2;
  }

  if (len > 0 && p_data != NULL) {
    /* ensure data not exceed MTU size */
    if (payload_size - p_buf->len < len) {
      len = payload_size - p_buf->len;
      /* update handle value pair length */
      if (op_code == GATT_RSP_READ_BY_TYPE) *p_pair_len = (len + 2);

      LOG(WARNING) << StringPrintf(
          "attribute value too long, to be truncated to %d", len);
    }

    ARRAY_TO_STREAM(p, p_data, len);
    p_buf->len += len;
  }

  return p_buf;
}

/*******************************************************************************
 *
 * Function         attp_send_msg_to_l2cap
 *
 * Description      Send message to L2CAP.
 *
 ******************************************************************************/
tGATT_STATUS attp_send_msg_to_l2cap(tGATT_TCB& tcb, uint16_t lcid,
                                    BT_HDR* p_toL2CAP) {
  uint16_t l2cap_ret;

  if (lcid == L2CAP_ATT_CID) {
    LOG_DEBUG("Sending ATT message on att fixed channel");
    l2cap_ret = L2CA_SendFixedChnlData(lcid, tcb.peer_bda, p_toL2CAP);
  } else {
    LOG_DEBUG("Sending ATT message on lcid:%hu", lcid);
    l2cap_ret = (uint16_t)L2CA_DataWrite(lcid, p_toL2CAP);
  }

  if (l2cap_ret == L2CAP_DW_FAILED) {
    LOG(ERROR) << __func__ << ": failed to write data to L2CAP";
    return GATT_INTERNAL_ERROR;
  } else if (l2cap_ret == L2CAP_DW_CONGESTED) {
    VLOG(1) << StringPrintf("ATT congested, message accepted");
    return GATT_CONGESTED;
  }
  return GATT_SUCCESS;
}

/** Build ATT Server PDUs */
BT_HDR* attp_build_sr_msg(tGATT_TCB& tcb, uint8_t op_code,
                          tGATT_SR_MSG* p_msg) {
  uint16_t offset = 0;

  switch (op_code) {
    case GATT_RSP_READ_BLOB:
    case GATT_RSP_PREPARE_WRITE:
      VLOG(1) << StringPrintf(
          "ATT_RSP_READ_BLOB/GATT_RSP_PREPARE_WRITE: len = %d offset = %d",
          p_msg->attr_value.len, p_msg->attr_value.offset);
      offset = p_msg->attr_value.offset;
      FALLTHROUGH_INTENDED; /* FALLTHROUGH */
    case GATT_RSP_READ_BY_TYPE:
    case GATT_RSP_READ:
    case GATT_HANDLE_VALUE_NOTIF:
    case GATT_HANDLE_VALUE_IND:
      return attp_build_value_cmd(
          tcb.payload_size, op_code, p_msg->attr_value.handle, offset,
          p_msg->attr_value.len, p_msg->attr_value.value);

    case GATT_RSP_WRITE:
      return attp_build_opcode_cmd(op_code);

    case GATT_RSP_ERROR:
      return attp_build_err_cmd(p_msg->error.cmd_code, p_msg->error.handle,
                                p_msg->error.reason);

    case GATT_RSP_EXEC_WRITE:
      return attp_build_exec_write_cmd(op_code, 0);

    case GATT_RSP_MTU:
      return attp_build_mtu_cmd(op_code, p_msg->mtu);

    default:
      LOG(FATAL) << "attp_build_sr_msg: unknown op code = " << +op_code;
      return nullptr;
  }
}

/*******************************************************************************
 *
 * Function         attp_send_sr_msg
 *
 * Description      This function sends the server response or indication
 *                  message to client.
 *
 * Parameter        p_tcb: pointer to the connecton control block.
 *                  p_msg: pointer to message parameters structure.
 *
 * Returns          GATT_SUCCESS if sucessfully sent; otherwise error code.
 *
 *
 ******************************************************************************/
tGATT_STATUS attp_send_sr_msg(tGATT_TCB& tcb, uint16_t cid, BT_HDR* p_msg) {
  if (p_msg == NULL) return GATT_NO_RESOURCES;

  p_msg->offset = L2CAP_MIN_OFFSET;
  return attp_send_msg_to_l2cap(tcb, cid, p_msg);
}

/*******************************************************************************
 *
 * Function         attp_cl_send_cmd
 *
 * Description      Send a ATT command or enqueue it.
 *
 * Returns          GATT_SUCCESS if command sent
 *                  GATT_CONGESTED if command sent but channel congested
 *                  GATT_CMD_STARTED if command queue up in GATT
 *                  GATT_ERROR if command sending failure
 *
 ******************************************************************************/
tGATT_STATUS attp_cl_send_cmd(tGATT_TCB& tcb, tGATT_CLCB* p_clcb,
                              uint8_t cmd_code, BT_HDR* p_cmd) {
  cmd_code &= ~GATT_AUTH_SIGN_MASK;

  if (gatt_tcb_is_cid_busy(tcb, p_clcb->cid) &&
      cmd_code != GATT_HANDLE_VALUE_CONF) {
    gatt_cmd_enq(tcb, p_clcb, true, cmd_code, p_cmd);
    LOG_DEBUG("Enqueued ATT command");
    return GATT_CMD_STARTED;
  }

  tGATT_STATUS att_ret = attp_send_msg_to_l2cap(tcb, p_clcb->cid, p_cmd);
  if (att_ret != GATT_CONGESTED && att_ret != GATT_SUCCESS) {
    LOG_WARN("Unable to send ATT command to l2cap layer");
    return GATT_INTERNAL_ERROR;
  }

  if (cmd_code == GATT_HANDLE_VALUE_CONF || cmd_code == GATT_CMD_WRITE) {
    return att_ret;
  }

  LOG_DEBUG("Starting ATT response timer");
  gatt_start_rsp_timer(p_clcb);
  gatt_cmd_enq(tcb, p_clcb, false, cmd_code, NULL);
  return att_ret;
}

/*******************************************************************************
 *
 * Function         attp_send_cl_confirmation_msg
 *
 * Description      This function sends the client confirmation
 *                  message to server.
 *
 * Parameter        p_tcb: pointer to the connection control block.
 *                  cid: channel id
 *
 * Returns          GATT_SUCCESS if successfully sent; otherwise error code.
 *
 *
 ******************************************************************************/
tGATT_STATUS attp_send_cl_confirmation_msg(tGATT_TCB& tcb, uint16_t cid) {
  BT_HDR* p_cmd = NULL;
  p_cmd = attp_build_opcode_cmd(GATT_HANDLE_VALUE_CONF);

  if (p_cmd == NULL) return GATT_NO_RESOURCES;

  /* no pending request or value confirmation */
  tGATT_STATUS att_ret = attp_send_msg_to_l2cap(tcb, cid, p_cmd);
  if (att_ret != GATT_CONGESTED && att_ret != GATT_SUCCESS) {
    return GATT_INTERNAL_ERROR;
  }

  return att_ret;
}

/*******************************************************************************
 *
 * Function         attp_send_cl_msg
 *
 * Description      This function sends the client request or confirmation
 *                  message to server.
 *
 * Parameter        p_tcb: pointer to the connectino control block.
 *                  p_clcb: clcb
 *                  op_code: message op code.
 *                  p_msg: pointer to message parameters structure.
 *
 * Returns          GATT_SUCCESS if sucessfully sent; otherwise error code.
 *
 *
 ******************************************************************************/
tGATT_STATUS attp_send_cl_msg(tGATT_TCB& tcb, tGATT_CLCB* p_clcb,
                              uint8_t op_code, tGATT_CL_MSG* p_msg) {
  BT_HDR* p_cmd = NULL;
  uint16_t offset = 0, handle;

  if (!p_clcb) {
    LOG_ERROR("Missing p_clcb");
    return GATT_ILLEGAL_PARAMETER;
  }

  uint16_t payload_size = gatt_tcb_get_payload_size_tx(tcb, p_clcb->cid);

  switch (op_code) {
    case GATT_REQ_MTU:
      if (p_msg->mtu > GATT_MAX_MTU_SIZE) {
        LOG_WARN(
            "GATT message MTU is larger than max GATT MTU size op_code:%hhu",
            op_code);
        return GATT_ILLEGAL_PARAMETER;
      }

      tcb.payload_size = p_msg->mtu;
      p_cmd = attp_build_mtu_cmd(GATT_REQ_MTU, p_msg->mtu);
      break;

    case GATT_REQ_FIND_INFO:
    case GATT_REQ_READ_BY_TYPE:
    case GATT_REQ_READ_BY_GRP_TYPE:
      if (!GATT_HANDLE_IS_VALID(p_msg->browse.s_handle) ||
          !GATT_HANDLE_IS_VALID(p_msg->browse.e_handle) ||
          p_msg->browse.s_handle > p_msg->browse.e_handle) {
        LOG_WARN("GATT message has invalid handle op_code:%hhu", op_code);
        return GATT_ILLEGAL_PARAMETER;
      }

      p_cmd = attp_build_browse_cmd(op_code, p_msg->browse.s_handle,
                                    p_msg->browse.e_handle, p_msg->browse.uuid);
      break;

    case GATT_REQ_READ_BLOB:
      offset = p_msg->read_blob.offset;
      FALLTHROUGH_INTENDED; /* FALLTHROUGH */
    case GATT_REQ_READ:
      handle =
          (op_code == GATT_REQ_READ) ? p_msg->handle : p_msg->read_blob.handle;
      /*  handle checking */
      if (!GATT_HANDLE_IS_VALID(handle)) {
        LOG_WARN("GATT message has invalid handle op_code:%hhu", op_code);
        return GATT_ILLEGAL_PARAMETER;
      }

      p_cmd = attp_build_handle_cmd(op_code, handle, offset);
      break;

    case GATT_REQ_PREPARE_WRITE:
      offset = p_msg->attr_value.offset;
      FALLTHROUGH_INTENDED; /* FALLTHROUGH */
    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
    case GATT_SIGN_CMD_WRITE:
      if (!GATT_HANDLE_IS_VALID(p_msg->attr_value.handle)) {
        LOG_WARN("GATT message has invalid handle op_code:%hhu", op_code);
        return GATT_ILLEGAL_PARAMETER;
      }

      p_cmd = attp_build_value_cmd(
          payload_size, op_code, p_msg->attr_value.handle, offset,
          p_msg->attr_value.len, p_msg->attr_value.value);
      break;

    case GATT_REQ_EXEC_WRITE:
      p_cmd = attp_build_exec_write_cmd(op_code, p_msg->exec_write);
      break;

    case GATT_REQ_FIND_TYPE_VALUE:
      p_cmd = attp_build_read_by_type_value_cmd(payload_size,
                                                &p_msg->find_type_value);
      break;

    case GATT_REQ_READ_MULTI:
    case GATT_REQ_READ_MULTI_VAR:
      p_cmd = attp_build_read_multi_cmd(op_code, payload_size,
                                        p_msg->read_multi.num_handles,
                                        p_msg->read_multi.handles);
      break;

    default:
      break;
  }

  if (p_cmd == NULL) {
    LOG_WARN(
        "Unable to build proper GATT message to send to peer device "
        "op_code:%hhu",
        op_code);
    return GATT_NO_RESOURCES;
  }

  return attp_cl_send_cmd(tcb, p_clcb, op_code, p_cmd);
}
