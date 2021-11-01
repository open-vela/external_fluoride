/****************************************************************************
 *
 *   Copyright (C) 2021 Xiaomi InC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __FLUORIDE_PORT_FLRD_H
#define __FLUORIDE_PORT_FLRD_H

#include <stddef.h>

#include <errno.h>
#include <stdio.h>

#include <zblue/bluetooth.h>
#include <zblue/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*bt_socket_data_cb_t)(void *handle,
                                         void *data, size_t len);
typedef int (*bt_socket_conn_cb_t)(void *handle, bool connected);

struct btav_source_cb_t {
    void(*a2dp_conn_state_cb)(bt_addr_t addr, uint8_t state);
    void (*a2dp_audio_state_cb) (bt_addr_t addr, uint8_t state);
    void (*a2dp_audio_config_cb) (bt_addr_t addr, uint32_t sample_rate, uint8_t channel_count);
    struct btav_source_cb_t *_next;
};

struct bthf_client_cb_t {
    void(*hfp_conn_state_cb)(bt_addr_t addr,uint8_t state);
    void(*hfp_audio_state_cb)(bt_addr_t addr,uint8_t state);
    void(*hfp_call_cb)(bt_addr_t addr, uint8_t call);
    void(*hfp_clip_cb)(bt_addr_t addr, char* number, char* name);
    struct bthf_client_cb_t *_next;
};

struct bt_adapter_cb_t {
    void(*device_found_cb)(bt_addr_t addr,char* name,int cod,int rssi);
    void(*discovery_state_changed_cb)(bool state);
    void(*bond_state_changed_cb)(bt_addr_t addr, uint8_t state);
    void(*acl_state_changed_cb)(bt_addr_t addr, uint8_t state);
};

void bt_socket_register_cb(bt_socket_conn_cb_t ccb,
                                 bt_socket_data_cb_t dcb);
int bt_socket_data_send(void *handle, const void *data, size_t len);


void a2dp_source_register_cb(struct btav_source_cb_t* cb);
int a2dp_source_connect(bt_addr_t addr);
int a2dp_source_disconnect(bt_addr_t addr);

void hfp_client_register_cb(struct bthf_client_cb_t* cb);
int hfp_client_connect(bt_addr_t addr);
int hfp_client_disconnect(bt_addr_t addr);
int hfp_client_dial(char *number);
int hfp_client_call_action(uint8_t action);

int hidc_send_report(uint8_t *report, uint16_t size);

void bt_adapter_register_cb(struct bt_adapter_cb_t* cb);
int bt_start_discovery(void);
int bt_stop_discovery(void);
int bt_create_bond(bt_addr_t addr,int transport);
int bt_remove_bond(bt_addr_t addr);
void bt_set_scan_mode(int scan_mode);

#ifdef __cplusplus
}
#endif


#endif /* __FLUORIDE_PORT_FLRD_H */
