#!/usr/bin/env python3
#
#   Copyright 2020 - The Android Open Source Project
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

from google.protobuf import empty_pb2 as empty_proto
from cert.event_stream import EventStream
from cert.closable import Closable
from cert.closable import safeClose
from hal import facade_pb2 as hal_facade


class PyHal(Closable):

    def __init__(self, device):
        self.device = device

        self.hci_event_stream = EventStream(self.device.hal.FetchHciEvent(empty_proto.Empty()))
        self.acl_stream = EventStream(self.device.hal.FetchHciAcl(empty_proto.Empty()))

        # We don't deal with SCO for now

    def close(self):
        safeClose(self.hci_event_stream)
        safeClose(self.acl_stream)

    def get_hci_event_stream(self):
        return self.hci_event_stream

    def get_acl_stream(self):
        return self.acl_stream

    def send_hci_command(self, command):
        self.device.hal.SendHciCommand(hal_facade.HciCommandPacket(payload=bytes(command)))

    def send_acl(self, acl):
        self.device.hal.SendHciAcl(hal_facade.HciAclPacket(payload=bytes(acl)))
