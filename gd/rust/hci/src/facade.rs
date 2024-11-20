//! HCI layer facade

use bt_hci_proto::empty::Empty;
use bt_hci_proto::facade::*;
use bt_hci_proto::facade_grpc::{create_hci_layer_facade, HciLayerFacade};

use futures::sink::SinkExt;
use gddi::{module, provides};
use tokio::runtime::Runtime;

use crate::HciExports;

use futures::prelude::*;
use grpcio::*;

use std::sync::Arc;

module! {
    facade_module,
    providers {
        HciLayerFacadeService => provide_facade,
    }
}

#[provides]
async fn provide_facade(hci_exports: HciExports, rt: Arc<Runtime>) -> HciLayerFacadeService {
    HciLayerFacadeService { hci_exports, rt }
}

/// HCI layer facade service
#[derive(Clone)]
pub struct HciLayerFacadeService {
    hci_exports: HciExports,
    rt: Arc<Runtime>,
}

impl HciLayerFacadeService {
    /// Convert to a grpc service
    pub fn create_grpc(self) -> grpcio::Service {
        create_hci_layer_facade(self)
    }
}

impl HciLayerFacade for HciLayerFacadeService {
    fn send_command_with_complete(
        &mut self,
        ctx: RpcContext<'_>,
        mut cmd: Command,
        sink: UnarySink<Empty>,
    ) {
        self.rt.block_on(
            self.hci_exports
                .enqueue_command_with_complete(cmd.take_payload().into()),
        );

        let f = sink
            .success(Empty::default())
            .map_err(|e: grpcio::Error| {
                println!(
                    "failed to handle enqueue_command_with_complete request: {:?}",
                    e
                )
            })
            .map(|_| ());

        ctx.spawn(f);
    }

    fn send_command_with_status(
        &mut self,
        ctx: RpcContext<'_>,
        mut cmd: Command,
        sink: UnarySink<Empty>,
    ) {
        self.rt.block_on(
            self.hci_exports
                .enqueue_command_with_complete(cmd.take_payload().into()),
        );

        let f = sink
            .success(Empty::default())
            .map_err(|e: grpcio::Error| {
                println!(
                    "failed to handle enqueue_command_with_status request: {:?}",
                    e
                )
            })
            .map(|_| ());

        ctx.spawn(f);
    }

    fn request_event(&mut self, ctx: RpcContext<'_>, code: EventRequest, sink: UnarySink<Empty>) {
        self.rt.block_on(
            self.hci_exports
                .register_event_handler(code.get_code() as u8),
        );

        let f = sink
            .success(Empty::default())
            .map_err(|e: grpcio::Error| {
                println!(
                    "failed to handle enqueue_command_with_status request: {:?}",
                    e
                )
            })
            .map(|_| ());

        ctx.spawn(f);
    }

    fn request_le_subevent(
        &mut self,
        _ctx: RpcContext<'_>,
        _code: EventRequest,
        _sink: UnarySink<Empty>,
    ) {
        unimplemented!()
    }

    fn send_acl(&mut self, _ctx: RpcContext<'_>, _data: AclPacket, _sink: UnarySink<Empty>) {
        unimplemented!()
    }

    fn stream_events(
        &mut self,
        _ctx: RpcContext<'_>,
        _req: Empty,
        mut resp: ServerStreamingSink<Event>,
    ) {
        let evt_rx = self.hci_exports.evt_rx.clone();

        self.rt.spawn(async move {
            while let Some(event) = evt_rx.lock().await.recv().await {
                let mut evt = Event::default();
                evt.set_payload(event.to_vec());
                resp.send((evt, WriteFlags::default())).await.unwrap();
            }
        });
    }

    fn stream_le_subevents(
        &mut self,
        _ctx: RpcContext<'_>,
        _req: Empty,
        mut _resp: ServerStreamingSink<LeSubevent>,
    ) {
        unimplemented!()
    }

    fn stream_acl(
        &mut self,
        _ctx: RpcContext<'_>,
        _req: Empty,
        mut _resp: ServerStreamingSink<AclPacket>,
    ) {
        unimplemented!()
    }
}
