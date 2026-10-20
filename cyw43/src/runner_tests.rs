//! Stopping the runner and initialising it again (`run_until`, `init` after a stop).

extern crate std;

use core::cell::Cell;
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use std::rc::Rc;

use embassy_net_driver_channel as ch;
use embassy_net_driver_channel::driver::{Driver, LinkState};

use super::*;
use crate::Cyw43439;
use crate::consts::Ioctl;
use crate::ioctl::IoctlType;

/// A device that has stopped answering: transfers complete (the host clocks them) but no
/// interrupt ever comes, and, with `hang_writes`, a write never completes either.
struct DeadBus {
    hang_writes: bool,
    init_ok: bool,
    inits: Rc<Cell<usize>>,
    writes: Rc<Cell<usize>>,
    powered_off: Rc<Cell<bool>>,
}

impl SealedBus for DeadBus {
    const TYPE: BusType = BusType::Spi;

    async fn init<'a>(&mut self, _: bool) -> crate::Result<()> {
        self.inits.set(self.inits.get() + 1);
        self.powered_off.set(false);
        if self.init_ok { Ok(()) } else { Err(crate::Error) }
    }
    async fn wlan_read(&mut self, _: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        Ok(())
    }
    async fn wlan_write(&mut self, _: &mut WriteBuffer) -> crate::Result<()> {
        self.writes.set(self.writes.get() + 1);
        if self.hang_writes {
            core::future::pending::<()>().await;
        }
        Ok(())
    }
    async fn bp_read(&mut self, _: u32, data: &mut [u8], _: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        data.fill(0);
        Ok(())
    }
    async fn bp_write(&mut self, _: u32, _: &[u8], _: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        Ok(())
    }
    async fn bp_read8(&mut self, _: u32) -> u8 {
        0
    }
    async fn bp_write8(&mut self, _: u32, _: u8) {}
    async fn bp_read16(&mut self, _: u32) -> u16 {
        0
    }
    async fn bp_write16(&mut self, _: u32, _: u16) {}
    async fn bp_read32(&mut self, _: u32) -> u32 {
        0
    }
    async fn bp_write32(&mut self, _: u32, _: u32) {}
    async fn read8(&mut self, _: u8, _: u32) -> u8 {
        0
    }
    async fn write8(&mut self, _: u8, _: u32, _: u8) {}
    async fn read16(&mut self, _: u8, _: u32) -> u16 {
        0
    }
    async fn write16(&mut self, _: u8, _: u32, _: u16) {}
    async fn read32(&mut self, _: u8, _: u32) -> u32 {
        0
    }
    async fn write32(&mut self, _: u8, _: u32, _: u32) {}
    async fn wait_for_event(&mut self) {
        core::future::pending::<()>().await;
    }
    async fn power_off(&mut self) {
        self.powered_off.set(true);
    }
}

struct Probe {
    inits: Rc<Cell<usize>>,
    writes: Rc<Cell<usize>>,
    powered_off: Rc<Cell<bool>>,
}

fn bus(hang_writes: bool, init_ok: bool) -> (DeadBus, Probe) {
    let p = Probe {
        inits: Rc::new(Cell::new(0)),
        writes: Rc::new(Cell::new(0)),
        powered_off: Rc::new(Cell::new(false)),
    };
    let b = DeadBus {
        hang_writes,
        init_ok,
        inits: p.inits.clone(),
        writes: p.writes.clone(),
        powered_off: p.powered_off.clone(),
    };
    (b, p)
}

/// A stop the test raises by setting the flag (and polling again).
fn stop_when(flag: &Cell<bool>) -> impl core::future::Future<Output = u8> + '_ {
    core::future::poll_fn(move |_| if flag.get() { Poll::Ready(7) } else { Poll::Pending })
}

fn cx() -> Context<'static> {
    Context::from_waker(Waker::noop())
}

macro_rules! runner {
    ($state:ident, $bus:expr, $device:ident, $runner:ident) => {
        let (channel, mut $device) = ch::new(
            &mut $state.net.ch,
            ch::driver::HardwareAddress::Ethernet([0; 6]),
            crate::MTU,
        );
        #[allow(unused_mut)]
        let mut $runner = Runner::new(
            channel,
            $bus,
            Cyw43439,
            &$state.ioctl_state,
            &$state.net.events,
            &$state.net.secure_network,
            #[cfg(feature = "bluetooth")]
            None,
        );
    };
}

#[test]
fn a_stop_ends_a_runner_waiting_for_credit_and_cancels_the_ioctl() {
    let mut state = crate::State::new();
    let (b, probe) = bus(false, true);
    runner!(state, b, device, runner);
    runner.ch.set_link_state(LinkState::Up);
    // No bus credit: the loop waits for an interrupt that never comes ("TX stalled").
    runner.sdpcm_seq_max = 0;
    runner.join_ok = true;

    let ioctls = &state.ioctl_state;
    let mut resp = [0xAAu8; 4];
    let mut cx = cx();
    {
        let mut ioctl = pin!(ioctls.do_ioctl(IoctlType::Get, Ioctl::GetVar, 0, &mut resp));
        assert!(ioctl.as_mut().poll(&mut cx).is_pending());

        let flag = Cell::new(false);
        {
            let mut run = pin!(runner.run_until(stop_when(&flag)));
            for _ in 0..3 {
                assert!(run.as_mut().poll(&mut cx).is_pending());
            }
            flag.set(true);
            assert_eq!(run.as_mut().poll(&mut cx), Poll::Ready(7));
        }

        // The caller of the ioctl is released with nothing, instead of waiting for ever.
        assert_eq!(ioctl.as_mut().poll(&mut cx), Poll::Ready(0));
    }
    assert_eq!(resp, [0xAA; 4], "an aborted ioctl leaves its buffer alone");
    assert!(device.link_state() == LinkState::Down);
    assert_eq!((runner.sdpcm_seq, runner.sdpcm_seq_max), (0, 1));
    assert!(!runner.join_ok);
    assert_eq!(probe.writes.get(), 0);
}

#[test]
fn a_stop_ends_a_runner_stuck_in_a_transfer() {
    let mut state = crate::State::new();
    let (b, probe) = bus(true, true);
    runner!(state, b, device, runner);
    let ioctls = &state.ioctl_state;
    let mut resp = [0u8; 4];
    let mut ioctl = pin!(ioctls.do_ioctl(IoctlType::Get, Ioctl::GetVar, 0, &mut resp));
    let mut cx = cx();
    assert!(ioctl.as_mut().poll(&mut cx).is_pending());

    let flag = Cell::new(false);
    {
        let mut run = pin!(runner.run_until(stop_when(&flag)));
        assert!(run.as_mut().poll(&mut cx).is_pending());
        // The runner took the ioctl and is inside its bus write, which never completes.
        assert_eq!(probe.writes.get(), 1);
        assert!(!ioctls.is_idle());
        flag.set(true);
        assert_eq!(run.as_mut().poll(&mut cx), Poll::Ready(7));
    }
    assert!(ioctls.is_idle());
    assert_eq!(ioctl.as_mut().poll(&mut cx), Poll::Ready(0));
    assert!(device.link_state() == LinkState::Down);
}

#[test]
fn the_runner_powers_off_and_initialises_again_after_a_stop() {
    let mut state = crate::State::new();
    let (b, probe) = bus(false, false);
    runner!(state, b, _device, runner);
    let fw = Aligned([0u8; 8]);
    let nvram = Aligned([0u8; 8]);
    let mut cx = cx();

    for round in 1..=3 {
        runner.sdpcm_seq = 0x42;
        runner.sdpcm_seq_max = 0x40;
        runner.mcast_gen = 9;
        runner.allmulti = true;
        let flag = Cell::new(true);
        assert_eq!(pin!(runner.run_until(stop_when(&flag))).poll(&mut cx), Poll::Ready(7));
        assert!(!probe.powered_off.get());
        assert_eq!(pin!(runner.power_off()).poll(&mut cx), Poll::Ready(()));
        assert!(probe.powered_off.get(), "power_off holds the device off");
        // A device that doesn't come back: init fails, and can be called again.
        runner.sdpcm_seq = 0x42;
        let r = pin!(runner.init(&fw, &nvram, None)).poll(&mut cx);
        assert!(matches!(r, Poll::Ready(Err(_))));
        assert_eq!(probe.inits.get(), round);
        assert_eq!((runner.sdpcm_seq, runner.sdpcm_seq_max), (0, 1));
        assert_eq!(runner.mcast_gen, 0);
        assert!(!runner.allmulti);
    }
}
