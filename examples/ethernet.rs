//! Example USB echo
//!
//! Uses a Usb CDC class to echo the data received by the host
//!

#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

use cortex_m::singleton;
use portenta_h7::{
    board::{self, Board, UsbBus, USB},
    hal::ethernet,
    hal::ethernet::PHY,
    led::{self, Led},
    log, log_init,
};
use rtic::app;
use systick_monotonic::Systick;
use usb_device::prelude::*;
use usbd_serial::CdcAcmClass;

use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::time::Instant;
use smoltcp::wire::{HardwareAddress, IpAddress, IpCidr};

const USB_BUS_BUFFER_SIZE: usize = 1024;
static mut USB_BUS_BUFFER: [u32; USB_BUS_BUFFER_SIZE] = [0u32; USB_BUS_BUFFER_SIZE];
const USB_HS_MAX_PACKET_SIZE: usize = 512;
static mut USB_APP_BUFFER: [u8; USB_HS_MAX_PACKET_SIZE] = [0u8; USB_HS_MAX_PACKET_SIZE];

/// TIME is an atomic u32 that counts milliseconds.
static TIME: AtomicU32 = AtomicU32::new(0);

// This data will be held by Net through a mutable reference
pub struct NetStorageStatic<'a> {
    socket_storage: [SocketStorage<'a>; 8],
}
// MaybeUninit allows us write code that is correct even if STORE is not
// initialised by the runtime
static mut STORE: MaybeUninit<NetStorageStatic> = MaybeUninit::uninit();

pub struct Net<'a> {
    iface: Interface,
    ethdev: ethernet::EthernetDMA<4, 4>,
    sockets: SocketSet<'a>,
}

impl<'a> Net<'a> {
    pub fn new(
        store: &'a mut NetStorageStatic<'a>,
        mut ethdev: ethernet::EthernetDMA<4, 4>,
        ethernet_addr: HardwareAddress,
        now: Instant,
    ) -> Self {
        let config = Config::new(ethernet_addr);
        let mut iface = Interface::new(config, &mut ethdev, now);
        // Set IP address
        iface.update_ip_addrs(|addrs| {
            let _ = addrs.push(IpCidr::new(IpAddress::v4(192, 168, 1, 99), 0));
        });

        let sockets = SocketSet::new(&mut store.socket_storage[..]);

        Net::<'a> {
            iface,
            ethdev,
            sockets,
        }
    }

    /// Polls on the ethernet interface. You should refer to the smoltcp
    /// documentation for poll() to understand how to call poll efficiently
    pub fn poll(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        self.iface
            .poll(timestamp, &mut self.ethdev, &mut self.sockets);
    }
}

#[app(device = portenta_h7::hal::pac, peripherals = false)]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led_blue: led::user::Blue,
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        usb_serial_port: CdcAcmClass<'static, UsbBus<USB>>,
        net: Net<'static>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1_000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        log_init!();

        let mono = Systick::new(cx.core.SYST, board::CORE_FREQUENCY.raw());

        // Get board resources
        let Board {
            led_blue,
            usb,
            ethernet: (eth_dma, eth_mac),
            ..
        } = Board::take();

        // Init USB stack
        let usb_bus = singleton!(
            : usb_device::class_prelude::UsbBusAllocator<UsbBus<USB>> =
                UsbBus::new(usb, unsafe { &mut USB_BUS_BUFFER })
        )
        .unwrap();
        let usb_serial_port = usbd_serial::CdcAcmClass::new(usb_bus, USB_HS_MAX_PACKET_SIZE as u16);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1234, 0xABCD))
            .manufacturer("example")
            .product("usb-echo")
            .serial_number("0123456789ABCDEF")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .max_packet_size_0(64)
            .max_power(100)
            .build();

        // Initialise ethernet PHY...
        let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac);
        lan8742a.phy_reset();
        lan8742a.phy_init();
        // The eth_dma should not be used until the PHY reports the link is up

        unsafe { ethernet::enable_interrupt() };

        // unsafe: mutable reference to static storage, we only do this once
        let store = unsafe {
            let store_ptr = STORE.as_mut_ptr();

            // Initialise the socket_storage field. Using `write` instead of
            // assignment via `=` to not call `drop` on the old, uninitialised
            // value
            addr_of_mut!((*store_ptr).socket_storage).write([SocketStorage::EMPTY; 8]);

            // Now that all fields are initialised we can safely use
            // assume_init_mut to return a mutable reference to STORE
            STORE.assume_init_mut()
        };

        let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&board::MAC_ADDRESS);
        let net = Net::new(store, eth_dma, mac_addr.into(), Instant::ZERO);

        (
            Shared {},
            Local {
                led_blue,
                usb_dev,
                usb_serial_port,
                net,
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = OTG_HS, local = [led_blue, usb_dev, usb_serial_port])]
    fn usb_process(cx: usb_process::Context) {
        let (usb_dev, usb_serial_port) = (cx.local.usb_dev, cx.local.usb_serial_port);
        let previous_state = usb_dev.state();

        // Trigger internal state machine. It should be called either from ISR on USB event,
        // or every 10 ms from normal execution context
        if usb_dev.poll(&mut [usb_serial_port]) {
            // Read from reception fifo
            match usb_serial_port.read_packet(unsafe { &mut USB_APP_BUFFER[..] }) {
                Ok(cnt) if cnt > 0 => {
                    #[cfg(debug_assertions)]
                    log!(
                        "Received {} bytes: {}",
                        cnt,
                        core::str::from_utf8(unsafe { &USB_APP_BUFFER[..cnt] })
                            .unwrap_or("not valid")
                    );
                    // Send back received data
                    match usb_serial_port.write_packet(unsafe { &USB_APP_BUFFER[..cnt] }) {
                        Ok(_) => (),
                        Err(err) => {
                            log!("Error in transmission: {:?}", err)
                        }
                    }
                }
                _ => (),
            }
        }

        // Signal enumeration status
        match usb_dev.state() {
            // Transition to enumeration
            UsbDeviceState::Configured if previous_state == UsbDeviceState::Addressed => {
                log!("Enumeration completed");
                cx.local.led_blue.on();
            }
            // Already enumerated
            UsbDeviceState::Configured => {}
            // Enumeration lost
            _ if previous_state == UsbDeviceState::Configured => {
                log!("Enumeration lost");
                cx.local.led_blue.off();
            }
            _ => (),
        }
    }

    #[task(binds = ETH, local = [net])]
    fn ethernet_event(ctx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }

        let time = TIME.load(Ordering::Relaxed);
        ctx.local.net.poll(time as i64);
    }
}
