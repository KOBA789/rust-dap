// Copyright 2021 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#![no_std]
#![no_main]

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rust_dap::DapError;
use rust_dap::SwdIo;
use rust_dap::SwdIoConfig;
use rust_dap::SwdRequest;
use rust_dap::DAP_TRANSFER_MISMATCH;
use rust_dap::DAP_TRANSFER_OK;
use rust_dap::USB_CLASS_MISCELLANEOUS;
use rust_dap::USB_PROTOCOL_IAD;
use rust_dap::USB_SUBCLASS_COMMON;
use stm32f1xx_hal as hal;
use cortex_m_rt::entry;
use cortex_m::asm::delay;

use hal::gpio::{Floating, Input, OpenDrain, Output, PushPull};
use hal::pac::{interrupt, CorePeripherals};
use hal::prelude::*;

use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;

use rust_dap::CmsisDap;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;

type SwdInputMode = Input<Floating>;
type SwdOutputMode = Output<PushPull>;
type SwdIoPin<MODE> = hal::gpio::gpiob::PB13<MODE>;
type SwClkPin<MODE> = hal::gpio::gpiob::PB14<MODE>;
type SwdIoInputPin = SwdIoPin<SwdInputMode>;
type SwdIoOutputPin = SwdIoPin<SwdOutputMode>;
type SwClkInputPin = SwClkPin<SwdInputMode>;
type SwClkOutputPin = SwClkPin<SwdOutputMode>;

struct BluePillSwdIo {
    swdio_in: Option<SwdIoInputPin>,
    swdio_out: Option<SwdIoOutputPin>,
    swclk_in: Option<SwClkInputPin>,
    swclk_out: Option<SwClkOutputPin>,
}

impl BluePillSwdIo {
    fn swclk_out(&mut self) -> Option<&mut SwClkOutputPin> {
        self.swclk_out.as_mut()
    }
    fn swdio_out(&mut self) -> Option<&mut SwdIoOutputPin> {
        self.swdio_out.as_mut()
    }
    fn swdio_in(&mut self) -> Option<&mut SwdIoInputPin> {
        self.swdio_in.as_mut()
    }

    fn clock_wait(&self, config: &SwdIoConfig) {
        cycle_delay(config.clock_wait_cycles);
    }
    fn cycle_clock(&mut self, config: &SwdIoConfig) {
        self.swclk_out().and_then(|p| p.set_low().ok());
        self.clock_wait(config);
        self.swclk_out().and_then(|p| p.set_high().ok());
        self.clock_wait(config);
    }
    fn turn_around(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.turn_around_cycles {
            self.cycle_clock(config);
        }
    }
    fn idle_cycle(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.idle_cycles {
            self.write_bit(config, false);
        }
    }
    fn write_bit(&mut self, config: &SwdIoConfig, value: bool) {
        self.swdio_out().and_then(|p| {
            if value {
                p.set_high().ok()
            } else {
                p.set_low().ok()
            }
        });
        self.swclk_out().and_then(|p| p.set_low().ok());
        self.clock_wait(config);
        self.swclk_out().and_then(|p| p.set_high().ok());
        self.clock_wait(config);
    }
    fn read_bit(&mut self, config: &SwdIoConfig) -> bool {
        self.swclk_out().and_then(|p| p.set_low().ok());
        self.clock_wait(config);
        let value = self
            .swdio_in()
            .map_or_else(|| false, |p| p.is_high().unwrap_or(false));
        self.swclk_out().and_then(|p| p.set_high().ok());
        self.clock_wait(config);
        value
    }
    fn set_swdio(&mut self, value: bool) {
        self.swdio_out().and_then(|p| {
            if value {
                p.set_high().ok()
            } else {
                p.set_low().ok()
            }
        });
    }
    #[allow(dead_code)]
    fn get_timestamp(&mut self) -> u32 {
        0
    }
}

impl SwdIo for BluePillSwdIo {
    fn connect(&mut self) {
        let port = unsafe { PORT.as_mut() };
        port.map(|port| {
            if let Some(old) = self.swdio_in.take() {
                let mut new = old.into_push_pull_output(port);
                new.set_low().ok();
                self.swdio_out = Some(new);
            }
            if let Some(old) = self.swclk_in.take() {
                let mut new = old.into_push_pull_output(port);
                new.set_low().ok();
                self.swclk_out = Some(new);
            }
        });
    }
    fn disconnect(&mut self) {
        let port = unsafe { PORT.as_mut() };
        port.map(|port| {
            if let Some(old) = self.swdio_out.take() {
                let new = old.into_floating_input(port);
                self.swdio_in = Some(new);
            }
            if let Some(old) = self.swclk_out.take() {
                let new = old.into_floating_input(port);
                self.swclk_in = Some(new);
            }
        });
    }
    fn swj_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut index = 0;
        let mut value = 0;
        let mut bits = 0;
        let mut count = count;

        while count > 0 {
            count -= 1;
            if bits == 0 {
                value = data[index];
                index += 1;
                bits = 8;
            }
            self.write_bit(config, value & 1 != 0);
            value >>= 1;
            bits -= 1;
        }
    }
    fn swd_read_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &mut [u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = 0;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                let bit_value = self.read_bit(config);
                value = if bit_value {
                    (value >> 1) | 0x80
                } else {
                    value >> 1
                };
            }
            value >>= bits;
            data[index] = value;
            index += 1;
        }
    }

    fn swd_write_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = data[index];
            index += 1;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                self.write_bit(config, value & 1 != 0);
                value >>= 1;
            }
        }
    }

    fn swd_transfer(
        &mut self,
        config: &SwdIoConfig,
        request: SwdRequest,
        data: u32,
    ) -> core::result::Result<u32, rust_dap::DapError> {
        // write request
        self.enable_output();
        {
            let mut parity = false;
            self.write_bit(config, true); // Start
            let bit = request.contains(SwdRequest::APnDP);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::RnW);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::A2);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::A3);
            self.write_bit(config, bit);
            parity ^= bit;
            self.write_bit(config, parity); // Parity
            self.write_bit(config, false); // Stop
            self.write_bit(config, true); // Park
        }

        // turnaround + read ack.
        self.disable_output();
        self.turn_around(config);
        let ack = {
            let mut ack = 0u8;
            ack |= if self.read_bit(config) { 0b001 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b010 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b100 } else { 0b000 };
            ack
        };
        if ack == rust_dap::DAP_TRANSFER_OK {
            let ack = if request.contains(SwdRequest::RnW) {
                // READ request
                let mut value = 0u32;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = self.read_bit(config);
                    parity ^= bit;
                    value = (value >> 1) | if bit { 0x80000000 } else { 0x00000000 };
                }
                let parity_expected = self.read_bit(config);
                self.turn_around(config);
                self.enable_output();
                if parity == parity_expected {
                    Ok(value)
                } else {
                    Err(DapError::SwdError(DAP_TRANSFER_MISMATCH))
                }
            } else {
                // WRITE request
                self.turn_around(config);
                self.enable_output();
                let mut value = data;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = value & 1 != 0;
                    self.write_bit(config, bit);
                    parity ^= bit;
                    value >>= 1;
                }
                self.write_bit(config, parity);
                Ok(0)
            };
            // TODO: capture timestamp
            self.idle_cycle(config);
            self.set_swdio(true);
            return ack;
        }

        // An error occured.
        if ack == rust_dap::DAP_TRANSFER_WAIT || ack == rust_dap::DAP_TRANSFER_FAULT {
            self.disable_output();
            if config.always_generate_data_phase && request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.cycle_clock(config);
                }
            }
            self.turn_around(config);
            self.enable_output();
            if config.always_generate_data_phase && !request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.write_bit(config, false);
                }
            }
            self.set_swdio(true);
            return Err(DapError::SwdError(ack));
        }

        // Protocol error
        self.turn_around(config);
        for _ in 0..33 {
            self.cycle_clock(config);
        }
        self.enable_output();
        self.set_swdio(true);
        return Err(DapError::SwdError(ack));
    }

    fn enable_output(&mut self) {
        let port = unsafe { PORT.as_mut() };
        port.map(|port| {
            if let Some(old) = self.swdio_in.take() {
                let mut new = old.into_push_pull_output(port);
                new.set_low().ok();
                self.swdio_out = Some(new);
            }
        });
    }

    fn disable_output(&mut self) {
        let port = unsafe { PORT.as_mut() };
        port.map(|port| {
            if let Some(old) = self.swdio_out.take() {
                let new = old.into_floating_input(port);
                self.swdio_in = Some(new);
            }
        });
    }
}

#[entry]
fn main() -> ! {
    let mut peripherals = hal::pac::Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut flash = peripherals.FLASH.constrain();
    let mut rcc = peripherals.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);
    assert!(clocks.usbclk_valid());

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low();
    delay(clocks.sysclk().0 / 100);

    let usb = hal::usb::Peripheral {
        usb: peripherals.USB,
        pin_dm: gpioa.pa11,
        pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
    };
    let usb_bus = UsbBus::new(usb);

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(usb_bus);
        USB_ALLOCATOR.as_ref().unwrap()
    };

    let swdio = BluePillSwdIo {
        swclk_in: Some(gpiob.pb14),
        swdio_in: Some(gpiob.pb13),
        swclk_out: None,
        swdio_out: None,
    };

    let crh = gpiob.crh;

    unsafe {
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_DAP = Some(CmsisDap::new(&bus_allocator, swdio));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x6666, 0x4444))
                .manufacturer("fugafuga.org")
                .product("CMSIS-DAP")
                .serial_number("test")
                .device_class(USB_CLASS_MISCELLANEOUS)
                .device_class(USB_SUBCLASS_COMMON)
                .device_protocol(USB_PROTOCOL_IAD)
                .composite_with_iads()
                .max_packet_size_0(64)
                .build(),
        );
        LED = Some(gpioc.pc13.into_open_drain_output(&mut gpioc.crh));
    }

    unsafe {
        core.NVIC.set_priority(interrupt::USB_HP_CAN_TX, 1);
        core.NVIC.set_priority(interrupt::USB_LP_CAN_RX0, 1);
        NVIC::unmask(interrupt::USB_HP_CAN_TX);
        NVIC::unmask(interrupt::USB_LP_CAN_RX0);
    }

    unsafe {
        PORT = Some(crh);
    }

    loop {
        // unsafe {
        //     USB_DAP.as_mut().map(|dap| {
        //         let _ = dap.process();
        //     });
        // }
        cycle_delay(15 * 1024 * 1024);
    }
}

static mut PORT: Option<hal::gpio::gpiob::CRH> = None;
static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus<hal::usb::Peripheral>>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus<hal::usb::Peripheral>>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus<hal::usb::Peripheral>>> = None;
static mut USB_DAP: Option<CmsisDap<UsbBus<hal::usb::Peripheral>, BluePillSwdIo, 64>> = None;
static mut LED: Option<hal::gpio::gpioc::PC13<Output<OpenDrain>>> = None;

fn poll_usb() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                USB_DAP.as_mut().map(|dap| {
                    usb_dev.poll(&mut [serial, dap]);

                    dap.process().ok();

                    let mut buf = [0u8; 64];

                    if let Ok(count) = serial.read(&mut buf) {
                        for (i, c) in buf.iter().enumerate() {
                            if i >= count {
                                break;
                            }
                            serial.write(&[c.clone()]).unwrap();
                            LED.as_mut().map(|led| led.toggle());
                        }
                    };
                });
            });
        });
    };
}

#[interrupt]
fn USB_HP_CAN_TX() {
    poll_usb();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    poll_usb();
}
