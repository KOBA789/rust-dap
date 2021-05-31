#![allow(non_upper_case_globals)]

use core::borrow::Borrow;
use core::borrow::BorrowMut;
use core::mem;
use core::convert::TryInto;

use usb_device::class_prelude::*;
use usb_device::Result;
use crate::cursor::{CursorRead, CursorWrite, BufferCursor, CursorError};
use crate::interface::*;
use num_enum::{IntoPrimitive, TryFromPrimitive, TryFromPrimitiveError};
use bitflags::bitflags;

#[derive(IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
enum DapCommandId {
    Info = 0x00,
    HostStatus = 0x01,
    Connect = 0x02,
    Disconnect = 0x03,
    TransferConfigure = 0x04,
    Transfer = 0x05,
    TransferBlock = 0x06,
    TransferAbort = 0x07,
    WriteAbort = 0x08,
    Delay = 0x09,
    ResetTarget = 0x0a,
    SWJPins = 0x10,
    SWJClock = 0x11,
    SWJSequence = 0x12,
    SWDConfigure = 0x13,
    SWDSequence = 0x1d,
}

#[derive(IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
enum DapInfoId {
    Vendor = 1,
    Product = 2,
    SerialNumber = 3,
    CmsisDapVer = 4,
    DeviceVendor = 5,
    DeviceName = 6,
    Capabilities = 0xf0,
    TimeStampClock = 0xf1,
    SwoBufferSize = 0xf2,
    PacketCount = 0xfe,
    PacketSize = 0xff,
}


bitflags! {
    pub struct SwdRequest: u8 {
        const APnDP = 0b00000001;
        const RnW = 0b00000010;
        const A2 = 0b00000100;
        const A3 = 0b00001000;
        const RESEND = Self::A3.bits;
        const RDBUFF = Self::A2.bits | Self::A3.bits;
        const TRANSFER_MATCH_VALUE = 0b00010000;
        const TRANSFER_MATCH_MASK = 0b00100000;
        const TRANSFER_TIMESTAMP = 0b10000000;
    }
}

pub const DAP_TRANSFER_OK: u8 = 0x01;
pub const DAP_TRANSFER_WAIT: u8 = 0x02;
pub const DAP_TRANSFER_FAULT: u8 = 0x04;
pub const DAP_TRANSFER_ERROR: u8 = 0x08;
pub const DAP_TRANSFER_MISMATCH: u8 = 0x10;


#[derive(Clone, Copy)]
pub struct SwdIoConfig
{
    pub clock_wait_cycles: u32,
    pub idle_cycles: u32,
    pub turn_around_cycles: u32,
}

pub trait SwdIo {
    fn connect(&mut self);
    fn disconnect(&mut self);
    fn swj_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]);
    fn swd_read_sequence(&mut self, config: &SwdIoConfig,count: usize, data: &mut [u8]);
    fn swd_write_sequence(&mut self, config: &SwdIoConfig,count: usize, data: &[u8]);
    fn swd_transfer(&mut self, config: &SwdIoConfig, request: SwdRequest, data: u32) -> core::result::Result<u32, DapError>;
    fn enable_output(&mut self);
    fn disable_output(&mut self);
}

const DAP_OK: u8 = 0x00;
const DAP_ERROR: u8 = 0xff;
const SWD_SEQUENCE_CLOCK: u8 = 0x3f;
const SWD_SEQUENCE_DIN: u8 = 0x80;

fn write_buffer(buffer: &mut [u8], data: &[u8]) -> core::result::Result<usize, CursorError> {
    let mut writer = BufferCursor::new(buffer);
    writer.write(data).map(|_| data.len())
}

pub enum DapError {
    InvalidCommand,
    InvalidDapInfoId,
    SwdError(u8),
    InternalError,
    ExceedRetryCount,
}

impl From<TryFromPrimitiveError<DapCommandId>> for DapError {
    fn from(_: TryFromPrimitiveError<DapCommandId>) -> Self {
        Self::InvalidCommand
    }
}
impl From<TryFromPrimitiveError<DapInfoId>> for DapError {
    fn from(_: TryFromPrimitiveError<DapInfoId>) -> Self {
        Self::InvalidDapInfoId
    }
}
impl From<CursorError> for DapError {
    fn from(_: CursorError) -> Self {
        Self::InternalError
    }
}
impl From<UsbError> for DapError {
    fn from(_: UsbError) -> Self{
        Self::InternalError
    }
}

pub struct CmsisDap<'a, B, Swd, const MAX_PACKET_SIZE: usize>
where
    B: UsbBus,
    Swd: SwdIo,
{
    inner: CmsisDapInterface<'a, B>,
    io: Swd,
    next_in_packet: [u8; MAX_PACKET_SIZE],
    next_in_packet_size: Option<usize>,
    pending_out_packet: [u8; MAX_PACKET_SIZE],
    pending_out_packet_size: usize,
    config: CmsisDapConfig,
}

impl Default for SwdIoConfig {
    fn default() -> Self {
        Self {
            clock_wait_cycles: 100,
            idle_cycles: 0,
            turn_around_cycles: 1,
        }
    }
}

struct CmsisDapConfig
{
    swdio: SwdIoConfig,
    retry_count: u32,
    match_mask: u32,
    match_retry_count: u32,
}

impl Default for CmsisDapConfig {
    fn default() -> Self {
        Self {
            swdio: SwdIoConfig::default(),
            retry_count: 5,
            match_mask: 0xffffffff,
            match_retry_count: 5,
        }
    }
}

impl<B, Swd, const MAX_PACKET_SIZE: usize> CmsisDap<'_, B, Swd, MAX_PACKET_SIZE>
where 
    B: UsbBus,
    Swd: SwdIo
{
    pub fn new(alloc: &UsbBusAllocator<B>, io: Swd) -> CmsisDap<'_, B, Swd, MAX_PACKET_SIZE> 
    {
        CmsisDap {
            inner: CmsisDapInterface::new(alloc, 64),
            io: io,
            next_in_packet: unsafe { mem::MaybeUninit::uninit().assume_init() },
            next_in_packet_size: None,
            pending_out_packet: unsafe {mem::MaybeUninit::uninit().assume_init() },
            pending_out_packet_size: 0,
            config: CmsisDapConfig::default(),
        }
    }

    fn send_next_packet(&mut self) -> Result<()> {
        if let Some(size) = self.next_in_packet_size  {
            if size > 0 {
                self.inner.write_packet(&self.next_in_packet[0..size])?;
            } else {
                self.inner.write_packet(&[])?;
            }
            self.next_in_packet_size = None;
        }
        Ok(())
    }
    fn process_out_packet(&mut self) -> Result<()> {
        if self.pending_out_packet_size == 0 {
            self.pending_out_packet_size = self.inner.read_packet(&mut self.pending_out_packet).map_or(0, |size| size);
        }
        Ok(())
    }

    pub fn process(&mut self) -> core::result::Result<(), DapError> {
        self.process_out_packet().ok();
        if self.pending_out_packet_size > 0 && self.next_in_packet_size == None {
            let mut response_length = 0;
            let mut bytes_processed = 0;
            while bytes_processed < self.pending_out_packet_size {
                let command_byte = self.pending_out_packet[bytes_processed];
                self.next_in_packet[response_length] = command_byte;
                response_length += 1;
                bytes_processed += 1;

                if let Ok(command) = DapCommandId::try_from_primitive(command_byte) {
                    let request = self.pending_out_packet[bytes_processed..self.pending_out_packet_size].borrow();
                    let response = self.next_in_packet[response_length..].borrow_mut();

                    let result = match command {
                        DapCommandId::Info => dap_info(request, response),
                        DapCommandId::HostStatus => dap_host_status(request, response),
                        DapCommandId::Connect => dap_connect(&mut self.io, request, response),
                        DapCommandId::Disconnect => dap_disconnect(&mut self.io, request, response),
                        DapCommandId::Transfer => swd_transfer(&mut self.config, &mut self.io, request, response),
                        DapCommandId::SWJClock => swj_clock(request, response),
                        DapCommandId::SWJSequence => swj_sequence(&self.config, &mut self.io, request, response),
                        DapCommandId::SWDSequence => swd_sequence(&self.config, &mut self.io, request, response),
                        _ => { Err(DapError::InvalidCommand) },
                    };
                    if let Ok((request_processed, response_generated)) = result {
                        bytes_processed += request_processed;
                        response_length += response_generated;
                    }
                } else {
                    // Just ignore the command.
                }
            }
            self.next_in_packet_size = Some(response_length);
            self.pending_out_packet_size = 0;
        }
        self.send_next_packet().ok();
        Ok(())
    }
}

impl<B, Swd, const MAX_PACKET_SIZE: usize> UsbClass<B> for CmsisDap<'_, B, Swd, MAX_PACKET_SIZE>
where 
    B: UsbBus,
    Swd: SwdIo,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        self.inner.get_configuration_descriptors(writer)
    }
    fn reset(&mut self) {
        self.inner.reset()
    }
    fn control_in(&mut self, xfer: ControlIn<B>) {
        self.inner.control_in(xfer)
    }
    fn control_out(&mut self, xfer: ControlOut<B>) {
        self.inner.control_out(xfer)
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.inner.in_ep_address() {
            let _ = self.send_next_packet();
        }
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.inner.out_ep_address() {
            let _ = self.process_out_packet();
        }
    }
}

fn dap_info(request: &[u8], response: &mut [u8]) -> core::result::Result<(usize, usize), DapError> {
    if request.len() > 0 {
        let id = DapInfoId::try_from_primitive(request[0])?;
        let length = {
            let buffer = &mut response[1..];
            match id {
                DapInfoId::Vendor => {
                    write_buffer(buffer, "Hoge".as_bytes())?
                },
                DapInfoId::Product => {
                    write_buffer(buffer, "Fuga".as_bytes())?
                },
                DapInfoId::SerialNumber => {
                    write_buffer(buffer, "Piyo".as_bytes())?
                },
                DapInfoId::CmsisDapVer => {
                    write_buffer(buffer, "2.0.0".as_bytes())?
                },
                DapInfoId::Capabilities => {
                    buffer[0] = 0b0000_0001;
                    1 as usize
                },
                _ => {0 as usize},
            }
        };
        response[0] = (length) as u8;
        Ok((1, length + 1))
    }
    else {
        Err(DapError::InvalidCommand)
    }
}
fn dap_connect<Swd: SwdIo>(swdio: &mut Swd, request: &[u8], response: &mut [u8]) -> core::result::Result<(usize, usize), DapError> {
    if request.len() >= 1 {
        swdio.connect();
        response[0] = request[0];
        Ok((1, 1))
    }
    else {
        Err(DapError::InvalidCommand)
    }
}
fn dap_disconnect<Swd: SwdIo>(swdio: &mut Swd, request: &[u8], response: &mut [u8]) -> core::result::Result<(usize, usize), DapError> {
    if request.len() >= 1 {
        swdio.disconnect();
        response[0] = DAP_OK;
        Ok((0, 1))
    }
    else {
        Err(DapError::InvalidCommand)
    }
}
fn dap_host_status(request: &[u8], response: &mut [u8]) -> core::result::Result<(usize, usize), DapError> {
    if request.len() >= 1 {
        response[0] = DAP_OK;
        Ok((4, 1))
    }
    else {
        Err(DapError::InvalidCommand)
    }
}
fn swj_clock(request: &[u8], response: &mut [u8]) -> core::result::Result<(usize, usize), DapError> {
    if request.len() >= 4 {
        let _clock = u32::from_le_bytes(request.try_into().unwrap());
        response[0] = DAP_OK;
        Ok((4, 1))
    }
    else {
        Err(DapError::InvalidCommand)
    }
}
fn swj_sequence<Swd: SwdIo>(config: &CmsisDapConfig, swdio: &mut Swd, request: &[u8], response: &mut [u8]) -> core::result::Result<(usize, usize), DapError> {
    if request.len() > 0 {
        let count = if request[0] == 0 { 256 } else { request[0] as usize };
        let count_bytes = (count + 7) >> 3;
        if request.len() > count_bytes {
            swdio.swj_sequence(&config.swdio, count, &request[1..count_bytes+1]);
            response[0] = DAP_OK;
            Ok((count_bytes + 1, 1))
        } else {
            response[0] = DAP_ERROR;
            Ok((request.len(), 1))
        }
    }
    else {
        Err(DapError::InvalidCommand)
    }
}
fn swd_sequence<Swd: SwdIo>(config: &CmsisDapConfig, swdio: &mut Swd, request: &[u8], response: &mut [u8]) -> core::result::Result<(usize, usize), DapError> {
    if request.len() > 0 {
        let mut sequence_count = request[0];
        let mut request_index = 1;
        let mut response_index = 1;
        while sequence_count > 0 {
            sequence_count -= 1;
            let sequence_info = request[request_index];
            request_index += 1;

            let clock_count = if sequence_info & SWD_SEQUENCE_CLOCK == 0 { 64 } else { sequence_info & SWD_SEQUENCE_CLOCK } as usize;
            let bytes_count = (clock_count + 7) >> 3;
            let do_input = sequence_info & SWD_SEQUENCE_DIN != 0;

            if do_input {
                swdio.disable_output();
                swdio.swd_read_sequence(&config.swdio, clock_count, &mut response[response_index..]);
            } else {
                swdio.enable_output();
                swdio.swd_write_sequence(&config.swdio, clock_count, &request[request_index..]);
            }

            if sequence_count == 0 {
                swdio.enable_output()
            }

            if do_input {
                request_index += 1;
                response_index += bytes_count;
            } else {
                request_index = 1 + bytes_count;
            }
        }
        response[0] = DAP_OK;
        Ok((request_index, response_index))
    }
    else {
        Err(DapError::InvalidCommand)
    }
}

fn swd_transfer_inner_with_retry<Swd: SwdIo>(config: &CmsisDapConfig, swdio: &mut Swd, request: SwdRequest, data: u32) -> core::result::Result<u32, DapError> {
    let mut retry_count = 0;
    loop {
        match swdio.swd_transfer(&config.swdio, request, data) {
            Ok(value) => break Ok(value),
            Err(DapError::SwdError(err)) => {
                if err != DAP_TRANSFER_WAIT || retry_count == config.retry_count {
                    break Err(DapError::SwdError(err))
                }
                retry_count += 1;
            },
            Err(err) => break Err(err),
        }
    }
}

fn read_swd_request<C: CursorRead>(cursor: &mut C) -> SwdRequest {
    let mut buffer: [u8; 1] = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
    cursor.read(&mut buffer).ok();
    unsafe { SwdRequest::from_bits_unchecked(buffer[0]) }
}
fn read_u32<C: CursorRead>(cursor: &mut C) -> u32 {
    let mut value: [u8; 4] = unsafe{ core::mem::MaybeUninit::uninit().assume_init() };
    cursor.read(&mut value).ok();
    u32::from_le_bytes(value)
}
fn write_u32<C: CursorWrite>(cursor: &mut C, value: u32) {
    let bytes = u32::to_le_bytes(value);
    cursor.write(&bytes).ok();
}

fn swd_transfer<Swd: SwdIo>(config: &mut CmsisDapConfig, swdio: &mut Swd, request: &[u8], response: &mut [u8]) -> core::result::Result<(usize, usize), DapError> {
    if request.len() == 0 {
        return Err(DapError::InvalidCommand)
    } else {
        let mut request_count = request[0];
        let mut request = BufferCursor::new_with_position(request, 1);
        let (response_header, response_body) = response.split_at_mut(2);
        let mut response = BufferCursor::new(response_body);
        let mut posted_read = false;
        let mut write_issued = false;
        let mut response_count = 0;
        let mut last_response = 
        loop {
            if request_count == 0 {
                break Ok(0);
            }
            request_count -= 1;

            let swd_request = read_swd_request(&mut request);

            if swd_request.contains(SwdRequest::RnW) {
                if posted_read {
                    // read
                    let result = 
                    if swd_request.contains(SwdRequest::APnDP) && !swd_request.contains(SwdRequest::TRANSFER_MATCH_VALUE) {
                        swd_transfer_inner_with_retry(&config, swdio, swd_request, 0)
                    } else {
                        swd_transfer_inner_with_retry(&config, swdio, SwdRequest::RDBUFF | SwdRequest::RnW, 0)
                    };
                    if let Ok(value) = result {
                        write_u32(&mut response, value);
                    } else {
                        break result;  // Error
                    }
                }
                if swd_request.contains(SwdRequest::TRANSFER_MATCH_VALUE) {
                    let match_value = read_u32(&mut request);
                    let mut match_retry_count = 0;
                    
                    if swd_request.contains(SwdRequest::APnDP) {
                        let result = loop {
                            match swd_transfer_inner_with_retry(&config, swdio, swd_request, 0) {
                                Ok(value) => {
                                    if value & config.match_mask == match_value {
                                        break Ok(value)
                                    } else if match_retry_count == config.match_retry_count {
                                        break Err(DapError::ExceedRetryCount)
                                    }
                                    match_retry_count += 1;
                                },
                                Err(DapError::SwdError(err)) => {
                                    if err != DAP_TRANSFER_WAIT || match_retry_count == config.match_retry_count {
                                        break Err(DapError::SwdError(err))
                                    }
                                    match_retry_count += 1;
                                },
                                Err(err) => break Err(err),
                            }
                        };
                        if result.is_err() {
                            break result;  // Error
                        }
                    }

                } else {
                    if swd_request.contains(SwdRequest::APnDP) {
                        // Read AP
                        if !posted_read {
                            match swd_transfer_inner_with_retry(&config, swdio, swd_request, 0) {
                                Ok(_) => {
                                    posted_read = true;
                                },
                                Err(err) => {
                                    break Err(err); // Error
                                }
                            }
                        }
                    } else {
                        // Read DP
                        match swd_transfer_inner_with_retry(&config, swdio, swd_request, 0) {
                            Ok(value) => {
                                write_u32(&mut response, value);
                            },
                            Err(err) => {
                                break Err(err); // Error
                            },
                        }
                    }
                    write_issued = false;
                }
            } else {
                // Write register
                if posted_read {
                    // The last request is posted read, so we have to read-out the result.
                    match swd_transfer_inner_with_retry(&config, swdio, SwdRequest::RDBUFF | SwdRequest::RnW, 0) {
                        Ok(value) => {
                            write_u32(&mut response, value);
                        },
                        Err(err) => {
                            break Err(err);  // Error
                        },
                    }
                }

                let value = read_u32(&mut request);
                if swd_request.contains(SwdRequest::TRANSFER_MATCH_MASK) {
                    // Update match mask
                    config.match_mask = value
                } else {
                    // Write DP/AP
                    match swd_transfer_inner_with_retry(&config, swdio, swd_request, value) {
                        Ok(_) => {},
                        Err(err) => {
                            break Err(err);  // Error
                        },
                    }

                    write_issued = true;
                }
            }

            response_count += 1;
        };

        // process remaining requests.
        while request_count > 0 {
            request_count -= 1;
            let swd_request = read_swd_request(&mut request);

            if swd_request.contains(SwdRequest::RnW) {
                // Read request
                if swd_request.contains(SwdRequest::TRANSFER_MATCH_VALUE) {
                    // Read match value
                    read_u32(&mut request);
                }
            } else {
                // Write request
                read_u32(&mut request);
            }
        }

        if last_response.is_ok() {
            if posted_read || write_issued {
                match swd_transfer_inner_with_retry(&config, swdio, SwdRequest::RDBUFF | SwdRequest::RnW, 0) {
                    Ok(value) => {
                        if posted_read {
                            write_u32(&mut response, value);
                        }
                    },
                    Err(err) => {
                        last_response = Err(err);
                    }
                }
            }
        }

        response_header[0] = response_count as u8;
        response_header[1] = last_response.map_or_else(|e| if let DapError::SwdError(err) = e { err } else { DAP_TRANSFER_ERROR }, |_| DAP_TRANSFER_OK );
        Ok((request.get_position() , response.get_position() + 2))
    }
}