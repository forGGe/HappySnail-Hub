extern crate typenum;
extern crate num;
extern crate text_io;
extern crate failure;

use std::ops::Shr;
use std::vec::Vec;
use std::str;
use std::fs::File;
use std::io::{Read, Write, BufRead};
use std::io::prelude::*;
use std::io::BufReader;
use typenum::*;
use num::cast::AsPrimitive;
use failure::{Error, Fail};

#[derive(Debug, Fail)]
pub enum ModbusError {
    #[fail(display = "Invalid frame")]
    InvalidFrame,
    #[fail(display = "Invalid CRC")]
    InvalidCRC,
    #[fail(display = "Invalid Data")]
    InvalidData,
    #[fail(display = "Invalid function ID")]
    InvalidFnId,
    #[fail(display = "Invalid slave address")]
    InvalidAddr,
    #[fail(display = "Mismatch between response and request")]
    ReqRespMismatch,
    #[fail(display = "No such index")]
    NoSuchIdx,
    #[fail(display = "I/O Error")]
    IoErr,
}

// Trait for serialize Modbus PDUs
pub trait RequestPDU {
    type Resp: ResponsePDU;
    fn as_ascii(&self) -> String;
    fn lrc(&self, slave: u8) -> u8;
    fn fn_id() -> u8;
}

pub trait ResponsePDU: Sized {
    type Req;
    fn from_ascii(req: &Self::Req, packet: &String) -> Result<Self, ModbusError>;
}

// Calculates LRC from given byte array
fn generic_lrc(data: &[u8]) -> u8 {
    // ISO 1155
    // u16 helps to avoid using std::Wrapping

    let r = data.iter().fold(0u16, |acc, x| (acc + *x as u16) & 0xff);
    (((r ^ 0xff) + 1) & 0xff) as u8
}

// Extract individual bytes from a number
fn exract_byte<T>(x: T, shift: u8) -> u8
    where T: Shr<u8, Output = T> + AsPrimitive<u8>
{
    (x >> shift).as_()
}

// Most of Modbus functions are just a pair of numbers.
// This allows to process them in generic manner.
struct RequestPair(u16, u16);

impl RequestPair {
    fn lrc(&self, slave: u8, fn_id: u8) -> u8 {
        generic_lrc(
            &[
                slave,
                fn_id,
                exract_byte(self.0, 8), exract_byte(self.0, 0),
                exract_byte(self.1, 8), exract_byte(self.1, 0),
            ]
        )
    }

    fn as_ascii(&self, fn_id: u8) -> String {
        format!("{:02X}{:04X}{:04X}", fn_id, self.0, self.1)
    }
}

/*------------------------------------------------------------------------------------------------*/

// Abstract Modbus function represented by address and count pair.
#[derive(Clone, Copy, Debug)]
struct ReadAddrCountGeneric<FnID: Unsigned> {
    addr: u16,
    cnt:  u16,
    ph0: std::marker::PhantomData<FnID>,
}

impl<FnID: Unsigned> ReadAddrCountGeneric<FnID> {
    pub fn new(addr: u16, cnt: u16) -> Self {
        ReadAddrCountGeneric::<FnID> {
            addr,
            cnt,
            ph0: std::marker::PhantomData,
        }
    }

    fn fn_id() -> u8 {
        FnID::to_u8()
    }

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, self.cnt).as_ascii(Self::fn_id())
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, self.cnt).lrc(slave, Self::fn_id())
    }
}

#[derive(Clone, Debug)]
struct ReadAddrCountGenericResp {
    cnt: u16,
    data: Vec<u8>,
}

impl ReadAddrCountGenericResp {
    fn from_ascii(packet: &String, req_cnt: u16) -> Result<Self, ModbusError> {
        // Bytes that should be received, according to the request
        let req_count = (req_cnt / 8) as u8 + 1;
        // Actual bytes received
        let byte_cnt: u8;

        match u8::from_str_radix(&packet[0..2], 16) {
            Ok(c) => {
                byte_cnt = c;
            },
            Err(e) => {
                println!("failed to extract count from: {} with err {}", packet, e);
                return Err(ModbusError::InvalidFrame);
            },
        }

        if byte_cnt != req_count {
            println!("byte_cnt: {}, req_count: {}", byte_cnt, req_count);
            return Err(ModbusError::ReqRespMismatch);
        }

        let data: Vec<u8> = packet[2..].as_bytes().chunks(2).map(|chunk| {
            let str_chunk = str::from_utf8(chunk).unwrap();
            u8::from_str_radix(str_chunk, 16).unwrap_or_else(|e| {
                    println!("failed to extract object value from: {}, err: {}", str_chunk, e);
                    0
                }
            )
        }).collect();

        if data.len() != byte_cnt as usize {
            return Err(ModbusError::InvalidData);
        }

        Ok(Self { cnt: req_cnt, data })
    }

    fn object(&self, mut idx: u16, req_addr: u16) -> Result<bool, ModbusError> {
        if idx >= (self.cnt + req_addr) || idx < req_addr {
            return Err(ModbusError::NoSuchIdx);
        }

        idx -= req_addr;

        let byte = self.data[(idx / 8) as usize];
        Ok((byte & (1 << (idx % 8))) != 0)
    }
}

pub struct ReadAddrCountGeneric16BitResp {
    cnt: u8,
    data: Vec<u16>,
}

impl ReadAddrCountGeneric16BitResp {
    fn from_ascii(req_cnt: u16, packet: &String) -> Result<Self, ModbusError> {
        // Actual bytes received
        let byte_cnt: u8;

        match u8::from_str_radix(&packet[0..2], 16) {
            Ok(c) => {
                byte_cnt = c;
            },
            Err(e) => {
                println!("failed to extract count from: {} with err {}", packet, e);
                return Err(ModbusError::InvalidFrame);
            },
        }

        if (byte_cnt / 2) as u16 != req_cnt {
            println!("byte_cnt: {}, req_cnt: {}", byte_cnt, req_cnt);
            return Err(ModbusError::ReqRespMismatch);
        }

        let data: Vec<u16> = packet[2..].as_bytes().chunks(4).map(|chunk| {
            let str_chunk = str::from_utf8(chunk).unwrap();
            u16::from_str_radix(str_chunk, 16).unwrap_or_else(|e| {
                    println!("failed to extract object value from: {}, err: {}", str_chunk, e);
                    0
                }
            )
        }).collect();

        if data.len() * 2 != byte_cnt as usize {
            return Err(ModbusError::InvalidData);
        }

        Ok(Self { cnt: byte_cnt, data })
    }

    fn object(&self, idx: u8) -> Result<u16, ModbusError> {
        self.data.get(idx as usize).ok_or(ModbusError::NoSuchIdx).map(|v| *v)
    }
}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #1: Read Coils.
#[derive(Clone, Copy, Debug)]
pub struct ReadCoils(ReadAddrCountGeneric<U1>);

impl ReadCoils {
    pub fn new(addr: u16, cnt: u16) -> Self {
        Self(ReadAddrCountGeneric::<U1>::new(addr, cnt))
    }
}

impl RequestPDU for ReadCoils {
    type Resp = ReadCoilsResp;

    fn fn_id()                  -> u8       { ReadAddrCountGeneric::<U1>::fn_id() }
    fn as_ascii(&self)          -> String   { self.0.as_ascii() }
    fn lrc(&self, slave: u8)    -> u8       { self.0.lrc(slave) }
}

// Modbus function #1 response.
#[derive(Debug)]
pub struct ReadCoilsResp {
    resp: ReadAddrCountGenericResp,
    req: ReadCoils,
}

impl ResponsePDU for ReadCoilsResp {
    type Req = ReadCoils;

    fn from_ascii(req: &Self::Req, packet: &String) -> Result<Self, ModbusError> {
        Ok(Self { req: *req, resp: ReadAddrCountGenericResp::from_ascii(packet, req.0.cnt)? })
    }
}

impl ReadCoilsResp {
    fn coil(&self, mut idx: u16) -> Result<bool, ModbusError> {
        self.resp.object(idx, self.req.0.addr)
    }
}

#[cfg(test)]
#[test]
fn test_read_coil() {
    use crate::modbus::ResponsePDU;
    use crate::modbus::ReadCoils;
    use crate::modbus::ReadCoilsResp;

    // 11 coils, starting from 42:
    // on, off, on, off, off, on, on, on, off, on, on
    // in hex: (E5, 06)

    let req = ReadCoils::new(42, 11);
    let resp = ReadCoilsResp::from_ascii(&req, &"02E506".to_string()).unwrap();
    println!("{:?}", resp);

    assert_eq!(resp.resp.cnt,    11);
    assert_eq!(resp.resp.data[0], 0xe5);
    assert_eq!(resp.resp.data[1], 0x06);

    assert_eq!(resp.coil(42).unwrap(), true);
    assert_eq!(resp.coil(43).unwrap(), false);
    assert_eq!(resp.coil(44).unwrap(), true);
    assert_eq!(resp.coil(45).unwrap(), false);
    assert_eq!(resp.coil(46).unwrap(), false);
    assert_eq!(resp.coil(47).unwrap(), true);
    assert_eq!(resp.coil(48).unwrap(), true);
    assert_eq!(resp.coil(49).unwrap(), true);
    assert_eq!(resp.coil(50).unwrap(), false);
    assert_eq!(resp.coil(51).unwrap(), true);
    assert_eq!(resp.coil(52).unwrap(), true);

    assert_eq!(resp.coil(53).is_err(), true);
    assert_eq!(resp.coil(41).is_err(), true);
    assert_eq!(resp.coil(0).is_err(),  true);

}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #2: Read Discrete Inputs.
#[derive(Clone, Copy)]
pub struct ReadDiscreteInputs(ReadAddrCountGeneric<U2>);

impl RequestPDU for ReadDiscreteInputs {
    type Resp = ReadDiscreteInputsResp;

    fn fn_id()                  -> u8       { ReadAddrCountGeneric::<U2>::fn_id() }
    fn as_ascii(&self)          -> String   { self.0.as_ascii() }
    fn lrc(&self, slave: u8)    -> u8       { self.0.lrc(slave) }
}

// Modbus function #2 response.
pub struct ReadDiscreteInputsResp {
    resp: ReadAddrCountGenericResp,
    req: ReadDiscreteInputs,
}

impl ResponsePDU for ReadDiscreteInputsResp {
    type Req = ReadDiscreteInputs;

    fn from_ascii(req: &Self::Req, packet: &String) -> Result<Self, ModbusError> {
        Ok(Self { req: *req, resp: ReadAddrCountGenericResp::from_ascii(packet, req.0.cnt)? })
    }
}

impl ReadDiscreteInputsResp {
    fn discrete_input(&self, mut idx: u16) -> Result<bool, ModbusError> {
        self.resp.object(idx, self.req.0.addr)
    }
}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #3: Read Multiple Holding Registers.
#[derive(Clone, Copy)]
pub struct ReadHoldingRegisters(ReadAddrCountGeneric<U3>);

impl ReadHoldingRegisters {
    pub fn new(addr: u16, cnt: u16) -> Self {
        Self( ReadAddrCountGeneric::<U3>::new(addr, cnt) )
    }
}

impl RequestPDU for ReadHoldingRegisters {
    type Resp = ReadHoldingRegistersResp;

    fn fn_id()                  -> u8       { ReadAddrCountGeneric::<U3>::fn_id() }
    fn as_ascii(&self)          -> String   { self.0.as_ascii() }
    fn lrc(&self, slave: u8)    -> u8       { self.0.lrc(slave) }
}

// Modbus function #3 response.
pub struct ReadHoldingRegistersResp {
    resp: ReadAddrCountGeneric16BitResp,
    req: ReadHoldingRegisters,
}

impl ResponsePDU for ReadHoldingRegistersResp {
    type Req = ReadHoldingRegisters;

    fn from_ascii(req: &Self::Req, packet: &String) -> Result<Self, ModbusError> {
        Ok(Self { resp: ReadAddrCountGeneric16BitResp::from_ascii(req.0.cnt, packet)?, req: *req })
    }
}

impl ReadHoldingRegistersResp {
    fn register(&self, idx: u8) -> Result<u16, ModbusError> {
        self.resp.object(idx)
    }
}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #4: Read Multiple Input Registers.
#[derive(Clone, Copy)]
pub struct ReadInputRegisters(ReadAddrCountGeneric<U4>);

impl RequestPDU for ReadInputRegisters {
    type Resp = ReadInputRegistersResp;

    fn fn_id()                  -> u8       { ReadAddrCountGeneric::<U4>::fn_id() }
    fn as_ascii(&self)          -> String   { self.0.as_ascii() }
    fn lrc(&self, slave: u8)    -> u8       { self.0.lrc(slave) }
}

// Modbus function #4 response.
pub struct ReadInputRegistersResp {
    resp: ReadAddrCountGeneric16BitResp,
    req: ReadInputRegisters,
}

impl ResponsePDU for ReadInputRegistersResp {
    type Req = ReadInputRegisters;

    fn from_ascii(req: &Self::Req, packet: &String) -> Result<Self, ModbusError> {
        Ok(Self { resp: ReadAddrCountGeneric16BitResp::from_ascii(req.0.cnt, packet)?, req: *req })
    }
}

impl ReadInputRegistersResp {
    fn register(&self, idx: u8) -> Result<u16, ModbusError> {
        self.resp.object(idx)
    }
}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #5: Write Single Coil.
#[derive(Copy, Clone)]
pub struct WriteSingleCoil {
    addr: u16,
    coil: bool,
}

impl WriteSingleCoil {
    const FN_ID: u8 = 0x05;
}

impl RequestPDU for WriteSingleCoil {
    type Resp = WriteSingleCoilResp;

    fn fn_id() -> u8 { Self::FN_ID }

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, if self.coil { 0xff00 } else { 0x0000 }).as_ascii(Self::FN_ID)
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, if self.coil { 0xff00 } else { 0x0000 }).lrc(slave, Self::FN_ID)
    }
}

// Modbus function #5 response.
pub struct WriteSingleCoilResp;

impl ResponsePDU for WriteSingleCoilResp {
    type Req = WriteSingleCoil;

    fn from_ascii(req: &Self::Req, packet: &String) -> Result<Self, ModbusError> {
        let resp_addr = u16::from_str_radix(&packet[0..4], 16).map_err(|e| ModbusError::InvalidData)?;
        let resp_val = u16::from_str_radix(&packet[4..8], 16).map_err(|e| ModbusError::InvalidData)?;

        if resp_addr != req.addr || resp_val != ( if req.coil { 0xff00 } else { 0x0000 } ) {
            Err(ModbusError::ReqRespMismatch)?
        }

        Ok(Self{})
    }
}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #6: Write Single Holding Register.
#[derive(Copy, Clone)]
pub struct WriteSingleHoldingRegister {
    addr: u16,
    val:  u16,
}

impl WriteSingleHoldingRegister {
    const FN_ID: u8 = 0x06;
}

impl RequestPDU for WriteSingleHoldingRegister {
    type Resp = WriteSingleHoldingRegisterResp;

    fn fn_id() -> u8 { Self::FN_ID }

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, self.val).as_ascii(Self::FN_ID)
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, self.val).lrc(slave, Self::FN_ID)
    }
}

// Modbus function #6 response.
pub struct WriteSingleHoldingRegisterResp;

impl ResponsePDU for WriteSingleHoldingRegisterResp {
    type Req = WriteSingleHoldingRegister;

    fn from_ascii(req: &Self::Req, packet: &String) -> Result<Self, ModbusError> {
        let resp_addr = u16::from_str_radix(&packet[0..4], 16).map_err(|e| ModbusError::InvalidData)?;
        let resp_val = u16::from_str_radix(&packet[4..8], 16).map_err(|e| ModbusError::InvalidData)?;

        if resp_addr != req.addr || resp_val != req.val {
            Err(ModbusError::ReqRespMismatch)?
        }

        Ok(Self{})
    }
}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #15: Write Multiple Coils
// For simplicity, maximum count of coils to be written is intentionally set to be
// 64 (8 bits x 8 groups == 64 bits)
#[derive(Copy, Clone)]
pub struct WriteMultipleCoils {
    addr: u16,
    cnt:  u16,
    sz:   u8,
    val:  u64,
}

impl WriteMultipleCoils {
    const FN_ID: u8 = 0x0f;
}

impl RequestPDU for WriteMultipleCoils {
    type Resp = WriteMultipleCoilsResp;

    fn fn_id() -> u8 { Self::FN_ID }

    fn as_ascii(&self) -> String {
        format!("{:02X}{:04X}{:04X}{:02X}{:16X}", Self::FN_ID, self.addr, self.cnt, self.sz, self.val)
    }

    fn lrc(&self, slave: u8) -> u8 {
        generic_lrc(
            &[
                slave,
                Self::FN_ID,
                exract_byte(self.addr, 8),  exract_byte(self.addr, 0),
                exract_byte(self.cnt,  8),  exract_byte(self.cnt,  0),
                self.sz,
                exract_byte(self.val,  56),
                exract_byte(self.val,  48),
                exract_byte(self.val,  40),
                exract_byte(self.val,  32),
                exract_byte(self.val,  24),
                exract_byte(self.val,  16),
                exract_byte(self.val,  8),
                exract_byte(self.val,  0),
            ]
        )
    }
}

// Modbus function #15 response.
pub struct WriteMultipleCoilsResp;

impl ResponsePDU for WriteMultipleCoilsResp {
    type Req = WriteMultipleCoils;

    fn from_ascii(req: &Self::Req, packet: &String) -> Result<Self, ModbusError> {
        let resp_addr = u16::from_str_radix(&packet[0..4], 16).map_err(|e| ModbusError::InvalidData)?;
        let resp_val = u16::from_str_radix(&packet[4..8], 16).map_err(|e| ModbusError::InvalidData)?;

        if resp_addr != req.addr || resp_val != req.cnt {
            Err(ModbusError::ReqRespMismatch)?
        }

        Ok(Self{})
    }
}

/*------------------------------------------------------------------------------------------------*/

// Makes the request frame from given PDU for given slave device
pub fn make_request_frame(slave_address: u8, pdu: &impl RequestPDU) -> String {
    format!(":{:02X}{}{:02X}\r\n", slave_address, pdu.as_ascii(), pdu.lrc(slave_address))
}

pub fn extract_response<T, R>(slave_address: u8, req: &T, packet: &String) -> Result<R, ModbusError>
where
    T: RequestPDU<Resp = R>,
    R: ResponsePDU<Req = T>,
{
    if !packet.starts_with(":") || !packet.ends_with("\r\n") {
        Err(ModbusError::InvalidFrame)?
    }

    let len = packet.len();

    let addr
        = u8::from_str_radix(packet.get(1..3).ok_or(ModbusError::InvalidAddr)?, 16)
            .map_err(|e| ModbusError::InvalidAddr)?;
    let fn_id
        = u8::from_str_radix(packet.get(3..5).ok_or(ModbusError::InvalidFnId)?, 16)
            .map_err(|e| ModbusError::InvalidFnId)?;
    let pdu = packet.get(5..len-4).ok_or(ModbusError::InvalidData)?;
    let crc
        = u8::from_str_radix(packet.get(len - 4..len - 2).ok_or(ModbusError::InvalidCRC)?, 16)
            .map_err(|e| ModbusError::InvalidCRC)?;

    // Data protected by CRC
    let data_under_crc = packet
        .get(1..len - 4)
        .ok_or(ModbusError::InvalidData)?
        .as_bytes()
        .chunks(2);

    println!("extracted addr: {:02X} fn: {:02X} pdu: {} crc: {:02X}",
            addr, fn_id, pdu, crc);

    let mut complete_data = Vec::new();

    for chunk in data_under_crc {
        let byte = u8::from_str_radix(str::from_utf8(chunk).unwrap(), 16).map_err(|e| ModbusError::InvalidData)?;
        complete_data.push(byte);
    }

    let calc_crc = generic_lrc(complete_data.as_slice());

    if calc_crc != crc {
        Err(ModbusError::InvalidCRC)?
    }

    if fn_id != T::fn_id() {
        Err(ModbusError::InvalidFnId)?
    }

    R::from_ascii(req, &pdu.to_string())
}

#[cfg(test)]
#[test]
fn test_extract_response() {
    use crate::modbus::ResponsePDU;
    use crate::modbus::ReadCoils;
    use crate::modbus::ReadCoilsResp;

    // 11 coils, starting from 42:
    // on, off, on, off, off, on, on, on, off, on, on
    // in hex: (E5, 06)

    let coil_addr = 42;
    let coil_cnt = 11;
    let slave_id = 8;

    let req = ReadCoils::new(coil_addr, coil_cnt);
    // TODO: fill correct values for slave ID etc.
    let rc = extract_response(slave_id, &req, &":080102E5060A\r\n".to_string());
    if let Err(e) = rc {
        println!("failed to extract: {}, aborting test", e);
        assert!(false);
    } else {
        let resp = rc.unwrap();

        println!("{:?}", resp);

        assert_eq!(resp.resp.cnt,    11);
        assert_eq!(resp.resp.data[0], 0xe5);
        assert_eq!(resp.resp.data[1], 0x06);

        assert_eq!(resp.coil(42).unwrap(), true);
        assert_eq!(resp.coil(43).unwrap(), false);
        assert_eq!(resp.coil(44).unwrap(), true);
        assert_eq!(resp.coil(45).unwrap(), false);
        assert_eq!(resp.coil(46).unwrap(), false);
        assert_eq!(resp.coil(47).unwrap(), true);
        assert_eq!(resp.coil(48).unwrap(), true);
        assert_eq!(resp.coil(49).unwrap(), true);
        assert_eq!(resp.coil(50).unwrap(), false);
        assert_eq!(resp.coil(51).unwrap(), true);
        assert_eq!(resp.coil(52).unwrap(), true);

        assert_eq!(resp.coil(53).is_err(), true);
        assert_eq!(resp.coil(41).is_err(), true);
        assert_eq!(resp.coil(0).is_err(),  true);

    }
}

fn do_master_request<T, R>(addr: u8, req: &T, mut fl: &File, timeout: u32) -> Result<R, ModbusError>
    where
        T: RequestPDU<Resp = R>,
        R: ResponsePDU<Req = T>,
{
    let mut data = Vec::new();
    fl.write_all(make_request_frame(addr, req).as_bytes()).map_err(|e| ModbusError::IoErr)?;
    let mut reader = BufReader::new(fl);
    reader.read_until(b'\n', &mut data).map_err(|e| ModbusError::IoErr)?;
    extract_response(addr, req, &String::from_utf8(data).map_err(|e| ModbusError::IoErr)?)
}
