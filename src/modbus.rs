extern crate typenum;
extern crate num;

use typenum::*;
use std::ops::Shr;
use num::cast::AsPrimitive;
use std::vec::Vec;
use std::io;

// Trait for serialize Modbus PDUs
pub trait RequestPDU {
    type Resp;
    fn as_ascii(&self) -> String;
    fn lrc(&self, slave: u8) -> u8;
}

pub trait ResponsePDU: Sized {
    type Req;
    fn from_ascii(req: &Self::Req) -> Result<Self, io::Error>;
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
#[derive(Clone, Copy)]
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

/*------------------------------------------------------------------------------------------------*/

// Modbus function #1: Read Coils.
#[derive(Clone, Copy)]
pub struct ReadCoils(ReadAddrCountGeneric<U1>);

impl RequestPDU for ReadCoils {
    type Resp = ReadCoilsResp;

    fn as_ascii(&self)          -> String   { self.0.as_ascii() }
    fn lrc(&self, slave: u8)    -> u8       { self.0.lrc(slave) }
}

// Modbus function #1 response.
pub struct ReadCoilsResp {
    cnt: u16,
    val: Vec<u8>,
    req: ReadCoils,
}

impl ResponsePDU for ReadCoilsResp {
    type Req = ReadCoils;

    fn from_ascii(req: &Self::Req) -> Result<Self, io::Error> {
        Ok(Self { cnt: 0, val: Vec::new(), req: *req })
    }
}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #2: Read Discrete Inputs.
#[derive(Clone, Copy)]
pub struct ReadDiscreteInputs(ReadAddrCountGeneric<U2>);

impl RequestPDU for ReadDiscreteInputs {
    type Resp = ReadDiscreteInputsResp;

    fn as_ascii(&self)          -> String   { self.0.as_ascii() }
    fn lrc(&self, slave: u8)    -> u8       { self.0.lrc(slave) }
}

// Modbus function #2 response.
pub struct ReadDiscreteInputsResp {
    cnt: u16,
    val: Vec<u8>,
    req: ReadDiscreteInputs,
}

impl ResponsePDU for ReadDiscreteInputsResp {
    type Req = ReadDiscreteInputs;

    fn from_ascii(req: &Self::Req) -> Result<Self, io::Error> {
        Ok(Self { cnt: 0, val: Vec::new(), req: *req })
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

    fn as_ascii(&self)          -> String   { self.0.as_ascii() }
    fn lrc(&self, slave: u8)    -> u8       { self.0.lrc(slave) }
}

// Modbus function #3 response.
pub struct ReadHoldingRegistersResp {
    cnt: u16,
    val: Vec<u8>,
    req: ReadHoldingRegisters,
}

impl ResponsePDU for ReadHoldingRegistersResp {
    type Req = ReadHoldingRegisters;

    fn from_ascii(req: &Self::Req) -> Result<Self, io::Error> {
        Ok(Self { cnt: 0, val: Vec::new(), req: *req })
    }
}

/*------------------------------------------------------------------------------------------------*/

// Modbus function #4: Read Input Registers.
#[derive(Clone, Copy)]
pub struct ReadInputRegisters(ReadAddrCountGeneric<U4>);

impl RequestPDU for ReadInputRegisters {
    type Resp = ReadInputRegistersResp;

    fn as_ascii(&self)          -> String   { self.0.as_ascii() }
    fn lrc(&self, slave: u8)    -> u8       { self.0.lrc(slave) }
}

// Modbus function #4 response.
pub struct ReadInputRegistersResp {
    cnt: u16,
    val: Vec<u8>,
    req: ReadInputRegisters,
}

impl ResponsePDU for ReadInputRegistersResp {
    type Req = ReadInputRegisters;

    fn from_ascii(req: &Self::Req) -> Result<Self, io::Error> {
        Ok(Self { cnt: 0, val: Vec::new(), req: *req })
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

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, if self.coil { 0xff00 } else { 0x0000 }).as_ascii(Self::FN_ID)
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, if self.coil { 0xff00 } else { 0x0000 }).lrc(slave, Self::FN_ID)
    }
}

// Modbus function #2 response.
pub struct WriteSingleCoilResp {
    cnt: u16,
    val: Vec<u8>,
    req: WriteSingleCoil,
}

impl ResponsePDU for WriteSingleCoilResp {
    type Req = WriteSingleCoil;

    fn from_ascii(req: &Self::Req) -> Result<Self, io::Error> {
        Ok(Self { cnt: 0, val: Vec::new(), req: *req })
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

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, self.val).as_ascii(Self::FN_ID)
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, self.val).lrc(slave, Self::FN_ID)
    }
}

// Modbus function #2 response.
pub struct WriteSingleHoldingRegisterResp {
    cnt: u16,
    val: Vec<u8>,
    req: WriteSingleHoldingRegister,
}

impl ResponsePDU for WriteSingleHoldingRegisterResp {
    type Req = WriteSingleHoldingRegister;

    fn from_ascii(req: &Self::Req) -> Result<Self, io::Error> {
        Ok(Self { cnt: 0, val: Vec::new(), req: *req })
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

// Modbus function #2 response.
pub struct WriteMultipleCoilsResp {
    cnt: u16,
    val: Vec<u8>,
    req: WriteMultipleCoils,
}

impl ResponsePDU for WriteMultipleCoilsResp {
    type Req = WriteMultipleCoils;

    fn from_ascii(req: &Self::Req) -> Result<Self, io::Error> {
        Ok(Self { cnt: 0, val: Vec::new(), req: *req })
    }
}

/*------------------------------------------------------------------------------------------------*/

// Makes the request frame from given PDU for given slave device
pub fn make_request_frame(slave_address: u8, pdu: impl RequestPDU) -> String {
    format!(":{:02X}{}{:02X}\r\n", slave_address, pdu.as_ascii(), pdu.lrc(slave_address))
}

