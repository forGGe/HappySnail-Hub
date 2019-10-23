extern crate clap;
extern crate paho_mqtt as mqtt;

use std::process;
use std::vec;
use std::time::Duration;
use std::num::Wrapping;
use clap::{Arg, App};

trait RequestPDU {
    const FN_ID: u8;
    fn as_ascii(&self) -> String;
    fn lrc(&self, slave: u8) -> u8;
}

fn generic_lrc(data: &[u8]) -> u8 {
    // ISO 1155
    // u16 helps to avoid using std::Wrapping

    let r = data.iter().fold(0u16, |acc, x| (acc + *x as u16) & 0xff);
    (((r ^ 0xff) + 1) & 0xff) as u8
}

//-----

struct RequestPair(u16, u16);

impl RequestPair {
    fn lrc(&self, slave: u8, fn_id: u8) -> u8 {
        generic_lrc(
            &[
                slave,
                fn_id,
                (self.0 >> 8) as u8, self.0 as u8,
                (self.1 >> 8) as u8, self.1 as u8
            ]
        )
    }

    fn as_ascii(&self, fn_id: u8) -> String {
        format!("{:02X}{:04X}{:04X}", fn_id, self.0, self.1)
    }
}

//-----

struct ReadCoils {
    addr: u16,
    cnt:  u16,
}

impl RequestPDU for ReadCoils {
    const FN_ID: u8 = 0x01;

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, self.cnt).as_ascii(Self::FN_ID)
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, self.cnt).lrc(slave, Self::FN_ID)
    }
}

//-----

struct ReadDiscreteInputs {
    addr: u16,
    cnt:  u16,
}

impl RequestPDU for ReadDiscreteInputs {
    const FN_ID: u8 = 0x02;

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, self.cnt).as_ascii(Self::FN_ID)
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, self.cnt).lrc(slave, Self::FN_ID)
    }
}

//-----

struct ReadHoldingRegisters {
    addr: u16,
    cnt:  u16,
}

impl RequestPDU for ReadHoldingRegisters {
    const FN_ID: u8 = 0x03;

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, self.cnt).as_ascii(Self::FN_ID)
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, self.cnt).lrc(slave, Self::FN_ID)
    }
}

//-----

struct WriteSingleCoil {
    addr: u16,
    coil: bool,
}

impl RequestPDU for WriteSingleCoil {
    const FN_ID: u8 = 0x05;

    fn as_ascii(&self) -> String {
        RequestPair(self.addr, if self.coil { 0xff00 } else { 0x0000 }).as_ascii(Self::FN_ID)
    }

    fn lrc(&self, slave: u8) -> u8 {
        RequestPair(self.addr, if self.coil { 0xff00 } else { 0x0000 }).lrc(slave, Self::FN_ID)
    }
}

//-----

fn make_request_frame(slave_address: u8, pdu: impl RequestPDU) -> String {
    format!(":{:02X}{}{:02X}\r\n", slave_address, pdu.as_ascii(), pdu.lrc(slave_address))
}

//-----

fn main() {
    println!("Hello, world!");

    println!("{}", make_request_frame(247, ReadHoldingRegisters { addr: 5001, cnt: 10 }));

    let matches = App::new("HappySnail Hub")
        .version("0.1.0")
        .author("Max Payne <forgge@gmail.com>")
        .about("HappySnail Data Collector and Controller")
        .arg(Arg::with_name("tty")
                 .short("t")
                 .long("tty")
                 .takes_value(true)
                 .required(true)
                 .help("ModBus/RS485 TTY to connect to"))
        .arg(Arg::with_name("tb_uri")
                 .short("h")
                 .long("tb_uri")
                 .takes_value(true)
                 .default_value("demo.thingsboard.io")
                 .help("ThingsBoard MQTT URI to connect to"))
        .arg(Arg::with_name("token")
                 .short("k")
                 .long("token")
                 .takes_value(true)
                 .required(true)
                 .help("ThingsBoard device token"))
        .arg(Arg::with_name("delay")
                 .short("d")
                 .long("delay")
                 .takes_value(true)
                 .required(true)
                 .help("Delay between data queries, in seconds"))
        .get_matches();

    let tty = matches.value_of("tty").unwrap().parse::<String>().expect("invalid type for tty parameter");
    let token = matches.value_of("token").unwrap().parse::<String>().expect("invalid type for token parameter");
    let delay = matches.value_of("delay").unwrap().parse::<u32>().expect("invalid type for delay parameter");
    let tb_uri = matches.value_of("tb_uri").unwrap().parse::<String>().expect("invalid type for tb_uri parameter");

    println!("Using TTY: {}, delay: {}, TB URI: {}", tty, delay, tb_uri);

    // Switch off persistance (for now)
    let opts = mqtt::CreateOptionsBuilder::new()
                   .server_uri(tb_uri)
                   .client_id("hs-hub")
                   .persistence(mqtt::PersistenceType::None)
                   .finalize();

    let cli = mqtt::Client::new(opts).unwrap_or_else(|err| {
        println!("Error creating the client: {:?}", err);
        process::exit(1);
    });

    let conn_opts = mqtt::ConnectOptionsBuilder::new()
        .keep_alive_interval(Duration::from_secs(20))
        .user_name(token)
        .clean_session(true)
        .finalize();

    // Connect to TB
    if let Err(e) = cli.connect(conn_opts) {
        println!("Unable to connect:\n\t{:?}", e);
        process::exit(1);
    }

    // Create a message and publish it
    let msg = mqtt::Message::new("test", "Hello world!", 0);
    if let Err(e) =  cli.publish(msg) {
        println!("Unable to publish:\n\t{:?}", e);
        process::exit(1);
    }

    // Disconnect from the broker
    cli.disconnect(mqtt::DisconnectOptions::new()).unwrap();
}
