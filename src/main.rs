extern crate clap;
extern crate typenum;
extern crate paho_mqtt as mqtt;

mod modbus;

use clap::{Arg, App};
use std::time::Duration;
use std::process;

//-----

fn main() {
    println!("Hello, world!");

    println!("{}", modbus::make_request_frame(247, modbus::ReadHoldingRegisters::new(5001, 10)));

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
