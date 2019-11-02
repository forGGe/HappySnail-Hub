#[macro_use]

extern crate clap;
extern crate typenum;
extern crate paho_mqtt as mqtt;

mod modbus;

use clap::{Arg, App};
use std::time::Duration;
use std::process;
use std::{thread, time};

use serde_json::json;

//-----

fn main() {
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

    let mut port = serialport::open_with_settings(
            tty.as_str(),
            &serialport::SerialPortSettings {
                baud_rate: 9600,
                data_bits: serialport::DataBits::Eight,
                flow_control: serialport::FlowControl::None,
                parity: serialport::Parity::None,
                stop_bits: serialport::StopBits::One,
                timeout: Duration::from_millis(2000),
            }
        ).expect("failed to open TTY port");

    loop {
        let req = modbus::ReadHoldingRegisters::new(1000, 6);
        let resp = modbus::do_master_req(42, &req, &mut port);

        if let Ok(r) = resp {
            // Unwrapping everything: do_master_req() must guarantee data presence if case of success.
            // If it not there, then something bad happened

            let temp = r.register(0).unwrap() as f32 + r.register(1).unwrap() as f32  / 100.0;
            let mcu_temp = r.register(2).unwrap() as f32 + r.register(3).unwrap() as f32  / 100.0;
            let humid = r.register(4).unwrap() as f32 + r.register(5).unwrap() as f32  / 100.0;

            println!("temperature: {}, MCU temperature: {}, humidity: {}", temp, mcu_temp, humid);

            // Create a message and publish it
            let msg_json = json!({
                "temp": temp,
                "mcu_temp": mcu_temp,
                "rh": humid,
            });

            let msg = mqtt::Message::new("v1/devices/me/telemetry", msg_json.to_string(), 0);
            if let Err(e) =  cli.publish(msg) {
                println!("Unable to publish:\n\t{:?}", e);
                break;
            }
        } else {
            println!("Error communicating with sensor: {}", resp.unwrap_err());
        }

        thread::sleep(time::Duration::from_secs(delay.into()));
    }

    // Disconnect from the broker
    cli.disconnect(mqtt::DisconnectOptions::new()).unwrap();
}
