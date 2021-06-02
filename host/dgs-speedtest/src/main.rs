use std::io::{self, Write};
use std::time::{Duration, Instant};


fn main() {
    let mut dgs_port = None;
    let ports = serialport::available_ports().unwrap();
    for port in ports {
        if let serialport::SerialPortType::UsbPort(usb_info) = port.port_type {
            if let Some(num) = usb_info.serial_number {
                if num.contains("diegesis") {
                    dgs_port = Some(port.port_name);
                }
            }
        } else {
            continue;
        }
    }

    let dgs_port = dgs_port.unwrap();

    println!("Found diegesis on port: {}", dgs_port);

    let port = serialport::new(&dgs_port, 115_200)
        .timeout(Duration::from_millis(10))
        .open();

    let mut start = Instant::now();
    let mut bytes_rxd = 0;
    let mut moving_avg = -1.0f64;

    match port {
        Ok(mut port) => {
            let mut serial_buf: Vec<u8> = vec![0; 1000];
            println!("Receiving data on {} at {} baud:", &dgs_port, 115200);
            loop {
                if start.elapsed() >= Duration::from_secs(1) {
                    if moving_avg <= 0.0 {
                        moving_avg = (bytes_rxd as f64);
                    } else {
                        moving_avg *= 0.9;
                        moving_avg += (bytes_rxd as f64) * 0.1;
                    }

                    println!("{} bytes/sec\t{:0.02} bytes/sec (avg)", bytes_rxd, moving_avg);
                    bytes_rxd = 0;
                    start = Instant::now();
                }
                match port.read(serial_buf.as_mut_slice()) {
                    Ok(t) => {
                        // TODO: Verify contents
                        bytes_rxd += t;
                    },
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
                    Err(e) => {
                        eprintln!("{:?}", e);
                        ::std::process::exit(1);
                    },
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", &dgs_port, e);
            ::std::process::exit(1);
        }
    }
}
