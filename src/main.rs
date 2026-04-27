mod frame;

use std::{error::Error, time::Duration, thread, fmt::Write};

use serial2::SerialPort;
use frame::{try_parse_frame, decode_log};

slint::include_modules!();

use crc::{Crc, CRC_8_MAXIM_DOW, CRC_32_ISO_HDLC};

const CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_MAXIM_DOW);
const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

fn build_frame(seq: u8, msg_type: u8, data: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(7 + data.len() + 4);
    frame.push(0xBC);
    frame.push(seq);
    frame.push(0x00);                                  // flags
    frame.push(msg_type);
    frame.extend_from_slice(&(data.len() as u16).to_be_bytes());
    let hdr_crc = CRC8.checksum(&frame[..6]);
    frame.push(hdr_crc);
    frame.extend_from_slice(data);
    if msg_type & 0x80 != 0 {
        // High-bit msg types get a "inverted little-endian CRC-32" trailer.
        let crc = !CRC32.checksum(data);
        frame.extend_from_slice(&crc.to_le_bytes());
    }
    frame
}

fn main() -> Result<(), Box<dyn Error>> {

    #[cfg(feature = "framebuffer")] {
        use slint_backend_linuxfb::LinuxFbPlatformBuilder;

        let platform = LinuxFbPlatformBuilder::new()
            .with_framebuffer("/dev/fb0")
            .with_input_autodiscovery(true)
            .build()
            .unwrap();

        slint::platform::set_platform(Box::new(platform)).unwrap();
    }

    // cVEND (NFC reader) located at /dev/ttymxc3
    // https://wiki.pm3.dev/cvend
    let port_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/ttymxc3".to_string());

    let baud = 115_200;
    
    let main_window = AppWindow::new()?;
    let weak = main_window.as_weak();

    // reader thread
    thread::spawn(move || {
        let mut port: SerialPort = match SerialPort::open(&port_path, baud) {
            Ok(p) => p,
            Err(e) => {
                let msg = format!("failed to open {}: {}", port_path, e);
                set_status_from_thread(&weak, msg);
                return;
            },
        };

        if let Err(e) = port.set_read_timeout(Duration::from_millis(500)) {
            let msg = format!("failed to set read timeout: {}", e);
            set_status_from_thread(&weak, msg);
            return;
        }

        set_status_from_thread(&weak, format!("listening on {}", port_path));

        // enable application level frame parsing 
        let enable_iso = build_frame(0x01, 0xE4, &[0x00, 0x06, 0x01, 0x01]);
        if let Err(e) = port.write_all(&enable_iso) {
            set_status_from_thread(&weak, format!("failed to enable ISO: {}", e));
            return;
        }

        let enable_ultralight = build_frame(0x02, 0xE4, &[0x00, 0x0A, 0x01, 0x01]);
        if let Err(e) = port.write_all(&enable_ultralight) {
            set_status_from_thread(&weak, format!("failed to enable Ultralight: {}", e));
            return;
        }

        // read the stuff
        let mut buf = [0u8; 256];
        let mut acc: Vec<u8> = Vec::new();
        let mut display = String::new();
        let mut log_display = String::new();

        // CardRelease frame — precomputed.
        //   start  seq  flags  type   len_hi  len_lo  hdr_crc
        //   0xBC   0x01 0x00   0x32   0x00    0x00    0x91
        // const CARD_RELEASE: &[u8] = &[0xBC, 0x01, 0x00, 0x32, 0x00, 0x00, 0x91];
        // let mut last_release = Instant::now();
        // let release_every = Duration::from_millis(500);

        let mut read_count: i32 = 0;
        let mut heartbeat_count: i32 = 0;

        loop {
            match port.read(&mut buf) {
                Ok(n) => {
                    acc.extend_from_slice(&buf[..n]);

                    while let Some(frame) = try_parse_frame(&mut acc) {
                        if frame.msg_type == 0x07 {
                            heartbeat_count += 1;
                        } else {
                            let _ = writeln!(display, "[{:#04X}] seq={}, data={:02X?}", frame.msg_type, frame.seq, frame.data);
                            if frame.msg_type == 0xED {
                                if let Some(log_msg) = decode_log(&frame.data) {
                                    let _ = writeln!(log_display, "{}", log_msg);
                                }
                            }
                            read_count += 1;
                        }
                    }

                    const MAX_DISPLAY: usize = 4096;
                    if display.len() > MAX_DISPLAY {
                        let drain_to = display.len() - MAX_DISPLAY;
                        display.drain(..drain_to);
                    }
                    if log_display.len() > MAX_DISPLAY {
                        let drain_to = log_display.len() - MAX_DISPLAY;
                        log_display.drain(..drain_to);
                    }

                    let snapshot = display.clone();
                    let log_snapshot = log_display.clone();
                    let weak_inner = weak.clone();
                    let rc = read_count;
                    let hc = heartbeat_count;
                    let _ = slint::invoke_from_event_loop(move || {
                        if let Some(main_window) = weak_inner.upgrade() {
                            main_window.set_raw_data(slint::SharedString::from(snapshot));
                            main_window.set_log_data(slint::SharedString::from(log_snapshot));
                            main_window.set_read_count(rc);
                            main_window.set_heartbeat_count(hc);
                        }
                    });

                }
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    // ignore timeout errors (no data received within the timeout)
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // ignore would-block errors (no data available to read)
                }
                Err(e) => {
                    let msg = format!("error reading from {}: {}", port_path, e);
                    set_status_from_thread(&weak, msg);
                    return;
                }
            }

            // if last_release.elapsed() >= release_every {
            //     if let Err(e) = port.write_all(CARD_RELEASE) {
            //         let msg = format!("write error: {e}");
            //         set_status_from_thread(&weak, msg);
            //     }
            //     last_release = Instant::now();
            // }
        }
    });

    

    main_window.run()?;

    Ok(())
}

fn set_status_from_thread(weak: &slint::Weak<AppWindow>, status: String) {
    let weak = weak.clone();
    let _ = slint::invoke_from_event_loop(move || {
        if let Some(main_window) = weak.upgrade() {
            main_window.set_status(status.into());
        }
    });
}