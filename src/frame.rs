use std::fmt::Write;
use crc::{Crc, CRC_8_MAXIM_DOW, CRC_32_ISO_HDLC};

const CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_MAXIM_DOW);
const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

#[derive(Debug)]
pub struct Frame {
  pub seq: u8,
  pub flags: u8,
  pub msg_type: u8,
  pub data: Vec<u8>,
}

pub fn frame_type_name(msg_type: u8) -> &'static str {
    match msg_type {
        0x02 => "Version",
        0x03 => "VersionReply",
        0x04 => "Status",
        0x05 => "StatusReply",
        0x07 => "Heartbeat",
        0x0F => "Startup",
        0x10 => "Reset",
        0x11 => "ResetReply",
        0x20 => "LEDs",
        0x22 => "Buzzer",
        0x32 => "CardRelease",
        0x46 => "AbortCardHandling",
        0xB1 => "ISORead",
        0xB3 => "ISOCardReleased",
        0xB4 => "APDUProx",
        0xB5 => "APDUProxReply",
        0xB9 => "DESFireRead",
        0xBB => "DESFireCardRemoved",
        0xBC => "DESFireCommand",
        0xBD => "DESFireCommandReply",
        0xBE => "UnhandledCard",
        0xD0 => "EMV",
        0xD1 => "EMVStatus",
        0xE4 => "ProxCardFunction",
        0xE5 => "ProxCardFunctionReply",
        0xED => "Log",
        _ => "Unknown",
    }
}

pub fn decode_version_reply(data: &[u8]) -> Option<String> {
    let labels = ["Device ID", "Firmware", "Working Dir"];
    let mut result = String::new();
    let mut i = 0;
    for label in &labels {
        if i >= data.len() { break; }
        let len = data[i] as usize;
        i += 1;
        if i + len > data.len() { break; }
        let s = String::from_utf8_lossy(&data[i..i + len]);
        let _ = writeln!(result, "{}: {}", label, s);
        i += len;
    }
    if result.is_empty() { None } else { Some(result) }
}

pub fn decode_log(data: &[u8]) -> Option<String> {
    if data.is_empty() { return None; }
    let level = match data[0] {
        1 => "INFO",
        2 => "WARN",
        3 => "ERROR",
        _ => "?",
    };
    let msg_bytes = &data[1..];
    let end = msg_bytes.iter().position(|&b| b == 0).unwrap_or(msg_bytes.len());
    let msg = String::from_utf8_lossy(&msg_bytes[..end]);
    Some(format!("[{}] {}", level, msg))
}

pub fn try_parse_frame(buf: &mut Vec<u8>) -> Option<Frame> {
  // look for start byte (0xBC)
  if let Some(start) = buf.iter().position(|&b| b == 0xBC) {
    buf.drain(..start); // discard bytes before start
  } else {
    buf.clear();
    return None;
  }

  if buf.len() < 7 {
    return None; // not enough data for header
  }

  let msg_type = buf[3];
  let payload_len = u16::from_be_bytes([buf[4], buf[5]]) as usize;
  let trailer = if msg_type & 0x80 != 0 { 4 } else { 0 };
  let total_len = 7 + payload_len + trailer;

  if buf.len() < total_len {
    return None; // not enough data for full frame
  }

  //validate header CRC
  let expected_hrdr_crc = CRC8.checksum(&buf[..6]);
  if buf[6] != expected_hrdr_crc {
    buf.drain(..7); // discard invalid header (resync)
    return None;
  }

  let data = buf[7..7+payload_len].to_vec();

  if trailer == 4 {
    let expected_crc = !CRC32.checksum(&data);
    let got_crc = u32::from_le_bytes(buf[7+payload_len..total_len].try_into().unwrap());
    if expected_crc != got_crc {
      buf.drain(..total_len); // discard invalid frame
      return None;
    }
  }

  let seq = buf[1];
  let flags = buf[2];

  buf.drain(..total_len); // consume the frame
  Some(Frame {
    seq,
    flags,
    msg_type,
    data,
  })
}