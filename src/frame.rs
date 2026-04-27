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