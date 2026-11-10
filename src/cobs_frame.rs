//! COBS (Consistent Overhead Byte Stuffing) framing for the serial link.
//!
//! A message is COBS-encoded and terminated with a single `0x00` delimiter
//! byte, matching the Linux `lm4f120-bridge` on the other end of the wire.
//! The encoder/decoder come from the `cobs` crate (no_std).

use cobs::{decode, encode};

/// Maximum decoded payload length carried in a single frame.
pub const MAX_PAYLOAD: usize = 256;
/// Worst-case COBS-encoded length of a [`MAX_PAYLOAD`]-byte payload, excluding
/// the trailing delimiter (`MAX_PAYLOAD + ceil(MAX_PAYLOAD / 254)`).
pub const MAX_ENCODED: usize = MAX_PAYLOAD + (MAX_PAYLOAD + 253) / 254;
/// A complete frame: the encoded payload plus one `0x00` delimiter byte.
pub const MAX_FRAME: usize = MAX_ENCODED + 1;

/// COBS-encode `payload` into `out` and append the `0x00` delimiter.
///
/// Returns the number of bytes written to `out`. `out` must be at least
/// [`MAX_FRAME`] bytes long and `payload` at most [`MAX_PAYLOAD`] bytes.
pub fn encode_frame(payload: &[u8], out: &mut [u8]) -> usize {
    if payload.is_empty() {
        return 0;
    }
    let n = encode(payload, out);
    out[n] = 0x00;
    n + 1
}

/// Decode a COBS-encoded `encoded` frame (without the trailing `0x00`) into
/// `out`. Returns the decoded length, or `None` on a malformed frame.
pub fn decode_frame(encoded: &[u8], out: &mut [u8]) -> Option<usize> {
    decode(encoded, out).ok().map(|r| r.frame_size())
}
