use zenoh_ros2::wire::{self, ZenohWire};
use zenoh_ros2::types::geometry::{Twist, Vector3};

#[test]
fn test_cdr_roundtrip_twist() {
    let mut buf = [0u8; 128];
    let original = Twist::new(
        Vector3::new(1.23, 4.56, 7.89),
        Vector3::new(-0.1, -0.2, 0.3),
    );
    let len = original.encode_cdr(&mut buf);
    assert!(len > 4);

    let decoded = Twist::decode_cdr(&buf[..len]).expect("Failed to decode Twist");
    assert_eq!(original, decoded);
}

#[test]
fn test_vle_roundtrip() {
    let mut buf = [0u8; 8];
    for val in [0, 1, 127, 128, 255, 300, 16384, 1000000] {
        let encoded_len = wire::encode_vle(val, &mut buf);
        let (decoded_val, consumed) = wire::decode_vle(&buf[..encoded_len]).expect("decode_vle failed");
        assert_eq!(val, decoded_val);
        assert_eq!(encoded_len, consumed);
    }
}

#[test]
fn test_zenoh_init_syn() {
    let mut buf = [0u8; 64];
    let zid = [0x12; 16];
    let len = ZenohWire::build_init_syn(&mut buf, &zid);
    assert_eq!(len, 22);
    assert_eq!(buf[0], 0x41);
    assert_eq!(buf[1], 0x09);
    assert_eq!(&buf[3..19], &zid);
}
