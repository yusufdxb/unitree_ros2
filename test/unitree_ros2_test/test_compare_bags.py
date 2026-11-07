"""Independent schema checks and bounded malformed-record regressions."""
import os
import struct

import check_bags
import compare_bags
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from test_bag_tool import recording, telemetry
from test_interfaces import interfaces, populate


def test_official_definitions_independently_decode_all_interfaces():
    definitions = compare_bags.Definitions()
    for typename in interfaces():
        cls = get_message(typename)
        message = populate(cls())
        raw = serialize_message(message)
        decoder = check_bags.CDR(raw, definitions.get)
        reference = decoder.message(definitions.get(typename))
        assert definitions.canonical(typename, reference) == definitions.canonical(
            typename, message), typename
        layout = definitions.fixed_layout(typename)
        if layout:
            little, big, packed, values, _ = layout
            assert packed.pack(*little.unpack_from(raw, 4)) == packed.pack(*values(message))
            # Big-endian wire input is constructed independently of the RMW serializer.
            big_endian = b'\x00\x00\x00\x00' + big.pack(*values(message))
            decoded = deserialize_message(big_endian, cls)
            assert definitions.canonical(typename, decoded) == definitions.canonical(
                typename, message), typename


def test_legacy_type_spelling_is_not_silently_omitted(tmp_path):
    typename = 'unitree_go/msg/LowState'
    path = recording(tmp_path, [('unitree_go/LowState', serialize_message(telemetry(typename)))])
    result, _ = check_bags.validate(
        path, get_message, deserialize_message, serialize_message, 0)
    assert result['passed'], result['errors']
    assert result['validated_messages'] == 1


def test_signed_zero_is_part_of_field_contract():
    with pytest.raises(AssertionError, match='zero sign changed'):
        check_bags.equal({'value': -0.0}, {'value': 0.0})


def test_legal_ieee_values_preserve_bits_in_native_deserialization():
    typename = 'unitree_go/msg/WirelessController'
    definitions = compare_bags.Definitions()
    cls = get_message(typename)
    raw = bytearray(serialize_message(telemetry(typename)))
    for bits in (0x80000000, 1, 0x7f800000, 0xff800000, 0x7fc01234):
        struct.pack_into('<I', raw, 4, bits)
        message = deserialize_message(bytes(raw), cls)
        assert struct.pack('<f', message.lx) == struct.pack('<I', bits)
        decoder = check_bags.CDR(raw, definitions.get)
        reference = decoder.message(definitions.get(typename))
        assert definitions.canonical(typename, reference) == definitions.canonical(
            typename, message)


@pytest.mark.xfail(os.environ.get('ROS_DISTRO') == 'humble', strict=True,
                   reason='Humble rosidl_generator_py rounds the float32 maximum down')
def test_float32_maximum_is_accepted_by_generated_python_interface():
    cls = get_message('unitree_go/msg/WirelessController')
    raw = bytearray(serialize_message(cls()))
    struct.pack_into('<I', raw, 4, 0x7f7fffff)
    assert struct.pack('<f', deserialize_message(bytes(raw), cls).lx) == raw[4:8]


def test_unfinalized_sqlite_is_not_silently_read_without_its_wal(tmp_path):
    path = tmp_path / 'active.db3'
    path.touch()
    sidecar = tmp_path / 'active.db3-wal'
    sidecar.write_bytes(b'pending transaction')
    with pytest.raises(ValueError, match='finalized SQLite'):
        check_bags.open_sqlite(path)
    assert sidecar.read_bytes() == b'pending transaction'


def test_invalid_boolean_and_unbounded_lengths_are_rejected():
    with pytest.raises(ValueError, match='invalid CDR boolean'):
        check_bags.CDR(b'\x00\x01\x00\x00\x02', get_message).field('boolean')
    for field in ('sequence<uint8>', 'string', 'sequence<unitree_go/MotorState>'):
        with pytest.raises(ValueError, match='invalid CDR'):
            check_bags.CDR(b'\x00\x01\x00\x00' + b'\xff' * 4, get_message).field(field)
    with pytest.raises(ValueError, match='exceeds declared bound'):
        check_bags.CDR(b'\x00\x01\x00\x00' + struct.pack('<I', 4) + b'abc\x00',
                       get_message).field('string<=2')


def test_invalid_encapsulation_and_all_truncation_boundaries():
    typename = 'unitree_go/msg/LowState'
    raw = serialize_message(telemetry(typename))
    for end in range(len(raw) - 3):
        with pytest.raises(ValueError):
            check_bags.CDR(raw[:end], get_message).message(get_message(typename))
    for header in (b'\x00\x03\x00\x00', b'\x00\x01\xff\xff'):
        with pytest.raises(ValueError, match='encapsulation'):
            check_bags.CDR(header + raw[4:], get_message)


def test_independent_report_reproduces_record_identity(tmp_path):
    typename = 'unitree_go/msg/LowState'
    path = recording(tmp_path, [(typename, serialize_message(telemetry(typename)))])
    result = compare_bags.inspect(path, compare_bags.Definitions(), get_message,
                                  deserialize_message, serialize_message, [], 0)
    assert result['passed'], result
    assert result['checked'] == 1
    assert result['topics'][0]['serialization_field_equivalent'] == 1
    assert result['source_unchanged']
