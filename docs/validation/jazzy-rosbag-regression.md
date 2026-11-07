# Native Jazzy ROS bag regression

This validation combines original serialized GO2 recordings, an independently
parsed official message contract, distribution comparisons, isolated DDS peers,
and timed replay. It does not establish current firmware interoperability,
command acceptance, hardware safety, or official Unitree support.

## Recording inventory and preservation

Discovery covered 458 existing recording files across research directories and
two mounted archives, plus one previously generated regression fixture. Counts
describe files and split parts, not independent physical trials. The remote development
workspace contains two previously selected recordings; their original serialized
CDR, rather than extracted numeric telemetry, supplied the earlier 9,675 samples.
The inventory contains 416 SQLite files, nine MCAP files, 32 Zstandard-compressed
SQLite files, and one unrelated ROS 1 fixture, totaling 147,491,155,861 bytes
including archival copies. Additional remote research mount searches found no
recordings. Unreachable lab machines and compressed general-purpose backups were
not searched internally.

The retained inventory records source paths, file hashes or an explicit hash
qualification, aliases, size, storage schema, duration, topics, counts, QoS,
embedded-definition availability, timestamp ordering, and corruption indicators.
File copies are deduplicated before comparing records. Global record identity is
SHA-256 over length-prefixed type and topic names, the original signed 64-bit
storage timestamp, and the complete original serialized payload. Replaying a
record repeatedly does not increase the independent-data count. Identical state
payloads at different timestamps remain distinct observations.

Originals were read through immutable SQLite connections and read-only container
mounts. Output paths reject source files, source-directory destinations, and
hardlink aliases of adjacent metadata. Nonempty WAL/journal files are rejected
rather than silently ignored. Full-file before/after hashes were checked for
replay and the smaller comparison inputs. Larger comparisons use explicitly
reported inode/size/mtime checks with separately inventoried hashes; this is not
represented as a repeated full-file content hash.

One 29.06 GB archive has no adjacent bag metadata; its embedded metadata is empty. Its full-container
integrity and current whole-file SHA-256 remain unverified. A single ordered
projection preserved every available native record's original ID, topic metadata,
storage timestamp, and exact CDR in a 1.94 GB derived SQLite subset. That subset
has a verified SHA-256 and passes SQLite integrity checks. Its 1,552,262 native
records count once against the original archive. This is native serialized-subset
coverage, not validation of the original cloud/image payloads or full bag. Its original SQLite ID range contains 107 unoccupied IDs, a source
missing-data indicator that cannot identify DDS loss or the absent message types.

Container checks passed for 428 SQLite storage paths, including decoded
compressed containers. Ten other original containers received projected checks
only; their full-container integrity remains unverified. Nine source paths lack a
current complete file hash. Forty-six paths lack adjacent metadata, and metadata
references 23 unavailable split files. Available source metadata counts match the
corresponding storage counts. Original file stat checks found no changes.

Ten recording paths have SQLite corruption/truncation indicators, including
archive copies. Header-declared size exceeds actual file size in these captures;
the capture/copy cause is unknown. They were inventoried and left unrepaired.
Missing metadata and absent original message definitions are recorded explicitly.
No extracted CSV/JSON telemetry is counted as full ROS bag coverage.

## Dataset counts and distribution results

There are 153 native-bearing original file identities after resolving archive
copies and six compressed-only parts. Twenty-five compressed containers preserve
the native topic/timestamp/CDR streams of already-tested raw files; one additional
vendor compressed fixture is byte-identical to its raw counterpart. The six new
compressed parts contribute 1,336,437 records, with no old/new or inter-part
identity overlap. Their exact native subsets were checked in both distributions.
Decompression and payload equivalence do not establish rosbag-player support for
every compression mode.

The original archives contain 16,843,482 unique native-labeled ROS record
identities. Of these, 4,958 are known synthetic API records. Another nine generated
telemetry fixture records are excluded from that original-archive total.
The remaining 16,838,524 non-synthetic archive records have separately qualified
provenance: 3,028,970 have documented hardware-capture context, 7,930,146 have
hardware archive labels, and 5,879,408 have unverified capture provenance. None
of these categories constitutes current firmware qualification.

| Recorded type | Unique original records | Known synthetic | Accepted non-synthetic payloads | Recorded schema failures |
| --- | ---: | ---: | ---: | ---: |
| `unitree_go/msg/LowState` | 8,197,318 | 0 | 8,197,318 | 0 |
| `unitree_go/msg/SportModeState` | 8,534,613 | 0 | 8,534,613 | 0 |
| `unitree_go/msg/WirelessController` | 12,124 | 0 | 12,124 | 0 |
| `unitree_api/msg/Request` | 76,634 | 2,479 | 74,155 | 0 |
| `unitree_api/msg/Response` | 20,880 | 2,479 | 18,401 | 0 |
| `unitree_go/msg/Go2FrontVideoData` | 1,913 | 0 | 0 | 1,913 |

There are 16,836,611 accepted non-synthetic native payloads and 1,913 incompatible
video payloads. These counts deduplicate archived record identities, not just
whole files, and do not include repeated replay executions. Original schema
provenance remains unverified where the capture contains no embedded definitions.

| Functional environment | Passed | Failed | Expected failure / skip |
| --- | ---: | ---: | ---: |
| Jazzy x86_64, native Cyclone DDS | 122 | 0 | 0 |
| Jazzy ARM64, QEMU user-mode on x86_64 | 122 | 0 | 0 |
| Humble x86_64, native Cyclone DDS | 121 | 0 | 1, exact float32 maximum in generated Python |

These totals count functional cases once and exclude CTest wrapper entries.
The isolated package-managed environments from the clean reproduction were
reused; affected example and regression targets were rebuilt. All five packages
and 24 examples remain build-tested. Native ARM hardware and hosted GitHub Actions
were not executed. Scoped changed Python files and new C++ tests pass lint;
unrelated upstream lint debt remains separate and was not reformatted.

## Independent correctness method

`test/unitree_ros2_test/compare_bags.py` accepts explicit SQLite files and never
publishes. It reads installed official `.msg` source definitions independently of
ROS-generated field metadata and RMW serializers. The source interface files are
unchanged from the upstream base. Generated root schemas must match the source
contract. Fixed layouts compile into independent bounds-checked struct layouts
and field accessors; variable layouts use the separate CDR decoder.

Every accepted record is checked against the independent decoder, including
integer widths/signs, nested fields, array dimensions, string/sequence lengths,
and floating-point field bits. Little-endian fixed-layout field bytes are compared
without normalizing their representations. Tests also cover independently
constructed big-endian CDR, negative zero, subnormal values, infinities, and a
quiet NaN payload. Field-level failures identify the topic, original record ID,
storage timestamp, and field or decoder error. Reports preserve ordered digests
of timestamps plus raw CDR, canonical typed fields, and reserialized CDR.

Round trips must preserve semantic field bytes. Whole-message byte identity is
reported separately because CDR alignment bytes can differ. Final alignment of
up to three unspecified bytes is accepted at a four-byte boundary. Observed
legacy SportModeState payloads contain 280 trailing zero bytes, accepted under
an explicit dataset-specific policy. This is not general permission to accept
unknown schema extensions; other unexplained trailers fail.

Humble and Jazzy process the same bytes independently. The Humble packaged
Fast DDS path provides a reference, with native Cyclone DDS also used for the
additional archives and functional tests. The Jazzy primary comparison explicitly
selects `rmw_cyclonedds_cpp`; additional Fast DDS results are retained separately.
An early default-middleware probe is identified in the evidence and is not used
as the sole native-Jazzy result. Across 153 original native-bearing identities
plus the generated fixture, 775 topic streams have matching distribution reports.
There are no missing distribution streams or unexplained semantic mismatches.
Two video streams account for all 1,913 known input failures. Per-topic
original-record and typed-field digests must agree; outgoing padding bytes need
not be identical.

The comparison preserves storage order and the actual published topic names,
including `/lowstate`, `/sportmodestate`, `/lf/sportmodestate`,
`/wirelesscontroller`, and the recorded API namespaces. Message timestamps, ticks,
and identity fields are preserved without treating them as the capture clock.
A bag cannot establish losses that occurred before recording or prove firmware
CRC acceptance.

## Official interface coverage

All 43 message definitions compile and pass an independent source-contract
synthetic check. There are no `.srv` definitions in the upstream repository.
Unitree API request/response topics are not ROS services. Nested coverage below
reuses its containing records and adds no independent samples. No row has live
firmware qualification from this session.

| Interface | Original recording coverage | Plain DDS software peer path |
| --- | --- | --- |
| `unitree_api/Request` | top-level accepted | yes |
| `unitree_api/RequestHeader` | nested accepted | yes |
| `unitree_api/RequestIdentity` | nested accepted | yes |
| `unitree_api/RequestLease` | nested accepted | yes |
| `unitree_api/RequestPolicy` | nested accepted | yes |
| `unitree_api/Response` | top-level accepted | yes |
| `unitree_api/ResponseHeader` | nested accepted | yes |
| `unitree_api/ResponseStatus` | nested accepted | yes |
| `unitree_go/AudioData` | none | unverified |
| `unitree_go/BmsCmd` | none | unverified |
| `unitree_go/BmsState` | nested accepted | yes |
| `unitree_go/Error` | none | unverified |
| `unitree_go/Go2FrontVideoData` | recorded schema mismatch | unverified |
| `unitree_go/HeightMap` | none | unverified |
| `unitree_go/IMUState` | nested accepted | yes |
| `unitree_go/InterfaceConfig` | none | unverified |
| `unitree_go/LidarState` | none | unverified |
| `unitree_go/LowCmd` | none | unverified |
| `unitree_go/LowState` | top-level accepted | yes |
| `unitree_go/MotorCmd` | none | unverified |
| `unitree_go/MotorCmds` | none | unverified |
| `unitree_go/MotorState` | nested accepted | yes |
| `unitree_go/MotorStates` | none | unverified |
| `unitree_go/PathPoint` | none | unverified |
| `unitree_go/Req` | none | unverified |
| `unitree_go/Res` | none | unverified |
| `unitree_go/SportModeCmd` | none | unverified |
| `unitree_go/SportModeState` | top-level accepted | yes |
| `unitree_go/TimeSpec` | nested accepted | yes |
| `unitree_go/UwbState` | none | unverified |
| `unitree_go/UwbSwitch` | none | unverified |
| `unitree_go/WirelessController` | top-level accepted | yes |
| `unitree_hg/BmsCmd` | none | unverified |
| `unitree_hg/BmsState` | none | unverified |
| `unitree_hg/HandCmd` | none | unverified |
| `unitree_hg/HandState` | none | unverified |
| `unitree_hg/IMUState` | none | yes |
| `unitree_hg/LowCmd` | none | unverified |
| `unitree_hg/LowState` | none | yes |
| `unitree_hg/MainBoardState` | none | unverified |
| `unitree_hg/MotorCmd` | none | unverified |
| `unitree_hg/MotorState` | none | yes |
| `unitree_hg/PressSensorState` | none | unverified |

Standard IMU, odometry, and Twist messages are also compared where recorded.
Point clouds, camera data, scans, transforms, and downstream custom interfaces
are inventoried; exhaustive payload validation of those streams is not claimed.
Standard ROS type names alone do not identify whether firmware or a downstream
project produced them. Missing downstream definitions do not change the official
Unitree contract. No UwbState or HG hardware data was available.

## Example and API coverage

All 24 executables remain build-tested on Jazzy x86_64, Jazzy ARM64 under QEMU,
and Humble x86_64. Runtime labels below refer to safe private-topic synthetic
checks. The real-data replay consumer is a separate regression node. Control
examples were not executed against hardware.

| Executable | Runtime coverage |
| --- | --- |
| `b2_sport_client` | build only; robot behavior unverified |
| `b2_stand_example` | build only; robot behavior unverified |
| `b2w_sport_client` | build only; robot behavior unverified |
| `b2w_stand_example` | build only; robot behavior unverified |
| `g1_ankle_swing_example` | build only; robot behavior unverified |
| `g1_arm_action_example` | build only; robot behavior unverified |
| `g1_arm_sdk_dds_example` | build only; robot behavior unverified |
| `g1_audio_client_example` | build only; robot behavior unverified |
| `g1_dex3_example` | build only; robot behavior unverified |
| `g1_dual_arm_example` | build only; robot behavior unverified |
| `g1_loco_client_example` | build only; robot behavior unverified |
| `g1_low_level_example` | build only; robot behavior unverified |
| `go2_robot_state_client` | build only; robot behavior unverified |
| `go2_sport_client` | build only; robot behavior unverified |
| `go2_stand_example` | build only; robot behavior unverified |
| `h2_ankle_swing_example` | build only; robot behavior unverified |
| `h2_loco_client` | build only; robot behavior unverified |
| `low_level_ctrl` | build only; robot behavior unverified |
| `low_level_ctrl_hg` | build only; robot behavior unverified |
| `read_low_state` | private telemetry/recorder check |
| `read_low_state_hg` | private telemetry/recorder check |
| `read_motion_state` | private telemetry/recorder check |
| `read_wireless_controller` | private telemetry/recorder check |
| `record_bag` | private telemetry/recorder check |

| API/client header | Additional validation |
| --- | --- |
| `b2/b2_motion_switch_client.hpp` | compilation; individual API semantics unverified |
| `common/b2_base_client.hpp` | independent response peer, correlation, errors, timeout and JSON |
| `common/base_client.hpp` | independent response peer, correlation, errors, timeout and JSON |
| `common/ros2_b2_sport_client.h` | compilation; individual API semantics unverified |
| `common/ros2_robot_state_client.h` | compilation; individual API semantics unverified |
| `common/ros2_sport_client.h` | compilation; individual API semantics unverified |
| `g1/g1_arm_action_client.hpp` | compilation; individual API semantics unverified |
| `g1/g1_audio_client.hpp` | compilation; individual API semantics unverified |
| `g1/g1_loco_client.hpp` | compilation; individual API semantics unverified |
| `g1/g1_motion_switch_client.hpp` | compilation; individual API semantics unverified |
| `h2/h2_loco_client.hpp` | compilation; individual API semantics unverified |
| `h2/h2_motion_switch_client.hpp` | compilation; individual API semantics unverified |

DDS tests additionally exercise GO/HG nested low states, sport timestamps,
wireless fields, and signed/unsigned API binary arrays against a separate plain
Cyclone DDS process. The previous 0.10.5/0.10.2 interoperability result is retained;
it is software-peer evidence rather than firmware qualification.

## Measured playback behavior

The stationary input supplies 5,682 LowState and 3,409 SportModeState records over
11.361 seconds, approximately 500 and 300 Hz respectively. The wireless input
supplies 584 records over 47.95 seconds. These 9,675 source records are a subset
of the inventory, regardless of how many replay cycles execute. Unique counts
identify archived ROS records, not independent physical trials or unique physical
sensor states.

Native x86_64 Jazzy measurements used domain 173, loopback-only Cyclone DDS,
unicast discovery, volatile durability, and depth 10,000. Runs occurred alongside
other validation work. Values describe the field-checking harness, not a robot
subscriber's throughput limit. Wall time includes two seconds of startup and
completion draining; peak RSS was sampled every 0.1 seconds.

| Stream / rate / reliability | Received | Wall s | Subscriber CPU s | Player CPU s | Peak subscriber KiB |
| --- | ---: | ---: | ---: | ---: | ---: |
| Low + Sport / 1x / reliable | 9,091 | 14.175 | 4.736 | 0.248 | 68,280 |
| Low + Sport / 2x / reliable | 9,091 | 8.468 | 4.020 | 0.236 | 79,456 |
| Low + Sport / 5x / reliable | 9,091 | 5.704 | 3.334 | 0.210 | 84,252 |
| Low + Sport / 5x / best effort | 9,091 | 5.717 | 3.349 | 0.195 | 86,420 |
| Wireless / 5x / best effort | 584 | 11.379 | 0.208 | 0.167 | 86,988 |

All final runs matched complete ordered field digests, with no unexplained loss
or duplication in those observable checks. A stream without an embedded sequence
or timestamp cannot distinguish compensating loss/duplication of identical
payloads; packet-level identity is not claimed. Both distributions also passed repeated 1x/2x/5x cycles, late
subscriber startup, subscriber destruction/recreation, final-sample reception,
and graph teardown. Volatile samples published while the subscriber was absent
are explicitly counted as intentionally unobserved. Controlled player termination
produced a nonzero exit and incomplete-count failure, with process cleanup.

After garbage collection across five cycles, Humble RSS was 55,584, 55,952,
59,824, 59,828, and 59,884 KiB. Jazzy RSS was 70,320, 72,488, 75,080, 75,212,
and 75,432 KiB. These short observations do not establish long-duration leak
freedom. Native ARM runtime stress was not performed.

Jazzy reports publisher scheduling drift, RMW receive-minus-source delay, and
callback dispatch delay separately. Retained reports using the earlier key
`dds_transport` describe the same queue-inclusive receive-delay metric. At 1x, receive-delay p99 was 0.198 ms for
LowState and 0.918 ms for SportModeState. At 5x, LowState p99 reached 989.583 ms
while the checker accumulated backlog, although delivery completed. This metric
includes middleware queues and is not isolated wire latency. Historical bag
epochs are never subtracted from current wall time. Humble's callback interface
provides no MessageInfo timing here, so its latency distributions are unavailable.

## Defects and corrections

- The common API client returned success with stale caller JSON on malformed or
  empty JSON responses, and C-string parsing hid bytes following a literal NUL.
  The JSON overload now reports the existing unknown-error code and preserves
  output on failure. Status-only calls still accept empty response data.
- The B2 client accepted the right request ID with the wrong API ID. Correlation
  now checks both fields under the response mutex and retains the first matching
  response. It also parses the complete JSON string. Its existing empty-response
  behavior remains an empty object. Both clients are checked by independent
  private-topic peers; no recorded API requests were published.
- Replay stopped draining two seconds after player exit while valid messages
  remained queued. A real 5x run received only the exact prefix 4,176/5,682 low
  states. A deterministic slow-checker fixture reproduces the failure. Draining
  now continues while callbacks progress under the existing hard deadline; the
  same real replay completes with exact counts and ordered fields.
- Direct native RMW deserialization can crash on buffers shorter than a CDR
  encapsulation. Replay now preflights encapsulation, fields and trailers before
  calling RMW. A regression proves malformed input never reaches it.
- Legacy two-component message type names are normalized before offline
  selection, with canonical names preserved under repeated normalization.
  Signed zero is checked, invalid booleans and bounded-string overflows are
  rejected, and inventory counts use one grouped metadata query. A metadata
  hardlink output regression prevents overwriting the original bag metadata.

## Fault injection and known incompatibilities

Bounded deterministic mutations use six original serialized messages, fixed seed
173, a 128 MiB worker address-space growth limit, and a three-second deadline.
Each distribution processed 163 original/mutated cases, including truncation,
incorrect encapsulation/options, length fields, unknown trailers, and bit changes.
Bit changes can be legal field values and are not automatically called corrupt.

The strict decoder rejects every input associated with a crashing RMW worker.
Direct Cyclone RMW calls nevertheless produced three signal exits in each bounded
run. Fresh-process checks without fork or address-space limits reproduced the
issue and sometimes accepted undersized buffers. This is an unresolved packaged
RMW limitation, not a passing malformed-input qualification. These API-level
experiments do not establish the behavior of a live DDS network receiver.

Seven additional empty-recording, missing/malformed-metadata, missing-split, and
incorrect-type cases fail safely in both distributions. Unspecified alignment
padding remains accepted. Original recordings were never mutated.

The exact largest finite float32 (bits `0x7f7fffff`) is accepted on Jazzy but
rejected by Humble's generated Python scalar range check, whose decimal bound
rounds down. Negative zero, subnormal values, infinities and quiet NaN pass.
A strict expected-failure test keeps this existing Humble generator limitation
visible. No installed ROS package or official message definition was patched.

Recorded Go2FrontVideoData does not match the official three-sequence schema.
In the first 406-packet capture, every record fits a diagnostic time/resolution/
length/H.264 layout instead; an independent media parser confirms 360p and 720p
content. Official Humble decoding also rejects it. Publisher introspection shows
a bare DDS producer and the official definition, but the actual producer schema
is unavailable. No replacement protocol, message rewrite, or adapter was added.
A second capture adds 1,507 packets with the same mismatch and diagnostic layout.
All 1,913 recorded video packets fail the official schema. Video compatibility
remains unresolved and these records are not accepted native field coverage.

## Reproduction

Build and source an isolated distribution using [the Jazzy instructions](../jazzy.md).
Use the same original files mounted read-only in the two distribution environments.
Keep reports outside source recording directories.

```bash
source setup_local.sh jazzy
python3 test/unitree_ros2_test/compare_bags.py /recordings/input_0.db3 \
  --standard-type sensor_msgs/msg/Imu \
  --standard-type nav_msgs/msg/Odometry --report /results/jazzy.json
```

Repeat with `source setup_local.sh humble` in Ubuntu 22.04 and a different report.
Compare each topic's `source_records_sha256`, `fields_sha256`, counts and errors.
`serialized_records_sha256` and `byte_identical` characterize encoding differences
rather than replacing semantic checks. The default verification hashes complete
input files before and after. For very large archived inputs, `--source-verification
stat` explicitly selects stat checks; use a read-only mount and retain a separately
qualified dataset manifest.

```bash
source setup_local.sh jazzy
python3 test/unitree_ros2_test/replay_bags.py /recordings/stationary \
  --rate 5 --reliability best_effort --report /results/replay-5x.json
colcon test --packages-select unitree_ros2_test --executor sequential \
  --return-code-on-test-failure
colcon test-result --test-result-base build/unitree_ros2_test --all --verbose
```

Run replay at 1x, 2x and 5x with distinct report names. Reliable is the default;
all replay topics are whitelisted telemetry and privately remapped. The saved
validation artifacts additionally contain the complete discovery/provenance
manifest, per-topic distribution reports, field-bit audit, lifecycle/stress
runners, source-subset lineage, fault inputs/worker results, JUnit, package
versions, original failures, and exact execution commands. They are not
redistributed as robot datasets in this repository.

Live GO2 firmware, other robot firmware, command acceptance, native ARM hardware,
network-link behavior, long-duration endurance and hosted GitHub CI remain
unverified. Next engineering work should establish the producer's video schema,
resolve the packaged RMW/Python-generator edge cases with their maintainers,
then qualify live read-only GO2 telemetry and execute hosted CI. Successful replay
cannot substitute for those gates.
