"""The startup process grace must never grant a stale flight data link."""
import importlib.util
from pathlib import Path
import pytest

SCRIPT=Path(__file__).resolve().parents[1]/"scripts/p2_sitl_gcs.py"
spec=importlib.util.spec_from_file_location("p2_gcs_startup",SCRIPT)
gcs=importlib.util.module_from_spec(spec);spec.loader.exec_module(gcs)


def test_disarmed_lockstep_pause_waits_without_becoming_healthy():
    assert gcs.heartbeat_watchdog_failure(0,50,120,10,False) is None
    assert not gcs.heartbeat_link_healthy(True,2533,10,10,50)
    assert gcs.heartbeat_watchdog_failure(0,53,120,52.5,False) is None
    assert gcs.heartbeat_link_healthy(True,2534,11,52.5,53)


@pytest.mark.parametrize("last,now,failed", [(None,119,False),(1,120,False),
    (None,120.001,True),(119.9,120.001,False),(114.9,120.001,True),
    (10,121,True),(499.9,500,False)])
def test_fixed_deadline_ends_only_stale_heartbeat_grace(last,now,failed):
    result=gcs.heartbeat_watchdog_failure(0,now,120,last,False)
    assert (result is not None)==failed


@pytest.mark.parametrize("now,last,failed", [(15,10,False),(15.001,10,True),
    (501,500.5,False),(501,490,True),(10,None,True)])
def test_once_armed_the_original_five_second_watchdog_remains(now,last,failed):
    # armed_heartbeat_seen is a permanent latch, including after a later
    # disarmed heartbeat; a long mission does not re-enter startup grace.
    result=gcs.heartbeat_watchdog_failure(0,now,120,last,True)
    assert (result is not None)==failed


@pytest.mark.parametrize("timeout", [0,-1,float("nan"),float("inf")])
def test_unbounded_or_invalid_startup_timeout_is_rejected(timeout):
    with pytest.raises(ValueError):gcs.heartbeat_watchdog_failure(0,1,timeout,None,False)


def test_link_health_stays_stricter_than_process_watchdog():
    assert gcs.heartbeat_watchdog_failure(0,14,120,10,True) is None
    assert not gcs.heartbeat_link_healthy(True,2,2,10,14)
    assert gcs.heartbeat_link_healthy(True,2,2,10,13)
    assert not gcs.heartbeat_link_healthy(True,2,1,10,11)
    assert not gcs.heartbeat_link_healthy(False,2,2,10,11)


def test_image_chain_can_keep_a_fresh_disarmed_link_after_startup_deadline():
    assert gcs.heartbeat_watchdog_failure(0,500,120,499.5,False) is None
    assert gcs.heartbeat_link_healthy(True,100,100,499.5,500)


def test_slow_lockstep_requests_real_heartbeats_and_ack_never_counts_as_heartbeat(tmp_path):
    """A protocol fixture replies only to read-only requests, not a flight proof."""
    import socket
    import subprocess
    import sys
    import time
    import json
    mavlink=pytest.importorskip('pymavlink.dialects.v20.common')
    server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    server.bind(('127.0.0.1',0));server.settimeout(.05)
    reservation=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    reservation.bind(('127.0.0.1',0));listen_port=reservation.getsockname()[1];reservation.close()
    destination=('127.0.0.1',listen_port)
    class Writer:
        def write(self,packet):server.sendto(packet,destination)
    protocol=mavlink.MAVLink(Writer(),srcSystem=1,srcComponent=1)
    process=subprocess.Popen([sys.executable,str(SCRIPT),'--output-dir',str(tmp_path),
        '--listen-port',str(listen_port),'--expected-px4-port',str(server.getsockname()[1]),
        '--startup-timeout','120'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    def health():
        try:return json.loads((tmp_path/'gcs_health.json').read_text())
        except (FileNotFoundError,json.JSONDecodeError):return {}
    try:
        # It must not transmit to a peer until a real PX4-format HB verifies it.
        ready_deadline=time.monotonic()+5
        while not health().get('running') and time.monotonic()<ready_deadline:
            assert process.poll() is None
            time.sleep(.02)
        assert health().get('running')
        with pytest.raises(socket.timeout):server.recvfrom(65535)
        protocol.heartbeat_send(mavlink.MAV_TYPE_QUADROTOR,mavlink.MAV_AUTOPILOT_PX4,
                                0,0,mavlink.MAV_STATE_STANDBY)
        requested=0;deadline=time.monotonic()+3.5
        while time.monotonic()<deadline and (requested<2 or not health().get('link_healthy')):
            try:packet,address=server.recvfrom(65535)
            except socket.timeout:continue
            assert address==destination
            for message in protocol.parse_buffer(packet) or []:
                if message.get_type()=='COMMAND_LONG':
                    assert message.command==mavlink.MAV_CMD_REQUEST_MESSAGE
                    assert message.param1==mavlink.MAVLINK_MSG_ID_HEARTBEAT
                    assert message.target_system==message.target_component==1
                    requested+=1
                    protocol.command_ack_send(mavlink.MAV_CMD_REQUEST_MESSAGE,mavlink.MAV_RESULT_ACCEPTED,
                                              0,0,255,190)
                    protocol.heartbeat_send(mavlink.MAV_TYPE_QUADROTOR,mavlink.MAV_AUTOPILOT_PX4,
                                            0,0,mavlink.MAV_STATE_STANDBY)
                else:assert message.get_type() in ('HEARTBEAT','PARAM_REQUEST_READ')
        assert requested>=2,'slow lockstep has no wall-time read-only heartbeat polling'
        row=health()
        assert row['link_healthy'] and row['heartbeat_request_acks_accepted']>=2
        assert row['heartbeat_requests_sent']>=2
        count=row['autopilot_heartbeats_received'];last=row['last_autopilot_heartbeat_wall']
        # Keep accepting telemetry requests but withhold actual HB replies.
        # ACK receipt alone must never keep the original 3-wall-second gate open.
        deadline=time.monotonic()+4.3
        while time.monotonic()<deadline:
            try:packet,_=server.recvfrom(65535)
            except socket.timeout:continue
            for message in protocol.parse_buffer(packet) or []:
                if message.get_type()=='COMMAND_LONG':
                    protocol.command_ack_send(mavlink.MAV_CMD_REQUEST_MESSAGE,mavlink.MAV_RESULT_ACCEPTED,
                                              0,0,255,190)
        row=health()
        assert row['heartbeat_request_acks_accepted']>2
        assert row['autopilot_heartbeats_received']==count
        assert row['last_autopilot_heartbeat_wall']==last
        assert not row['link_healthy']
    finally:
        process.terminate();process.communicate(timeout=3);server.close()
