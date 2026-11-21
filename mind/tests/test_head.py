"""The head goal (SET_HEAD, docs §3c layer 1): BodyClient.look / release_head
against the virtual body."""

import pytest

from kilo.body.client import HEAD_NOT_HELD, BodyClient
from kilo.link.serial_link import LinkConfig, SerialLink
from kilo.link.transport import loopback_pair
from kilo.protocol import kilo_protocol as P
from kilo.vbody.sim import VirtualBody

from conftest import FAST, keep_alive

HOLD_S = P.HEAD_HOLD_MS / 1000.0


@pytest.fixture
def rig():
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
    link.mark_alive()
    link.start()
    yield BodyClient(link), vb
    link.stop()
    vb.stop()


def _held(client):
    return "head_held" in client.state.flags and (client.state.owners or ("", ""))[1] == "PI_HOLD"


@pytest.mark.slow
def test_look_is_a_refreshed_goal_and_release_head(rig):
    client, vb = rig
    client.wait_ready(2)
    client.stand()
    f = client.look(30, -10.5)
    assert f == {"mode": 1, "pan": 480, "tilt": -168, "tail": HEAD_NOT_HELD, "max_speed_dps": 120}
    assert client.link.goals()["SET_HEAD"].epoch == client.state.epoch
    assert client.link.wait_until(lambda: _held(client), 1)
    assert vb.head == (480, -168, HEAD_NOT_HELD)
    # Refreshed at 5 Hz: still held well past HEAD_HOLD_MS.
    keep_alive(client.link, HOLD_S + 0.5)
    assert vb.head_until is not None and _held(client)
    # release_head: the goal is gone and the body lets go at once.
    client.release_head()
    assert "SET_HEAD" not in client.link.goals()
    assert client.link.wait_until(lambda: vb.head_until is None, 0.5)
    assert client.link.wait_until(lambda: "head_held" not in client.state.flags, 1)
    assert client.look(0, 0, tail_deg=12, speed_dps=0)["tail"] == 192
    with pytest.raises(ValueError):
        client.look(181, 0)
    with pytest.raises(ValueError):
        client.look(0, 0, speed_dps=-1)


@pytest.mark.slow
def test_look_dropped_by_an_epoch_bump(rig):
    client, vb = rig
    client.wait_ready(2)
    client.stand()
    client.look(20, 0)
    assert client.link.wait_until(lambda: vb.head_until is not None, 1)
    vb.trigger_fall(-1)
    assert client.link.wait_until(lambda: "SET_HEAD" not in client.link.goals(), 1)
    assert vb.head_until is None
    # After RECOVERED the body stands (doc §3g) and a new look works.
    assert client.link.wait_until(lambda: vb.posture == "STAND" and not vb.fallen, 3)
    client.look(-15, 5)
    assert client.link.wait_until(lambda: _held(client), 1)
