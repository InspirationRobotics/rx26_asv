"""Loopback tests for the inter-vehicle comms link (ivc_link).

ivc_link is ROS-free by design (its docstring says it runs "under the node, the
episode harness, and pytest") — this is that pytest coverage. Mirrors
test_robocomms_integration.py: a real server + client over localhost TCP, proving
framing, bidirectional exchange, backpressure, and health/teardown semantics."""
import socket
import threading
import time

from rx26_asv.api.ivc.ivc_link import IvcClient, IvcServer


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_for(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _connected_pair(**server_kw):
    """A live server<->client pair on localhost. Caller closes both."""
    port = free_port()
    server = IvcServer(port=port, **server_kw)
    threading.Thread(target=lambda: server.connect_or_serve(accept_timeout_s=5.0),
                     daemon=True).start()
    client = IvcClient(server_ip="127.0.0.1", port=port)
    assert wait_for(client.connect_or_serve), "client could not connect"
    assert wait_for(lambda: server.alive), "server never accepted"
    return server, client


def test_bidirectional_exchange():
    server, client = _connected_pair()
    try:
        assert client.send("hello")
        assert wait_for(lambda: server.queue.qsize() >= 1)
        assert server.get_next() == "hello"
        assert server.send("world")
        assert wait_for(lambda: client.queue.qsize() >= 1)
        assert client.get_next() == "world"
    finally:
        client.close()
        server.close()


def test_newline_framing_splits_concatenated_messages():
    server, client = _connected_pair()
    try:
        # raw write that packs 2.5 messages into one segment: the reader must
        # split on '\n' and buffer the partial tail until it completes.
        client.conn.sendall(b"one\ntwo\nthr")
        assert wait_for(lambda: server.queue.qsize() >= 2)
        assert server.get_next() == "one"
        assert server.get_next() == "two"
        client.conn.sendall(b"ee\n")
        assert wait_for(lambda: server.queue.qsize() >= 1)
        assert server.get_next() == "three"
    finally:
        client.close()
        server.close()


def test_peer_death_is_observable():
    server, client = _connected_pair()
    try:
        assert client.alive
        server.close()                       # peer vanishes mid-mission
        assert wait_for(lambda: not client.alive)
        h = client.health()
        assert h["alive"] is False and h["dead_reason"] is not None
    finally:
        client.close()


def test_clean_close_not_reported_as_death():
    server, client = _connected_pair()
    client.close()                           # our own orderly shutdown
    assert client.dead_reason is None
    server.close()


def test_send_on_down_link_returns_false():
    client = IvcClient(server_ip="127.0.0.1", port=free_port())  # never connected
    assert client.send("nobody home") is False


def test_full_queue_drops_are_counted_not_blocking():
    # backpressure: a slow consumer must never block the reader — overflow is
    # dropped and counted, loudly.
    server, client = _connected_pair(recv_maxlen=2)
    try:
        for i in range(6):
            client.send(f"m{i}")
        assert wait_for(lambda: server.dropped >= 4)
        assert server.queue.qsize() == 2      # capped, never grows unbounded
        assert server.health()["dropped"] >= 4
    finally:
        client.close()
        server.close()


# --- regression: a link that comes back up must not still report itself dead ---
# connect_or_serve() stamped dead_reason on failure and never cleared it on a
# later success, and `alive` requires dead_reason to be None. One transient
# failure therefore marked a working link dead for the life of the object.
#
# This is what made test_peer_death_is_observable flaky in CI: _connected_pair
# retries the (non-idempotent) client connect, so whenever the client lost the
# scheduling race with the server thread's bind/listen it connected on the
# second attempt and came up permanently "dead". Forced deterministically here
# rather than left to thread timing.

def _failed_then_successful_client():
    """A client whose FIRST connect attempt is refused, second succeeds."""
    port = free_port()                       # nothing listening on it yet
    client = IvcClient(server_ip="127.0.0.1", port=port, connect_timeout_s=1.0)
    assert client.connect_or_serve() is False, "expected the first attempt to fail"
    assert client.dead_reason is not None

    server = IvcServer(port=port)
    threading.Thread(target=lambda: server.connect_or_serve(accept_timeout_s=5.0),
                     daemon=True).start()
    assert wait_for(client.connect_or_serve), "client never reconnected"
    assert wait_for(lambda: server.alive), "server never accepted"
    return server, client


def test_reconnect_clears_the_previous_failure():
    server, client = _failed_then_successful_client()
    try:
        assert client.dead_reason is None, \
            f"stale failure survived a successful reconnect: {client.dead_reason}"
        assert client.alive, "reconnected link reports itself dead"
        assert client.health()["alive"] is True
    finally:
        client.close()
        server.close()


def test_reconnected_link_actually_carries_traffic():
    """Guards the inverse error: clearing dead_reason must not paper over a
    link that is not really up."""
    server, client = _failed_then_successful_client()
    try:
        assert client.send("after-reconnect")
        assert wait_for(lambda: server.queue.qsize() >= 1)
        assert server.get_next() == "after-reconnect"
    finally:
        client.close()
        server.close()


def test_reconnect_does_not_orphan_the_previous_reader():
    """_start_reader() overwrites _thread, so a reconnect over a live link
    would leave two readers draining into one queue."""
    server, client = _connected_pair()
    try:
        first_reader = client._thread
        server2 = IvcServer(port=client.port)
        server.close()                        # drop the original peer
        threading.Thread(
            target=lambda: server2.connect_or_serve(accept_timeout_s=5.0),
            daemon=True).start()
        assert wait_for(client.connect_or_serve), "client never reconnected"
        assert wait_for(lambda: not first_reader.is_alive()), \
            "previous reader thread still running alongside its replacement"
        assert client._thread is not first_reader
        assert client.alive
    finally:
        client.close()
        server2.close()
