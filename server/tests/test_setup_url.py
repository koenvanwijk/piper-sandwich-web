from urllib.parse import parse_qs, urlsplit

from piper_sim_recorder.cli import DEFAULT_PAGE, setup_url


def test_setup_url_token_only_in_fragment():
    u = setup_url(DEFAULT_PAGE, "wss://pc.ts.net/ws", "geheim", {"front": (640, 480), "top": (640, 480)})
    p = urlsplit(u)
    assert p.scheme == "https" and p.path == "/piper-sandwich-web/record.html"
    assert p.query == "" and "geheim" not in p.path
    f = parse_qs(p.fragment)
    assert f == {"rec": ["wss://pc.ts.net/ws"], "token": ["geheim"], "cams": ["front,top"], "camsize": ["640x480"]}


def test_setup_url_mixed_sizes_no_camsize():
    f = parse_qs(urlsplit(setup_url("http://x/", "ws://localhost:1/ws", "", {"a": (1, 2), "b": (3, 4)})).fragment)
    assert "token" not in f and "camsize" not in f and f["cams"] == ["a,b"]
