import os

import pytest

from x4tags import config

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_example_config_loads():
    cfg = config.load(os.path.join(ROOT, "config.example.toml"))
    assert cfg.camera.size() == (2880, 1440) and cfg.tags.size_m == pytest.approx(0.1651)
    assert cfg.mount.z_m == 0.5 and cfg.output.nt_server is None


def test_defaults_without_a_file():
    cfg = config.load(None)
    assert cfg.detector.refine_bow_px == 0.2 and cfg.camera.device is None


def test_none_strings_and_unknown_keys(tmp_path):
    p = tmp_path / "c.toml"
    p.write_text('[detector]\nrefine_bow_px = "none"\n[tags]\nsize_m = "none"\n')
    cfg = config.load(str(p))
    assert cfg.detector.refine_bow_px is None and cfg.tags.size_m is None
    p.write_text("[detector]\nmin_tag = 3\n")
    with pytest.raises(ValueError, match="min_tag"):
        config.load(str(p))
    p.write_text("[cameras]\n")
    with pytest.raises(ValueError, match="cameras"):
        config.load(str(p))


def test_types_are_checked_and_team_numbers_need_no_quotes(tmp_path):
    p = tmp_path / "c.toml"
    p.write_text("[output]\nnt_server = 1234\n")
    assert config.load(str(p)).output.nt_server == "1234"
    for bad, match in (("[camera]\nreduce = 3\n", "reduce"), ('[mount]\nyaw_deg = "90"\n', "yaw_deg"),
                       ('[tags]\nsize_m = "auto"\n', "size_m"), ("[output]\nprint = 1\n", "print")):
        p.write_text(bad)
        with pytest.raises(ValueError, match=match):
            config.load(str(p))


def test_camera_mode_sets_the_size_and_latency(tmp_path):
    p = tmp_path / "c.toml"
    p.write_text('[camera]\nmode = "dual1080"\n')
    cfg = config.load(str(p))
    assert cfg.camera.size() == (1920, 1080) and cfg.camera.latency() == 112.0
    p.write_text('[camera]\nmode = "dual1080"\nlatency_ms = 90\nwidth = 960\nheight = 540\n')
    cfg = config.load(str(p))
    assert cfg.camera.size() == (960, 540) and cfg.camera.latency() == 90.0
    p.write_text('[camera]\nmode = "fisheye"\n')
    with pytest.raises(ValueError, match="mode"):
        config.load(str(p))


def test_width_and_height_must_match_the_mode(tmp_path):
    p = tmp_path / "c.toml"
    for body in ('[camera]\nmode = "dual1080"\nwidth = 2880\nheight = 1440\n', '[camera]\nwidth = 1920\n'):
        p.write_text(body)
        with pytest.raises(ValueError, match="width"):
            config.load(str(p))
