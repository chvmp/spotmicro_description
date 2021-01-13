#!/usr/bin/env python

from configparser import ConfigParser

from attrdict import AttrDict


def parse_cfg(cfg):
    """Parse config.cfg

    Args:
        cfg (str): Path of config.cfg
    """
    parsed_cfg = ConfigParser()
    parsed_cfg.read(cfg, encoding="utf-8")

    out = AttrDict()
    names = ["lf", "lh", "rf", "rh"]
    for name in names:
        out[name] = _set_base_dict(parsed_cfg, name)
        out[name].update(_parse_length(parsed_cfg))

    out["test_pose"] = parsed_cfg["test"]["test_pose"]

    return out


def _set_base_dict(parsed_cfg, name):
    return {
        "bound": _parse_bound(parsed_cfg, name),
        "init": _parse_init(parsed_cfg, name),
        "shoulder": _parse_shoulder(parsed_cfg, name),
        "axis": _parse_axis(parsed_cfg, name),
        "test": _parse_test(parsed_cfg, name)
    }


def _parse_bound(parsed_cfg, name):
    out = {}
    for key in ["haa", "hfe", "kfe"]:
        tgt = name + "_" + key + "_bound"
        out[key] = parsed_cfg[name][tgt]

    return out


def _parse_init(parsed_cfg, name):
    out = {}
    out["angle"] = parsed_cfg[name][name + "_init"]
    out["position"] = parsed_cfg[name][name + "_init_position"]


def _parse_shoulder(parsed_cfg, name):
    out = parse_cfg[name].get(name + "_shoulder")
    if out is None:
        return [0., 0., 0.]
    return out


def _parse_axis(parsed_cfg, name):
    out = {}
    for i in range(1, 4):
        tgt = name + "_axis_{}".format(i)
        out[str(i)] = parse_cfg[name][tgt]
    return out


def _parse_length(parsed_cfg):
    out = {}
    for key, val in parsed_cfg["length"].items():
        out[key] = val
    return out


def _parse_test(parsed_cfg, name):
    out = parsed_cfg["test"]["test_joint_" + name]
    return out
