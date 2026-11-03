import base64
import importlib.util
from pathlib import Path

import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location(
    "g1_probe", Path(__file__).parents[1]/"scripts/g1_probe.py")
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


def message(values):
    data = np.asarray(values, dtype="<f4")
    return {"width": data.size, "height": 1, "step": data.size*4,
            "pixelFormatType": "R_FLOAT32",
            "data": base64.b64encode(data.tobytes()).decode("ascii")}


def test_gazebo_payload_preserves_unknown_near_far_statistics():
    result = PROBE.depth_summary(message([.5, 4, np.nan, np.inf, -np.inf]))
    assert result["finite_pixels"] == 2
    assert result["finite_min_m"] == .5 and result["finite_max_m"] == 4
    assert result["nan_pixels"] == result["positive_inf_pixels"] == result["negative_inf_pixels"] == 1
    assert not result["no_return_semantics_certified"]


def test_wrong_depth_byte_count_is_not_interpreted():
    corrupt = {**message([1, 2]), "data": base64.b64encode(b"1234").decode("ascii")}
    with pytest.raises(ValueError, match="payload size"):
        PROBE.depth_summary(corrupt)


@pytest.mark.parametrize("fields", [{"step": 100}, {"height": 0},
                                   {"pixelFormatType": "RGB_INT8"}])
def test_reject_wrong_image_format_stride_and_shape(fields):
    with pytest.raises(ValueError, match="metadata"):
        PROBE.depth_summary({**message([1]), **fields})
