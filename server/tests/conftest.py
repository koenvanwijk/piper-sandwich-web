import io
import os

import numpy as np
import pytest
from PIL import Image

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.pop("HF_TOKEN", None)                   # tests gebruiken NOOIT een echt token


def jpeg(w=64, h=48, color=(120, 60, 30)) -> bytes:
    b = io.BytesIO()
    Image.new("RGB", (w, h), color).save(b, "JPEG")
    return b.getvalue()


@pytest.fixture
def jpg():
    return jpeg
