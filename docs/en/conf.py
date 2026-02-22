import os
import sys

import esp_docs.conf_docs as esp_docs_conf
from esp_docs.conf_docs import *  # noqa: F401,F403

docs_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if docs_root not in sys.path:
    sys.path.insert(0, docs_root)

from conf_common import *  # noqa: F401,F403

project = "LoRa Link Layer"
copyright = "2026, taruntom1"

master_doc = "index"
language = "en"

exclude_patterns = [
    "_build",
]


def setup(app):
    esp_docs_conf.setup(app)
    app.add_config_value("pdf_file", "", "html")
