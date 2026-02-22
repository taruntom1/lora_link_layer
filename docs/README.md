# Documentation

This repository uses an ESP-IDF-style documentation pipeline:

- Sphinx sources in `docs/en`
- Doxygen configuration in `docs/doxygen/Doxyfile`
- API extraction from public headers in `components/lora_radio/include`

## Local build

From the repository root:

```bash
python -m pip install -r docs/requirements.txt
doxygen docs/doxygen/Doxyfile
build-docs -l en -t esp32s3 -- build
```

Outputs:

- Doxygen HTML: `docs/doxygen/html/index.html`
- Doxygen XML: `docs/doxygen/xml`
- Sphinx HTML: `docs/_build/en/esp32s3/index.html`

## Warning policy

- Sphinx warning baseline: `docs/sphinx-known-warnings.txt`
- Doxygen warning baseline: `docs/doxygen-known-warnings.txt`

CI fails on new warnings outside these baseline files.