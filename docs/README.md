# Documentation (TDK v1.7)

This directory is the user-manual source for **TDK v1.7**. It is published at https://flexivrobotics.github.io/flexiv_tdk/v1/. The site root opens v2 at https://flexivrobotics.github.io/flexiv_tdk/v2/, and the header on each site switches between the two lines.

## Local preview

```bash
pip install mkdocs-material
mkdocs serve
```

The site will be available at http://127.0.0.1:8000/.

## API reference

The C++ API is generated for v1 release tags that contain `docs/doxygen/Doxyfile.in`. `API_VERSION_PREFIX=v1` keeps `v1` and `v1.*`, and names ending in `_draft` are skipped. The script writes `docs/api/doxygen/<tag>/`, mirrors the latest tag to `docs/api/doxygen/`, and writes `docs/api/versions.json` for the dropdown.

To generate locally:

```bash
sudo apt install doxygen graphviz
API_VERSION_PREFIX=v1 bash docs/doxygen/build_api_docs.sh
mkdocs serve -a 127.0.0.1:8000
```

Open http://127.0.0.1:8000/api/ and switch API versions with the dropdown. Generated output under `docs/api/doxygen/` and `docs/api/versions.json` is git-ignored.