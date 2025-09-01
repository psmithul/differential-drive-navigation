# Environment record

Python is pinned to [3.12.11](https://www.python.org/downloads/release/python-31211/), released on 3 June 2025. NumPy does the numerical work and Matplotlib writes plots. Tests use the standard library. There are no model services or runtime network calls.

Every package is pinned, including plotting dependencies and the installer. The hash lock permits only distributions uploaded before 1 September 2025. Dates and hashes were checked against PyPI JSON metadata.

| Package | Version | Release date |
| --- | --- | --- |
| [numpy](https://pypi.org/project/numpy/2.2.6/) | 2.2.6 | 2025-05-17 |
| [matplotlib](https://pypi.org/project/matplotlib/3.10.3/) | 3.10.3 | 2025-05-08 |
| [contourpy](https://pypi.org/project/contourpy/1.3.2/) | 1.3.2 | 2025-04-15 |
| [cycler](https://pypi.org/project/cycler/0.12.1/) | 0.12.1 | 2023-10-07 |
| [fonttools](https://pypi.org/project/fonttools/4.58.0/) | 4.58.0 | 2025-05-10 |
| [kiwisolver](https://pypi.org/project/kiwisolver/1.4.8/) | 1.4.8 | 2024-12-24 |
| [packaging](https://pypi.org/project/packaging/25.0/) | 25.0 | 2025-04-19 |
| [pillow](https://pypi.org/project/pillow/11.2.1/) | 11.2.1 | 2025-04-12 |
| [pyparsing](https://pypi.org/project/pyparsing/3.2.3/) | 3.2.3 | 2025-03-25 |
| [python-dateutil](https://pypi.org/project/python-dateutil/2.9.0.post0/) | 2.9.0.post0 | 2024-03-01 |
| [six](https://pypi.org/project/six/1.17.0/) | 1.17.0 | 2024-12-04 |
| [pip](https://pypi.org/project/pip/25.1.1/) | 25.1.1 | 2025-05-02 |

The machine used for reconstruction runs a current macOS release. These pins reproduce the Python runtime version and packages, not a complete 2025 operating-system image. The Git hosting service, repository creation time, and upload time are current; assigning old commit timestamps cannot change them.

`requirements.lock` records accepted SHA-256 distribution hashes. `dependencies.json` records release dates, latest accepted upload dates, and source links. `scripts/check_environment.py` checks the installed Python and package versions against that record.
