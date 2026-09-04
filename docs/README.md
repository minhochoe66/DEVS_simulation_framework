# Documentation

| Document | Contents |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | DEVS model hierarchy, per-atomic-model specification (states, ports, time advance), event flow, and a map from the article's figures and tables to the source files. |
| [CONFIGURATION.md](CONFIGURATION.md) | Complete schema of the four files in `JSON/`: run control, shop-floor layout, process routing, and robot/planner parameters. |
| [OUTPUTS.md](OUTPUTS.md) | Output directory layout, trajectory and analysis CSV formats, the eleven collected metrics, and how to replay a run. |

Start with the [project README](../README.md) for installation and a first run.

## Korean development notes

[`ko/`](ko/) holds the original Korean notes written during development:

- `monte-carlo-guide.ko.md` — Monte Carlo pipeline walkthrough
- `output-folder-structure.ko.md` — result folder structure and the reasoning behind it

They are kept for the authors' reference. Where they disagree with the documents above — the
folder-layout example in the Monte Carlo guide predates the current scheme — the English
documents are authoritative.
