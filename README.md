## GNSSDataPath
A modular real-time GNSS data pipeline model for studying UART communication, buffering, timing behavior, and end-to-end correction-data transport in embedded systems.

## Build

Requires the Lingua Franca compiler (`lfc`) on your PATH.

From the project root:

```bash
make
```

Or directly:

```bash
lfc GNSSPipeline.lf
```

## Run

On Linux/WSL:

```bash
./GNSSPipeline
```

On Windows PowerShell:

```powershell
.\GNSSPipeline
```

## Clean

```bash
make clean
```

## What it contains

- `GNSSPipeline.lf`: the complete GNSS pipeline example
- `Makefile`: build and clean targets

The example models a GNSS receiver, a UART driver, a ring-buffer forwarder, and a SoC-side position engine.