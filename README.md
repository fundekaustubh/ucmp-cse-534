# UCMP for Reconfigurable Data Center Networks (SIGCOMM 2024 Paper Reproduction)

This repository provides an ns-3 based miniature implementation of **Uniform-Cost Multi-Path Routing (UCMP)**, inspired by the SIGCOMM 2024 paper on UCMP for Reconfigurable Data Center Networks.

UCMP is implemented as a new routing protocol in ns-3 and demonstrated through a custom simulation scenario to evaluate its behavior and performance under time-sliced dynamic network configurations and incorporation of ECN-marking for cost computation.

## Project Structure

The repository contains:

<pre>
ns-allinone-3.35/
├── ns-3.35/
│   ├── src/internet/model/
│   │   ├── ucmp-routing.cc
│   │   ├── ucmp-routing.h
│   ├── src/internet/helper/
│   │   ├── ucmp-routing-helper.cc
│   │   ├── ucmp-routing-helper.h
│   ├── scratch/
│   │   ├── ucmp-slice-simulator.cc
</pre>


## How to Build and Run

### Install dependencies

**On Ubuntu/Debian**:

```
sudo apt update
sudo apt install build-essential g++ python3 python3-dev python3-setuptools git mercurial cmake ninja-build pkg-config qt5-default
```
These packages are needed to build ns-3 and run simulations.

###  Build ns-3 with UCMP
```
cd ns-allinone-3.35/ns-3.35
```

### Configure the project (enable examples and tests to avoid build issues)
```
./waf configure --enable-examples --enable-tests
```

### Build the project
```
./waf build
```

### Run the main UCMP simulation script
```
./waf --run scratch/ucmp-slice-simulator
```
The simulation will output log data and routing actions directly to the console.

## Notes
The project is self-contained. No need to download ns-3 separately — it is included in ns-allinone-3.35/.

All UCMP routing protocol source files are already placed in the appropriate src/internet/ directories.