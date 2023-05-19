# db-CBS

## Preparation

The default version of OMPL 1.5.2 needs to be updated to at least 1.6.0. The following installs OMPL with the Python bindings

```
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod +x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh -p
```

## Building

Tested on Ubuntu 22.04.

```
mkdir buildRelease
cd buildRelease
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Running

```
cd buildRelease
python3 ../scripts/benchmark.py
```
