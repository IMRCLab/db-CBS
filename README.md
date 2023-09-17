# db-CBS

## Get primitives

The primitives are on the TUB cloud. The easiest is to use a symlink:

```
ln -s /home/${USER}/tubCloud/projects/db-cbs/motions motions
```

Alternatively, download a copy

```
wget https://tubcloud.tu-berlin.de/s/CijbRaJadf6JwH3/download
unzip download
rm download
```

## Building

Tested on Ubuntu 22.04.

```
mkdir buildRelease
cd buildRelease
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="/opt/openrobots/" ..
make -j
```

## Running

```
cd buildRelease
python3 ../scripts/benchmark.py
```
