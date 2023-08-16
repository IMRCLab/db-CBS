# db-CBS

## Get primitives

The primitives are on the TUB cloud. The easiest is to use a symlink:

```
ln -s /home/whoenig/tubCloud/projects/db-cbs/motions motions
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
