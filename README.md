# Short-Circuit-Benders

This solver uses Logic-Based Benders Decomposition (LBBD) for a VRP variant in a short-supply-chain setting. See the following publication for more details. **Publication:** * .*

**Third-party software:** this repository does not bundle [IBM CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio) or [BaPCod](https://bapcod.math.u-bordeaux.fr/). You must obtain and install them under their respective licenses.

## Build

**CPLEX** must be installed so CMake can find headers and libraries. The module `cmake/FindCPLEX.cmake` searches under `CPLEX_ROOT_DIR` (CMake cache variable). On Linux the default hint is `/opt/ibm/ILOG/CPLEX_Studio2211/`; override if needed, for example:

```bash
mkdir build && cd build
cmake .. -DCPLEX_ROOT_DIR=/path/to/CPLEX_StudioXXXX
cmake --build .
```

To build **without** BaPCod (default):

```bash
cmake .. -DBapCod=OFF
cmake --build .
```

### Optional: BaPCod

Install BaPCod with the RCSP library. Then adapt the **VehicleRoutingWithTimeWindows** demo: replace its sources with the files in the **`BapCodVRP/`** directory and **`include/`** , and build the static VRP library.

Create a **`libs/`** directory in this project and place:

- `libVRP_solver.a` — from your adapted demo build  
- `libbapcod_release.a` — from the BaPCod distribution  
- `librcsplinux.a` — RCSP library (Linux), from the BaPCod / RCSP build  

Configure paths when enabling BaPCod, for example:

```bash
cmake .. -DBapCod=ON \
  -DBapCod_PATH=/path/to/bapcod \
  -DBOOST_ROOT=/path/to/boost \
  -DCPLEX_ROOT_DIR=/path/to/CPLEX_StudioXXXX
cmake --build .
```

## Usage

```text
./Bender <instance.data> [options]
```

For a list of flags:

```text
./Bender --help
```

**Recommended run with BaPCod:**

```text
./Bender <instance.data> -FR=1 -BC=2 -IC=2 -GAP=2 -MS=20 -H=2 -YT=2 -FF=1 -TL=1800
```

**MILP only with CPLEX (no Benders main loop in the usual path):**

```text
./Bender <instance.data> -H=2 -CPL=1
```

## Instance format (10 lines)

1. **First line — 11 integers:**  
   `P C H K Nvh CapP WorkP CapH WorkH T TH`  
   - `P` — number of producers  
   - `C` — number of clients  
   - `H` — number of hubs  
   - `K` — number of products  
   - `Nvh` — number of vehicles per hub  
   - `CapP` — producer vehicle capacity  
   - `WorkP` — max distance a producer can travel in one period  
   - `CapH` — hub vehicle capacity  
   - `WorkH` — max distance a hub vehicle travels in one period  
   - `T` — number of periods  
   - `TH` — number of tours a hub vehicle may use per period  

2. **Producer stocks** (non-negative integers): for each producer, `K` values (may be zero).

3. **Client demands:** for each client, `K` requested amounts.

4. **Producer availability:** `P` binary vectors of length `T` (`1` = available that period).

5. **Client availability:** `C` binary vectors of length `T`.

6. **Client time windows:** for each client, `K` pairs `[earliest, latest]` (integers).

7. **Perishability:** `K` integers, number of periods a product can travel.

8. **Distance matrix:** `node * node` non-negative values, with `node = P + H + C + H` (the order is important). 

9. **Product sizes:** `K` integers.

10. **Coordinates:** for each of the `node` vertices, two numbers `x y` (producers, hub-related vertices, clients, in the same order as in the distance matrix / code).



## Instance Generator

You can generate new instances using the instance generator :

./GeneratorTrue.sh GenLPTrue.jl P C H PP CC T

- `P` — number of producers  
- `C` — number of clients  
- `H` — number of hubs  
- `K` — number of products
- `PP` — number of products per producers
- `CC` — number of producers producing the same set of products. For example with 6 producers if CC=2 then we have 3 groups of 2 producers having the same set of products. If CC=3 then we have 2 groups of 3 producers having the same set of products.

## Results
You can find individual results for each instance and each configurations, in the .csv files.

The helper script **`Instance/readSol.py`** loads every `*.csv` in those directories, merges them into an Excel workbook for inspection, and prints **LaTeX table fragments** summarizing the results.
This python script requires the dependencies openpyxl and pandas.

