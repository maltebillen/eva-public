# Pre-requirements:
Follow these step-by-step guided to install the necessary libraries. Before getting started, ensure that `gcc` and `g++` compilers are installed.
I prefer to keep libraries tidied up in a folder `~/libs` and install them in a folder `~/local`. 

## Step 1: Install CMAKE.
- Navigate to library folder: `cd ~/libs`
- Download cmake: `wget https://cmake.org/files/v3.24/cmake-3.24.4.tar.gz`
- Extract the zipped folder: `tar -zxf cmake-3.24.4.tar.gz`
- Remove zip folder: `rm -r cmake-3.24.4.tar.gz`
- Navigate to cmake folder: `cd cmake-3.24.4`
- Create a build folder: `mkdir build`, and open it `cd build`
- Configure cmake: `./configure --prefix=$HOME/local/cmake`
- Build cmake: `gmake`
- Install cmake: `make install`
- Navigate to root folder: `cd`, and open bash file: `nano .bashrc`
- Add the following line to the bash file: `export PATH=$HOME/local/cmake/bin:$PATH`
- Close, and re-load the bash file. 
- Verify that cmake is running: `cmake --version`

## Step 2: Install HiGHS Solver.
- Navigate to library folder: `cd ~/libs`
- Download HiGHS release version: `wget https://github.com/ERGO-Code/HiGHS/archive/refs/tags/v1.9.0.tar.gz`
- Extract the zipped folder: `tar -zxf v1.9.0.tar.gz`
- Remove zip folder: `rm -r v1.9.0.tar.gz`
- Navigate to HiGHS folder: `cd HiGHS-1.9.0`
- Create a build folder: `mkdir build`, and open it `cd build`
- Configure with cmake: `cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/local/highs`
- Build HiGHS: `make`
- Install HiGHS: `make install`
- Navigate to root folder: `cd`, and open bash file: `nano .bashrc`
- Add the following line to the bash file: `export PATH=$HOME/local/highs/bin:$PATH`
- Add the following line to the bash file: `export LD_LIBRARY_PATH=$HOME/local/highs/lib:$LD_LIBRARY_PATH`
- Close, and re-load the bash file.
- Verify that cmake is running: `highs --version`

## Step 3: Install Boost Library.
- Navigate to library folder: `cd ~/libs`
- Download cmake: `wget https://archives.boost.io/release/1.84.0/source/boost_1_84_0.tar.gz`
- Extract the zipped folder: `tar -zxf boost_1_84_0.tar.gz`
- Remove zip folder: `rm -r boost_1_84_0.tar.gz`
- Navigate to HiGHS folder: `cd boost_1_84_0`
- Configure boost library: `./bootstrap.sh --prefix=$HOME/local/boost`
- Install boost libray: `./b2 install`
- Navigate to root folder: `cd`, and open bash file: `nano .bashrc`
- Add the following line to the bash file: `export BOOST_ROOT=$HOME/local/boost`
- Add the following line to the bash file: `export PATH=$BOOST_ROOT/bin:$PATH`
- Add the following line to the bash file: `export LD_LIBRARY_PATH=$BOOST_ROOT/lib:$LD_LIBRARY_PATH`
- Add the following line to the bash file: `export CPLUS_INCLUDE_PATH=$BOOST_ROOT/include:$CPLUS_INCLUDE_PATH`
- Close, and re-load the bash file.

# Installation of EVA-Research:
- Clone the git-folder or download into target folder. The root folder for the project is `/eva`.

## Compile the code:
- Create a build folder: `mkdir build`, and open it `cd build`
- Generate build files `cmake ..`
- Build EVA-Research: `cmake --build .`

## Running EVA-Research:
There are two different ways how the software can be run. It can be called directly from the terminal if only one specific config file should be executed, or via a python script.

### Version A: Terminal.
- Run the following command: `./build/app/EVA_Research [path_to_data] [path_to_config]`
- The command line arguments must be set:
    - path_to_data [PATH]: The path to the directory that contains the relevant data files of the instance. The path must end with '/'.
        - Example: `/home/eva/job_handler/data/scotland/toy/12/`
    - path_to_config [PATH]: The path where the config.csv file is located. The path must end with '/'.
        - Example: `/home/eva/job_handler/data/scotland/toy/12F/debug/`


### Version B: Using Python Script (Recommended).
Once:
- Navigate to root folder: `cd`, and open bash file: `nano .bashrc`
- Add the following line to the bash file: `export PATH=$HOME/..PATH_TO_EVA../eva/build/app:$PATH`. Note: Replace `..PATH_TO_EVA..` with the respective path leading to the root folder of the project. This is necessary for the python script to find the executable.
- Close, and re-load the bash file.

To run the script:
- Navigate to `~/eva/job_handler`
- Ensure that all python dependencies are installed. Recommended to use a virtual enviroment. If done, skip interim step:
    - Install all python libraries: `pip3 install -r requirements.txt`
- Run python script: `python3 eva_research.py`. This starts the UI.
- The UI in the terminal will guide through the selection of available datasets.

# Config parameters:
The behaviour of the model can be adapted by changing various config parameters. All available parameters are summarised in the following table. The configurations must be stored in `config.csv` with headers "Parameter,Datatype,Value". All parameters marked with an asterisk `*` are mandatory and must be added to the file.

| Parameter | Type | Values | Example | Description |   
|-----------|------|--------|---------|-------------|
| *`DATE_START` | `datetime` | YYYY-MM-DD hh:mm:ss+zz:zz | 2024-05-01 00:00:00+00:00 | The start date of the planning horizon. |
| *`DATE_END` | `datetime` | YYYY-MM-DD hh:mm:ss+zz:zz | 2024-06-15 00:00:00+00:00 | The end date of the planning horizon. |
| `CONST_PLANNING_HORIZON_LENGTH` | `uint` | [seconds] | 43200 | The length of the decision horizon. |
| `CONST_PLANNING_HORIZON_OVERLAP` | `uint` | [seconds] | 86400 | The length of the forecast horizon.  |
| `CONST_PUT_VEHICLE_ON_CHARGE` | `uint` | [seconds] | 300 | The buffer time before charging. |
| `CONST_TAKE_VEHICLE_OFF_CHARGE` | `uint` | [seconds] | 300 | The buffer time after charging. |
| `CONST_CHARGER_CAPACITY_CHECK` | `uint` | [seconds] | 300 | The time between checking charger capacities. |
| `CONST_LINEAR_OPTIMALITY_GAP` | `double` | [0,1] | 0.001 | The convergence criteria for column generation. |
| `CONST_INTEGER_OPTIMALITY_GAP` | `double` | [0,1] | 0.001 | The accepted integer optimality gap. |
| `CONST_FRAC_THRESHOLD_TRUNC_CG` | `double` | [0,1] | 0.9 | The threshold to fix variables greater than in the truncated column generation. |
| `CONST_COLUMN_GENERATION_TIMELIMIT` | `uint` | [seconds] | 7200 | The maximum duration for column generation. |
| `CONST_BRANCH_AND_PRICE_TIMELIMIT` | `uint` | [seconds] | 7200 | The maximum duration for finding integer solutions. |
| `CONST_NR_THREADS` | `uint` | [#] | 8 | The maximum number of threads allowed. |
| `CONST_NR_COLS_PER_VEHICLE_ITER` | `uint` | [#] | 200 | The maximum number of columns added per vehicle per column generation iteration. |
| `CONST_NTH_ITER_SOLVE_ALL` | `uint` | [#] | 20 | Every n-th iteration solve for negative reduced cost columns for all vehicles. Only if `FLAG_INTERIM_SOLVE_ALL_VEHICLES` is on. |
| `CONST_NTH_BRANCHING_NODE_DIVE` | `uint` | [#] | 10 | Every n-th branching node, perform a quick dive in the branch-and-price best-first search tree. |
| `CONST_NTH_ITER_MODEL_CLEAN` | `uint` | [#] | 10 | Every n-th iteration clean the model from repeated positive reduced cost columns. Only if `FLAG_USE_MODEL_CLEANUP` is on. |
| `CONST_DEL_CONSECUTIVE_POS_RC` | `uint` | [#] | 25 | The number of repeatedly positive reduced cost values to be marked for clean-up. |
| `CONST_CODE_ALGORITHM_TYPE` | `uint` | {0,1,2,3,4,5} | 4 | The algorithm types. 0: Price-and-Branch, 1: Best-First BnP, 2: Depth-First BnP, 3: Truncated CG, 4: Truncated CG + Best-First BnP, 5: Truncated CG + Depth-First BnP (recommended). |
| `CONST_CODE_PRICING_PROBLEM_TYPE` | `uint` | {0,1,2} | 1 | The pricing problem types: 0: Time-Space Network, 1: Connection Segment-Based Network, 2: Centralised Timeline Segment-Based Network |
| `CONST_MAX_NUMBER_FIRST_TIER_EVAL_STRONG_BRANCHING` | `uint` | {0,&infin;} | 50 | The number of candidate branches in first evaluation of strong branching. |
| `CONST_MAX_NUMBER_SECOND_TIER_EVAL_STRONG_BRANCHING` | `uint` | {0,&infin;} | 25 | The number of candidate branches in second evaluation of strong branching. |
| `FLAG_MINIMISE_NUMBER_VEHICLES` | `bool` | {false,true} | false | Signal if the number of vehicles should be minimised: 0: No, 1: Yes. |
| `FLAG_USE_MODEL_CLEANUP` | `bool` | {false,true} | true | Signal if the model should be cleaned-up. 0: No, 1: Yes. |
| `FLAG_INTERIM_SOLVE_ALL_VEHICLES` | `bool` | {false,true} | true | Signal if the model should solve sometimes all vehicles. 0: No, 1: Yes. |
| `COST_DEADLEG_FIX` | `double` | [0,&infin;] | 5 | The fixed cost coefficient for all deadlegs. |
| `COST_DEADLEG_PER_KM` | `double` | [0,&infin;] | 2 | The flexible cost coefficient for every kilometre driven on a deadleg. |
| `COST_COEFFICIENT_PENALTY_MAINTENANCE` | `double` | [0,&infin;] | 0.002 | The cost coefficient for the cleanliness function. |
| `COST_UNCOVERED_TRIP` | `double` | [0,&infin;] | 20000 | The fixed cost coefficient for an unassigned trip. |

If the python script is used to run the software, the file `runConfig.json` can be updated to modify config values. It allows to add multiple different values for every config parameter. The python script will automatically create all parameter combinations and run all configurations.

# Datasets:
