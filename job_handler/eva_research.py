import subprocess
import tools

# Visualisation
import pandas as pd
import os
import json
import shutil

# Parameters:
#PATH_PARENT = 'C:/Users/s1996068/OneDrive - University of Edinburgh/Edinburgh/PhD/Research/EIBS/eva/'
#PATH_APP = PATH_PARENT + 'out/build/x64-release/app/EVA_Research.exe'
#PATH_DATA =  PATH_PARENT + 'job_handler/data/'

CTR_RUNS = 0
CTR_ERRORS = 0
LIST_ERRORS = []

# CONFIG CONSTANTS
class Param:
    def __init__(self, name, param_type, values, inFilename):
        self._name = name
        self._type = param_type
        self._values = values
        self._inFilename = inFilename

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    @property
    def values(self):
        return self._values
    
    @property
    def inFilename(self):
        return self._inFilename
    
    def __str__(self):
        return f"Name: {self._name}, Type: {self._type}, Values: {self._values}"

class Config:
    def __init__(self, config_list):
        self._CONFIG_LIST = config_list

    def all(self):
        result = [[]]

        # Step 1: Create all combinations of config:
        for param in self._CONFIG_LIST:
            prev_configs = result
            tmp_configurations = []

            for value in param.values:
                for config_list in prev_configs:
                    if(len(config_list) > 0):
                        new_config = config_list.copy()
                        new_config.append(Param(param.name, param.type, [value], param.inFilename))
                        tmp_configurations.append(new_config)
                    else:
                        new_config = [Param(param.name, param.type, [value], param.inFilename)]
                        tmp_configurations.append(new_config)
                
            result = tmp_configurations

        return result

def createConfig(config_json) -> Config:
    
    # Iterate over all params in the json_config data:
    config_list = list()
    for param in config_json:
        config_list.append(Param(param["name"], param["datatype"], param["values"], len(param["values"]) > 1))

    return Config(config_list)

def printAndChooseFolder(path) -> str:
    options = []
    n = 0
    
    for folder in sorted(os.listdir(path)):
        if(len(folder) > 0):
            if(os.path.isdir(path + folder) and folder[0] != '.'):
                print("[" + str(n) + "] " + str(folder))
                options.append(folder)
                n += 1

    print()
    while True:
        idx = input("Enter [#]: ")
        
        if(int(idx) < len(options)):
            return options[int(idx)]
        else:
            print("Invalid index. Number must be between 0 and " + str((len(options) - 1)) + ". Try again!") 

def optimise(pathApp, pathData, paramsConfig):
    global CTR_RUNS, CTR_ERRORS, LIST_ERRORS

    i = 0
    config = createConfig(paramsConfig)
    allConfigSettings = config.all()
    pathInstances = pathData + "instances/"

    if os.path.exists(pathInstances):
        shutil.rmtree(pathInstances)

    # Then, create an empty folder:
    os.mkdir(pathInstances)
    
    for config_list in allConfigSettings:
        dfConfig = pd.DataFrame(columns=['Parameter Key','Datatype','Value'])
        path_suffix = ""
        j = 0

        for param in config_list:
            dfConfig.loc[j] = [param.name, param.type, param.values[0]]
            if(param.inFilename):
                path_suffix = path_suffix + "_" + str(param.values[0])
            j += 1

        # Store Config dataframe, and create run-folder:
        path_config = pathInstances + "config_setting_" + path_suffix + "/"

        # First remove the folder if it exists already with all its contents:
        if os.path.exists(path_config):
            shutil.rmtree(path_config)
        
        # Then, create an empty folder:
        os.mkdir(path_config)
        dfConfig.to_csv(path_config + "config.csv", sep=',',index=False)

        # Run optimisation:
        try:
            CTR_RUNS += 1
            process = subprocess.run(args=[pathApp, str(pathData), str(path_config)], check=True)
            process.check_returncode()
                
            # Retrieve the results and create report:
            tools.visualisation.visualiseSchedule(path_config)
        except Exception as error:
            CTR_ERRORS += 1
            LIST_ERRORS.append(path_config)
            file = open(path_config + "errorLog.txt", "w")
            file.write(repr(error))
            file.close()

        # Prepare next:
        i = i + 1        
        print()  

def getPathExe() -> str:
    # Find the executable in the system's PATH
    executable_path = shutil.which(cmd="EVA_Research")

    if executable_path:
        print(f"Using Executable: {executable_path}")
        print()

        return executable_path
    else:
        raise ValueError("Executable not found. Make sure the directory is added to the PATH environment variables.")

def runEVA():
    global CTR_RUNS, CTR_ERRORS, LIST_ERRORS

    # Read the config params:
    curDir = os.path.dirname(os.path.abspath(__file__))
    runConfigFile = open(curDir + '/runConfig.json')
    runConfig = json.load(runConfigFile)
    paramsConfig = runConfig['paramsConfig']

    # Get the app and data paths:
    pathApp = getPathExe() # runConfig['pathExecutable']
    pathData = curDir + '/data/'

    # Display options to choose which network to optimise:
    print("Available regions:")
    regionFolder = printAndChooseFolder(pathData)
    pathData = pathData + regionFolder + "/"
    print("Available datasets:")
    networkSizeFolder = printAndChooseFolder(pathData)
    pathData = pathData + networkSizeFolder + "/"
    
    # Provide path to dataset:
    # Run the optimisation
    chooseFolder = input("Choose frequency? [y,n]: ")
    print()

    if(chooseFolder == 'y'):
        print("Available Frequencies:")
        frequency = printAndChooseFolder(pathData)
        cur_path_data = pathData + frequency + "/"

        print("Starting optimisation of network '" + pathData + "' with frequency '" + frequency + "'")

        optimise(pathApp, cur_path_data,paramsConfig)
    else:
        for frequency in os.listdir(pathData):
            cur_path_data = pathData + "/" + frequency + "/"

            print("Starting optimisation of network '" + pathData + "' with frequency '" + frequency + "'")

            optimise(pathApp, cur_path_data,paramsConfig)

    # Code a performance review for this instance:
    # ie. time, cost, bounds, gaps, dist. service coverage, # unallocated trips, # vehicles, ...
    print()
    print("Finished solving ", CTR_RUNS ," configurations with ", CTR_ERRORS, " errors.")
    if CTR_ERRORS > 0:
        print("Paths to config folder with error:")
        for err_path_config in LIST_ERRORS:
            print(err_path_config)

if __name__ == '__main__':
    try:
        runEVA()
    except ValueError as ve:
        print(f"Error: {ve}")