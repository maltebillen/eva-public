# Visualisation
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import matplotlib.cbook as cbook
import matplotlib.dates as mdates
import datetime
import math
import os

COLOUR_MAP = {
    'DEADLEG':'#D3D3D3', 
    'TRIP':'#C8A2C8', 
    'CHARGING':'#758E4F',
    'MAINTENANCE':'#F4A900',
    'PUT_ON_CHARGE':'#758E75',
    'TAKE_OFF_CHARGE':'#758E75',
    'START_SCHEDULE':'#000000',
    'OUT_OF_ROTATION':'#F5F5DC'
    }
HEIGHT_MAP = {
    'DEADLEG':0.6, 
    'TRIP':0.8, 
    'CHARGING':0.8,
    'MAINTENANCE':0.8,
    'PUT_ON_CHARGE':0.8,
    'TAKE_OFF_CHARGE':0.8,
    'START_SCHEDULE':0.8,
    'OUT_OF_ROTATION':0.6
    }

def color(row):
    if row['Type'] != None:
        return COLOUR_MAP[row['Type']]
    else:
        return '#FFFFFF'
    
def height(row):
    if row['Type'] != None:
        return HEIGHT_MAP[row['Type']]
    else:
        return 0

def numberPlate(row):    
    return 'Bus ' + str(row['VehicleID']);
                
def prepareSchedule(dataPath):
    df_vis_schedule = pd.read_csv(dataPath + "outputs/VS_Output.csv")

    active_vehicles = (df_vis_schedule.groupby("VehicleID").count().Type > 0)
    active_vehicles = active_vehicles[active_vehicles == True].index.to_list()

    df_vis_schedule = df_vis_schedule.loc[df_vis_schedule.VehicleID.isin(active_vehicles)]
    df_vis_schedule['StartTime'] = df_vis_schedule['StartTime'].map(lambda x: datetime.datetime.strptime(x, '%Y-%m-%d %H:%M:%S%z'))
    df_vis_schedule['EndTime'] = df_vis_schedule['EndTime'].map(lambda x: datetime.datetime.strptime(x, '%Y-%m-%d %H:%M:%S%z'))
    df_vis_schedule['Duration'] = df_vis_schedule.EndTime - df_vis_schedule.StartTime
    df_vis_schedule['ChangeoverTime'] = df_vis_schedule.sort_values(['VehicleID','StartTime'],ascending=False).groupby('VehicleID', group_keys=False)[['StartTime','EndTime']].apply(lambda x: (x.shift(1).StartTime - x.EndTime).to_frame('change'))
    df_vis_schedule['NumberPlate'] = df_vis_schedule.apply(numberPlate, axis=1)
    df_vis_schedule['BarHeight'] = df_vis_schedule.apply(height, axis=1)

    if(len(df_vis_schedule) > 0):
        df_vis_schedule['Color'] = df_vis_schedule.apply(color, axis=1)

    return(df_vis_schedule)

def prepareUnallocatedTrips(dataPath):
    df_unallocatedTrips = pd.read_csv(dataPath + "outputs/VS_UnallocatedTrips.csv")
    df_unallocatedTrips['VehicleID'] = 'U'
    df_unallocatedTrips['StartTime'] = df_unallocatedTrips['StartTime'].map(lambda x: datetime.datetime.strptime(x, '%Y-%m-%d %H:%M:%S%z'))
    df_unallocatedTrips['EndTime'] = df_unallocatedTrips['EndTime'].map(lambda x: datetime.datetime.strptime(x, '%Y-%m-%d %H:%M:%S%z'))
    df_unallocatedTrips['Duration'] = df_unallocatedTrips.EndTime - df_unallocatedTrips.StartTime
    df_unallocatedTrips['Type'] = 'TRIP'
    
    if(len(df_unallocatedTrips) > 0):
        df_unallocatedTrips['Color'] = df_unallocatedTrips.apply(color, axis=1)
        df_unallocatedTrips['NumberPlate'] = df_unallocatedTrips.apply(numberPlate, axis=1)
        df_unallocatedTrips['BarHeight'] = df_unallocatedTrips.apply(height, axis=1)
   
    return(df_unallocatedTrips)

def visualiseSchedule(dataPath):
    # Check if Output was created:
    if os.path.isfile(dataPath + "outputs/VS_UnallocatedTrips.csv") and os.path.isfile(dataPath + "outputs/VS_Output.csv"):

        # Retrieve the unallocated trips, and bring in desired format:
        df_unallocatedTrips = prepareUnallocatedTrips(dataPath)

        # Retrieve the vehicle schedule, and concat the unassigned trips:
        df_vis_schedule = prepareSchedule(dataPath)
        df = df_vis_schedule
        if not df_unallocatedTrips.empty:
            df = pd.concat([df_vis_schedule, df_unallocatedTrips], ignore_index=True)            
            
        legend_elements = [Patch(facecolor=COLOUR_MAP[i], label=i)  for i in COLOUR_MAP]
        
        fig, ax = plt.subplots(1, figsize=(64, 36))
        ax.barh(y=df.NumberPlate, width=df.Duration, height=df.BarHeight, left=df.StartTime, color=df.Color, align="center")
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%b-%d %H:%M'))
        ax.set(xlim=[df.StartTime.min(), df.EndTime.max()])
        
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)
        plt.legend(handles=legend_elements,loc='upper right', fontsize=24)
        
        fig.savefig(dataPath + 'outputs/schedule.png', bbox_inches="tight")
        plt.close()
    else:
        print("Output files don't exist. Skip visualisation.")


def visualiseCharging(dataPath):
    df_vis_schedule = prepareSchedule(dataPath)
    
    nbBuses = len(df_vis_schedule.NumberPlate.unique())
    rowLength = math.ceil(nbBuses / 4)
    colLength = math.ceil(nbBuses/rowLength)
    multi = 10

    fig, axs = plt.subplots(rowLength, colLength, figsize=(colLength * multi, rowLength * multi))
    row, col = 0, 0

    for bus in df_vis_schedule.NumberPlate.unique():
        df_bus = df_vis_schedule.loc[df_vis_schedule.NumberPlate == bus].sort_values(by="StartTime").reset_index()
        
        if rowLength < 2:
            axis =  axs[col]
        else:
            axis =  axs[row, col]

        axis.scatter(df_bus.index,df_bus.StartSOC, color=df_bus.Color)
        axis.plot(df_bus.index,df_bus.StartSOC, color="lightblue")
        axis.set_title(bus)

        col += 1
        if col >= colLength:
            row += 1
            col = 0

    fig.tight_layout()
    fig.savefig(dataPath + 'outputs/charging.png')


def stats(dataPath):
    # Retrieve and prepare the dataset
    df_vis_schedule = prepareSchedule(dataPath)
    df_unallocated_trips = prepareUnallocatedTrips(dataPath)

    df_grouped = df_vis_schedule.groupby('NumberPlate', group_keys=False)

    # Add the statistics per vehicle: "statsVehicles.csv"
    df_stats_vehicles = pd.DataFrame(index=df_vis_schedule['NumberPlate'].unique())
    df_stats_vehicles['minChargingDuration'] = df_vis_schedule.loc[df_vis_schedule.ActivityType == 'Charging'].groupby(['NumberPlate']).Duration.min()
    df_stats_vehicles['numTrips'] = df_vis_schedule.groupby(['NumberPlate']).TripID.count()
    df_stats_vehicles['numService'] = df_vis_schedule.loc[df_vis_schedule.ActivityType == 'Service'].groupby(['NumberPlate']).ActivityType.count()
    df_stats_vehicles['totalUtilisationTime'] = df_grouped['Duration'].sum()
    df_stats_vehicles['totalIdleTime'] = df_grouped['ChangeoverTime'].sum()
    df_stats_vehicles['percentageIdleTime'] = df_grouped['ChangeoverTime'].sum()/df_grouped['Duration'].sum()
    df_stats_vehicles['longestIdleTime'] = df_grouped['ChangeoverTime'].max()
    df_stats_vehicles['longestTimeBetweenService'] = df_grouped['TimeBetweenService'].max()

    # HERE: Tomorrow - stats, performance, and write up.
    # Continue here coding more statistics that describe the quality of the schedule. 
    # E.g. deadlegs, line swaps

    df_stats_vehicles.to_csv(dataPath + "outputs/statsVehicles.csv")
    
