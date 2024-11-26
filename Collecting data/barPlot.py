import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd

Baseline_data = pd.read_csv(
    'https://github.com/polo-0831/Robotics-systems-Group/blob/master/Collecting%20data/Result_data_baseline.csv',
    sep=',',
    on_bad_lines='skip')
Improved_data = pd.read_csv(
    'https://github.com/polo-0831/Robotics-systems-Group/blob/master/Collecting%20data/Result_data_improved.csv',
    sep=',',
    on_bad_lines='skip')

# The following will produce a boxplot where each box collates
# (collects together) the measurements for all angles, at the
# distance intervals.
#
# Note that, we specify the data source for the plot with data=Data
# x axis set using the label column, 'Distance'
# y axis set using the label column, 'Measurement'
#

plt.figure(figsize=(12, 8))
print(Baseline_data.columns)

bplot = sns.boxplot(y='a', x='a0', data=Baseline_data, width=0.5)
bplot.set_title('Measurements for Each Distance from Obstacle')
bplot.set(xlabel="Distance (cm)", ylabel="Measurements (raw 8-bit ADC)")
