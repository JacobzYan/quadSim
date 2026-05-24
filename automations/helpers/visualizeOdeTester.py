import pandas as pd
import sys
import matplotlib.pyplot as plot

try:
    csv_path = sys.argv[1]
except:
    print(f"Unable to read csv file")


df = pd.read_csv(csv_path)
t_name = df.columns[0]
nVar = len(df.columns)-1



fig = plot.figure()
for i,x in enumerate(df.columns[1:]):
    ax = fig.add_subplot(nVar,1,i+1)
    ax.plot(df[t_name], df[x])
    ax.grid()
    ax.set_xlabel(f'{t_name} [s]')
    ax.set_ylabel(x)

fig.savefig('automations/outputs/visualizeOdeTesterOutput.png')
print(f'graph saved to /output')