import pandas as pd
import numpy as np
import plotly.express as px

df = pd.read_csv('message.txt', sep=' ', header=None)

print(df)

fig = px.scatter(df, x=5, y=4)
fig.show()

