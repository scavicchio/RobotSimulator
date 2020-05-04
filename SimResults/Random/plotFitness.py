import pandas as pd
import matplotlib.pyplot as plt

df=pd.read_csv('DistanceRandom.csv', sep=',',header=None)

print(df)

data = []
data.append(0)
y = []
y.append(0)
bestTMP = 0
columns = list(df) 
for item in columns: 
	#print(df[item][0])
	tmp = float(df[item][0])
	if (tmp >= data[-1]):
		bestTMP = tmp
		data.append(tmp)
	else:
		data.append(bestTMP)

plt.plot(data)
plt.show()