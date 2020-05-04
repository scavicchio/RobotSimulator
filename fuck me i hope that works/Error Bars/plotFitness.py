import pandas as pd
import matplotlib.pyplot as plt

df=pd.read_csv('weightedscoresHILLsaveFLipped.csv', sep=',',header=None)
df2=pd.read_csv('weightedscoresHILLsaveFLipped 2.csv', sep=',',header=None)
df3=pd.read_csv('weightedscoresHILLsaveFLipped 3.csv', sep=',',header=None)
df4=pd.read_csv('weightedscoresHILLsaveFLipped 4.csv', sep=',',header=None)
df5=pd.read_csv('weightedscoresHILLsaveFLipped 5.csv', sep=',',header=None)
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

x1 = 2000
y1 = df[x1][0]
array = 
std1 =  stdev()

x2 = 4000
y2 = df[x2][0]
std2 =  

x3 = 6000
y3 = df[x3][0]
std3 = 

x4 = 8000
y4 = df[x4][0]
std4 = 

x5 = 10000
y5 = df[x5][0]
std5 =  

plt.plot(data)

plt.errorbar(x1,y1,yerr=std1)
plt.errorbar(x2,y2,yerr=std2)
plt.errorbar(x3,y3,yerr=std3)
plt.errorbar(x4,y4,yerr=std4)
plt.errorbar(x5,y5,yerr=std5)


plt.show()