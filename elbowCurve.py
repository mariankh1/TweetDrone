import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import seaborn as sns; sns.set()
import csv
df = pd.read_csv("datasets/Sandy500text.csv")
df.head()
BBox = [df.lon.min(),df.lon.max(),df.lat.min(),df.lat.max()]

K_clusters = range(1,10)
kmeans = [KMeans(n_clusters=i) for i in K_clusters]

Y_axis = df[['lat']]
X_axis = df[['lon']]
score = [kmeans[i].fit(Y_axis).score(Y_axis) for i in range(len(kmeans))]

# Visualize
#plt.plot(K_clusters, score)

#plt.xlabel('Number of Clusters')
#plt.ylabel('Score')
#plt.title('Elbow Curve')
#plt.show()
X=df.loc[:,['id','tweet','lat','lon']]
kmeans = KMeans(n_clusters = 20, init ='k-means++')
kmeans.fit(X[X.columns[2:4]]) # Compute k-means clustering.
X['cluster_label'] = kmeans.fit_predict(X[X.columns[2:4]])
centers = kmeans.cluster_centers_ # Coordinates of cluster centers.
labels = kmeans.predict(X[X.columns[2:4]]) # Labels of each pointX.head(10)
print (X.head(10))

X.plot.scatter(x = 'lat', y = 'lon', c=labels, s=50, cmap='viridis')
plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)
print (centers)
plt.show()
