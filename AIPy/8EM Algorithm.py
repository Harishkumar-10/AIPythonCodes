import numpy as np
import pandas as pd
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt

# Load data from CSV file
data = pd.read_csv('./data/data.csv')

# Convert data to numpy array
X = data.values

# Create an instance of Gaussian Mixture Model
gmm = GaussianMixture(n_components=3)  # Set the desired number of clusters

# Fit the model to the data
gmm.fit(X)

# Get the predicted cluster labels for each data point
labels = gmm.predict(X)

# Plot the data points with different colors for each cluster
plt.scatter(X[:, 0], X[:, 1], c=labels, cmap='viridis')
plt.title('EM Clustering Result')
plt.xlabel('Feature 1')
plt.ylabel('Feature 2')
plt.show()
