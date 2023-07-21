import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score

# Example dataset
X = np.array([[2.7810836, 2.550537003],
              [1.465489372, 2.362125076],
              [3.396561688, 4.400293529],
              [1.38807019, 1.850220317],
              [3.06407232, 3.005305973],
              [7.627531214, 2.759262235],
              [5.332441248, 2.088626775],
              [6.922596716, 1.77106367],
              [8.675418651, -0.242068655],
              [7.673756466, 3.508563011]])
y = np.array([0, 0, 0, 0, 0, 1, 1, 1, 1, 1])

# Split the dataset into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Create a k-NN classifier with k=3
k = 3
knn = KNeighborsClassifier(n_neighbors=k)

# Train the classifier
knn.fit(X_train, y_train)

# Make predictions on the test set
y_pred = knn.predict(X_test)

# Calculate the accuracy of the classifier
accuracy = accuracy_score(y_test, y_pred)
print(f"Accuracy: {accuracy}")

# Plot the decision boundaries
h = 0.02  # Step size in the mesh
x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1
y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
xx, yy = np.meshgrid(np.arange(x_min, x_max, h),
                     np.arange(y_min, y_max, h))
Z = knn.predict(np.c_[xx.ravel(), yy.ravel()])

# Create a color map
cmap = ListedColormap(['#FFAAAA', '#AAFFAA'])

# Plot the filled contour
Z = Z.reshape(xx.shape)
plt.figure()
plt.contourf(xx, yy, Z, cmap=cmap, alpha=0.8)

# Plot the training points
plt.scatter(X_train[:, 0], X_train[:, 1], c=y_train, cmap=cmap, edgecolor='k')
plt.xlabel('Feature 1')
plt.ylabel('Feature 2')
plt.title('k-NN Classifier (k=3) - Decision Boundaries')
plt.show()
