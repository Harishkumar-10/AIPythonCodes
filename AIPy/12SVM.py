import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm, datasets

# Load the iris dataset
iris = datasets.load_iris()
X = iris.data[:, :2]  # We only take the first two features for simplicity
y = iris.target

# Create an instance of SVM and fit the data
svm_classifier = svm.SVC(kernel='linear', C=1.0)
svm_classifier.fit(X, y)

# Create a mesh to plot in
x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1
y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
h = 0.02  # step size in the mesh
xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))

# Perform classification on the mesh grid points
Z = svm_classifier.predict(np.c_[xx.ravel(), yy.ravel()])

# Put the result into a color plot
Z = Z.reshape(xx.shape)
plt.contourf(xx, yy, Z, cmap=plt.cm.Paired)

# Plot the training points
plt.scatter(X[:, 0], X[:, 1], c=y, cmap=plt.cm.Paired, edgecolors='k')
plt.xlabel('Sepal Length')
plt.ylabel('Sepal Width')
plt.title('SVM Decision Boundary')
plt.show()
