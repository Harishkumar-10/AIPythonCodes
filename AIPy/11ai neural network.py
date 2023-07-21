import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt

# Load the wheat-seed dataset from the CSV file
data = pd.read_csv('./data/wheat-seed-data.csv')

# Separate the features (X) and target variable (y)
X = data.drop('Class', axis=1).values
y = data['Class'].values

# Scale the features
scaler = StandardScaler()
X = scaler.fit_transform(X)

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Create an MLP classifier
mlp = MLPClassifier(hidden_layer_sizes=(16,), max_iter=1000, random_state=42)

# Train the MLP classifier
mlp.fit(X_train, y_train)

# Make predictions on the test set
y_pred = mlp.predict(X_test)

# Calculate the accuracy of the model
accuracy = accuracy_score(y_test, y_pred)
print("Accuracy:", accuracy)

# Generate a line graph to visualize the accuracy
epochs = np.arange(1, len(mlp.loss_curve_) + 1)
loss = mlp.loss_curve_

plt.plot(epochs, loss)
plt.xlabel('Epochs')
plt.ylabel('Loss')
plt.title('MLP Training Loss')
plt.show()
