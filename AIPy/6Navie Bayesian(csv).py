import pandas as pd
import matplotlib.pyplot as plt
from sklearn.naive_bayes import GaussianNB
from sklearn.metrics import accuracy_score, confusion_matrix

# Load the data from CSV file
data = pd.read_csv('./data/6data.csv')

# Separate features and labels
features = data.iloc[:, :-1].values.astype(float)
labels = data.iloc[:, -1].values

# Split the data into training and test datasets
train_data = data.sample(frac=0.7, random_state=42)
test_data = data.drop(train_data.index)

# Separate features and labels in training dataset
train_features = train_data.iloc[:, :-1].values.astype(float)
train_labels = train_data.iloc[:, -1].values

# Train the Naive Bayes classifier
classifier = GaussianNB()
classifier.fit(train_features, train_labels)

# Separate features and labels in test dataset
test_features = test_data.iloc[:, :-1].values.astype(float)
test_labels = test_data.iloc[:, -1].values

# Predict the labels for test dataset
predicted_labels = classifier.predict(test_features)

# Compute the accuracy of the classifier
accuracy = accuracy_score(test_labels, predicted_labels)
print("Accuracy:", accuracy)

# Plot the training data
train_class1 = train_features[train_labels == 'Class1']
train_class2 = train_features[train_labels == 'Class2']
plt.scatter(train_class1[:, 0], train_class1[:, 1], c='red', label='Class1')
plt.scatter(train_class2[:, 0], train_class2[:, 1], c='blue', label='Class2')
plt.xlabel('Feature 1')
plt.ylabel('Feature 2')
plt.title('Training Data')
plt.legend()
plt.show()

# Plot the test data
test_class1 = test_features[test_labels == 'Class1']
test_class2 = test_features[test_labels == 'Class2']
plt.scatter(test_class1[:, 0], test_class1[:, 1], c='red', label='Class1')
plt.scatter(test_class2[:, 0], test_class2[:, 1], c='blue', label='Class2')
plt.xlabel('Feature 1')
plt.ylabel('Feature 2')
plt.title('Test Data')
plt.legend()
plt.show()

# Compute the confusion matrix
cm = confusion_matrix(test_labels, predicted_labels)
print("Confusion Matrix:")
print(cm)
