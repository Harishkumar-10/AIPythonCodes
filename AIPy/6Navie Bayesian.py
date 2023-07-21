import numpy as np
import matplotlib.pyplot as plt
from sklearn.naive_bayes import GaussianNB
from sklearn.metrics import accuracy_score

# Sample training dataset
train_data = np.array([
    [6.0, 2.2, 'Class1'],
    [5.0, 3.2, 'Class2'],
    [4.9, 3.1, 'Class2'],
    [5.5, 2.3, 'Class1'],
    [6.1, 3.0, 'Class1'],
    [4.7, 3.2, 'Class2']
])

# Sample test datasets
test_data = np.array([
    [5.9, 2.8, 'Class1'],
    [4.8, 3.0, 'Class2'],
    [5.3, 2.4, 'Class1']
])

# Separate features and labels in training dataset
train_features = train_data[:, :-1].astype(float)
train_labels = train_data[:, -1]

# Train the Naive Bayes classifier
classifier = GaussianNB()
classifier.fit(train_features, train_labels)

# Separate features and labels in test datasets
test_features = test_data[:, :-1].astype(float)
test_labels = test_data[:, -1]

# Predict the labels for test datasets
predicted_labels = classifier.predict(test_features)

# Compute the accuracy of the classifier
accuracy = accuracy_score(test_labels, predicted_labels)
print("Accuracy:", accuracy)

# Create a figure with subplots
fig, axs = plt.subplots(1, 3, figsize=(12, 4))

# Plot the training data
train_class1 = train_features[train_labels == 'Class1']
train_class2 = train_features[train_labels == 'Class2']
axs[0].scatter(train_class1[:, 0], train_class1[:, 1], c='red', label='Class1')
axs[0].scatter(train_class2[:, 0], train_class2[:, 1], c='blue', label='Class2')
axs[0].set_xlabel('Feature 1')
axs[0].set_ylabel('Feature 2')
axs[0].set_title('Training Data')
axs[0].legend()

# Plot the test data
test_class1 = test_features[test_labels == 'Class1']
test_class2 = test_features[test_labels == 'Class2']
axs[1].scatter(test_class1[:, 0], test_class1[:, 1], c='red', label='Class1')
axs[1].scatter(test_class2[:, 0], test_class2[:, 1], c='blue', label='Class2')
axs[1].set_xlabel('Feature 1')
axs[1].set_ylabel('Feature 2')
axs[1].set_title('Test Data')
axs[1].legend()

# Plot the confusion matrix
confusion_matrix = np.array([[np.sum((test_labels == 'Class1') & (predicted_labels == 'Class1')),
                             np.sum((test_labels == 'Class1') & (predicted_labels == 'Class2'))],
                            [np.sum((test_labels == 'Class2') & (predicted_labels == 'Class1')),
                             np.sum((test_labels == 'Class2') & (predicted_labels == 'Class2'))]])
axs[2].imshow(confusion_matrix, cmap='Blues')
axs[2].set_xticks([0, 1])
axs[2].set_yticks([0, 1])
axs[2].set_xticklabels(['Class1', 'Class2'])
axs[2].set_yticklabels(['Class1', 'Class2'])
axs[2].set_title('Confusion Matrix')

# Adjust spacing between subplots
plt.tight_layout()

# Show the plot
plt.show()
