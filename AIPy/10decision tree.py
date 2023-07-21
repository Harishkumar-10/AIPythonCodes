from sklearn.datasets import load_wine
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier, plot_tree
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt

# Load the Monk2 dataset
monk2_data = load_wine()

# Separate the features (X) and target variable (y)
X = monk2_data.data
y = monk2_data.target

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Create an unpruned decision tree classifier
unpruned_tree = DecisionTreeClassifier(random_state=42)
unpruned_tree.fit(X_train, y_train)

# Evaluate the unpruned decision tree on the test set
unpruned_predictions = unpruned_tree.predict(X_test)
unpruned_accuracy = accuracy_score(y_test, unpruned_predictions)

# Create a pruned decision tree classifier
pruned_tree = DecisionTreeClassifier(ccp_alpha=0.015, random_state=42)  # Adjust the ccp_alpha value for pruning
pruned_tree.fit(X_train, y_train)

# Evaluate the pruned decision tree on the test set
pruned_predictions = pruned_tree.predict(X_test)
pruned_accuracy = accuracy_score(y_test, pruned_predictions)

# Print the accuracies of the unpruned and pruned decision trees
print("Unpruned Accuracy:", unpruned_accuracy)
print("Pruned Accuracy:", pruned_accuracy)

# Visualize the structure of the unpruned decision tree
plt.figure(figsize=(12, 6))
plt.title("Unpruned Decision Tree")
_ = plot_tree(unpruned_tree, filled=True)

# Visualize the structure of the pruned decision tree
plt.figure(figsize=(12, 6))
plt.title("Pruned Decision Tree")
_ = plot_tree(pruned_tree, filled=True)

plt.show()
