from sklearn.datasets import make_classification
import pandas as pd

# Generate a synthetic dataset
X, y = make_classification(
    n_samples=100,  # Number of samples
    n_features=5,  # Number of features
    n_informative=3,  # Number of informative features
    n_classes=2,  # Number of classes
    random_state=42
)

# Create a pandas DataFrame from the generated data
df = pd.DataFrame(X, columns=[f"feature_{i}" for i in range(X.shape[1])])
df["target"] = y

# Save the DataFrame to a CSV file
df.to_csv("monk2.csv", index=False)
