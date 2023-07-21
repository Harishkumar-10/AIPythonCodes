import numpy as np
import pandas as pd

# Generate sample data
np.random.seed(0)
n_samples = 100
n_features = 2
X = np.random.randn(n_samples, n_features)

# Save data to CSV file
df = pd.DataFrame(X, columns=['Feature 1', 'Feature 2'])
df.to_csv('data.csv', index=False)
