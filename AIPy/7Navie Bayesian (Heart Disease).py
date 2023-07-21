import pandas as pd
import numpy as np
from pgmpy.models import BayesianModel
from pgmpy.estimators import MaximumLikelihoodEstimator, BayesianEstimator
from pgmpy.inference import VariableElimination

# Load the heart disease dataset
data = pd.read_csv('./data/heart_disease.csv')

# Define the Bayesian network structure
model = BayesianModel([
    ('age', 'heartdisease'),
    ('sex', 'heartdisease'),
    ('exang', 'heartdisease'),
    ('cp', 'heartdisease'),
    ('heartdisease', 'restecg'),
    ('heartdisease', 'chol')
])

# Fit the data to estimate the model parameters using Maximum Likelihood Estimation
model.fit(data, estimator=MaximumLikelihoodEstimator)

# Perform Bayesian parameter estimation for the model
estimator = BayesianEstimator(model, data)

# Print the learned model parameters (CPDs)
for cpd in model.get_cpds():
    print(cpd)

# Create an inference object
infer = VariableElimination(model)

# Query the model to perform diagnosis
query = infer.query(variables=['heartdisease'], evidence={
    'age': 28,
    'sex': 'male',
    'exang': 'no',
    'cp': 'non-anginal',
    'restecg': 'normal',
    'chol': 245
})

# Print the diagnosis results
print(query['heartdisease'])
