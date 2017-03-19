import numpy as np
from sklearn.preprocessing import StandardScaler

car_features = [[1,2,3],[3,3,3]]
notcar_features = [[4,5,6],[6,6,6]]

y = np.hstack((np.ones(len(car_features)), 
              np.zeros(len(notcar_features))))

# Create an array stack of feature vectors
X = np.vstack((car_features, notcar_features)).astype(np.float64)
# Fit a per-column scaler
X_scaler = StandardScaler().fit(X)
# Apply the scaler to X
scaled_X = X_scaler.transform(X)

print(X)
print(y)
print(scaled_X)
