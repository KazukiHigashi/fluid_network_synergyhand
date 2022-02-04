import numpy as np
import pickle

def forward_propagation(x):
    with open("weights1215_py2", "rb") as f:
        w = pickle.load(f)
    def norm(x):
        return (x - w[0]) / np.sqrt(w[1])
    def relu(x):
        return np.maximum(0, x)
    W12 = w[3]
    b1 = w[4]
    W23 = w[5]
    b2 = w[6]
    W34 = w[7]
    b3 = w[8]
    
    nx = norm(x)

    print(nx)

    if len(nx) == 1:
        u = np.dot(W34.T, relu(np.dot(W23.T, relu(np.dot(W12.T, nx[0])+b1))+b2))+b3
    else:
        u = [np.dot(W34.T, relu(np.dot(W23.T, relu(np.dot(W12.T, nxx)+b1))+b2))+b3 for nxx in nx]
    return np.array(u)

test_features = np.array([[191., 0.36992, 10.38274, 0., 1., 0.],[191., 0.36992, 10.38274, 0., 1., 0.]])
test_features = np.array([[191., 0.36992, 10.38274, 0., 1., 0.]])


print(forward_propagation(test_features))
