import sklearn.model_selection as skms
import pandas as pd
import sklearn.linear_model as sklm
import sklearn.svm as sksvm
import sklearn.tree as skt
import sklearn.neighbors as skn
import sklearn.discriminant_analysis as skda
import sklearn.naive_bayes as sknb
import numpy as np

df = pd.read_csv('data.csv')
X = df.drop(labels=['id', 'target'], axis=1)
y = df['target']

clf = sklm.LogisticRegression()
accuracies = skms.cross_val_score(clf, X, y, cv=5)

print('LogisticRegression')
print('Accuracies:', accuracies)
print('Mean accuracy:', np.mean(accuracies))

sv = sksvm.SVC()
accuracies = skms.cross_val_score(sv, X, y, cv=5)

print('SVC')
print('Accuracies:', accuracies)
print('Mean accuracy:', np.mean(accuracies))

tree = skt.DecisionTreeClassifier()
accuracies = skms.cross_val_score(tree, X, y, cv=5)

print('DecisionTreeClassifier')
print('Accuracies:', accuracies)
print('Mean accuracy:', np.mean(accuracies))

knn = skn.KNeighborsClassifier()
accuracies = skms.cross_val_score(knn, X, y, cv=5)

print('KNeighborsClassifier')
print('Accuracies:', accuracies)
print('Mean accuracy:', np.mean(accuracies))

perceptron = sklm.Perceptron()
accuracies = skms.cross_val_score(perceptron, X, y, cv=5)

print('Perceptron')
print('Accuracies:', accuracies)
print('Mean accuracy:', np.mean(accuracies))

qdc = skda.QuadraticDiscriminantAnalysis()
accuracies = skms.cross_val_score(qdc, X, y, cv=5)

print('QuadraticDiscriminantAnalysis')
print('Accuracies:', accuracies)
print('Mean accuracy:', np.mean(accuracies))

nb = sknb.GaussianNB()
accuracies = skms.cross_val_score(nb, X, y, cv=5)

print('GaussianNB')
print('Accuracies:', accuracies)
print('Mean accuracy:', np.mean(accuracies))