#! /usr/bin/env python3

import IPython

import numpy as np
import matplotlib.pyplot as plt
from sklearn import preprocessing
from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report
from sklearn.svm import SVC, LinearSVC
from smmap_jupyter import classification_data


data, metadata = classification_data.load_data(aggregate_data=False)
X_train, X_test, Y_train, Y_test, metadata_train, metadata_test, field_names = \
    classification_data.preprocess(data, metadata, max_train_datapoints=None, max_test_datapoints=None)
scaler = preprocessing.MinMaxScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

IPython.embed()

# classifier = SVC(C=10.0, kernel='linear', class_weight='balanced')
# classifier.fit(X_train_scaled, Y_train)
# classifier = LinearSVC(penalty='l1', loss='squared_hinge', dual=False, C=1000.0, class_weight='balanced', max_iter=1000000)
# classifier.fit(X_train_scaled, Y_train)

print("Fitting data ... ", end="")
classifier = SVC(C=1000.0, kernel='rbf', class_weight='balanced', gamma='scale')
classifier.fit(X_train_scaled, Y_train)
print("done. Using", len(classifier.support_), "support vectors.")

classification_data.write_min_max_scaler(scaler, str(len(field_names)) + "feature.scaler")
classification_data.libsvm_write_model(classifier, str(len(field_names)) + "feature.model", X_train_scaled.var())

print("Full training set scores:")
Y_train_predict = classifier.predict(X_train_scaled)
print(classification_report(Y_train, Y_train_predict))
print("Confusion matrix train:")
print(confusion_matrix(Y_train, Y_train_predict))
print()
print("Full testing set scores:")
Y_test_predict = classifier.predict(X_test_scaled)
print(classification_report(Y_test, Y_test_predict))
print("Confusion matrix test:")
print(confusion_matrix(Y_test, Y_test_predict))


# param_grid = [
#     #     {'C': [1.0, 10.0, 100.0, 1000.0], 'kernel': ['linear']},
#     #     {'C': [1.0, 10.0, 100.0, 1000.0], 'kernel': ['rbf'], 'gamma': [0.001, 0.0001]},
#     {'C': [1.0, 10.0, 100.0, 1000.0], 'kernel': ['linear'], 'class_weight': ['balanced']},
#     #     {'C': [1.0, 10.0, 100.0, 1000.0], 'kernel': ['rbf'],    'class_weight': ['balanced'], 'gamma': ['auto']},
# ]

# scores = ['precision', 'recall']

# for score in scores:
#     print("Tuning hyper-parameters for %s ... " % score)
#     print()
#
#     classifier = GridSearchCV(SVC(), param_grid, n_jobs=-1, cv=4, scoring="%s_macro" % score)
#     classifier.fit(X_train_scaled, Y_train)
#
#     print("Best parameter set found on development set:")
#     print()
#     print(classifier.best_params_)
#     print()
#     print("Grid scores on development set:")
#     print()
#     means = classifier.cv_results_['mean_test_score']
#     stds = classifier.cv_results_['std_test_score']
#     for mean, std, params in zip(means, stds, classifier.cv_results_['params']):
#         print("%0.3f (+/-%0.03f) for %r" % (mean, std * 2, params))
#     print()
#
#     print("Detailed classification report:")
#     print()
#     print("The model is trained on the full training set.")
#     print()
#     print("Full training set scores:")
#     Y_train_predict = classifier.predict(X_train_scaled)
#     print(classification_report(Y_train, Y_train_predict))
#     print()
#     print("Full testing set scores:")
#     Y_test_predict = classifier.predict(X_test_scaled)
#     print(classification_report(Y_test, Y_test_predict))
#     print()
#     print()
#
#     cm_train = confusion_matrix(Y_train, Y_train_predict)
#     cm_test = confusion_matrix(Y_test, Y_test_predict)
#     print("Confusion matrix train:")
#     print(cm_train)
#     print("Confusion matrix test:")
#     print(cm_test)

IPython.embed()

# dist_est = GradientBoostingRegressor(n_estimators=100, learning_rate=0.1,
#                                      max_depth=1, random_state=0, loss='ls').fit(X_train, Y_train[:, 0])
# print "Mean squared error: ", mean_squared_error(Y_test[:, 0], dist_est.predict(X_test))
# print "Feature importance: ", dist_est.feature_importances_
#
# foh_est = GradientBoostingRegressor(n_estimators=100, learning_rate=0.1,
#                                     max_depth=1, random_state=0, loss='ls').fit(X_train, Y_train[:, 1])
# print "Mean squared error: ", mean_squared_error(Y_test[:, 1], foh_est.predict(X_test))
# print "Feature importance: ", foh_est.feature_importances_
#
#
# dist_predict = dist_est.predict(X_test)
# plt.scatter(dist_predict, Y_data[:, 0])
# plt.xlabel('Predicted dist')
# plt.ylabel('True dist')
# plt.show()
