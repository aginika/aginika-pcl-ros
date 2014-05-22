#!/usr/bin/env python
import numpy as np
from sklearn.cross_validation import cross_val_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.tree import DecisionTreeClassifier


#file open
data_x = [];
for l in open('normal_dots5.txt').readlines():
    double_strings = l.split(",");
    data_x.append(map(lambda x: float(x), double_strings))

#back1
#data_y = [0] * 2105 + [1] * int(len(data_x) -2105)
data_y = [0] * 5328 + [1] * int(len(data_x) -5328)
#data_y = [0] * 3142 + [1] * int(len(data_x) -3142)
#data_y = [0] * 3352 + [1] * int(len(data_x) -3352)
#data_y = [0] * 1098 + [1] * int(len(data_x) -1098)

high_score = 0
high_estimators=0
high_depth=0
high_features=0
for n_estimators_val in range(100, 2000, 50):
    for max_features_val in range(2, 10):
        for max_depth_val in range(1,50):
            clf = RandomForestClassifier(n_estimators=n_estimators_val, max_features=max_features_val, max_depth=max_depth_val, min_samples_split=1, random_state=0)
            scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
#            print "random foreset : "
#            print scores.mean()
            if scores.mean() > high_score:
                high_score = scores.mean()
                print high_score
                print n_estimators_val," ",max_depth_val," ",max_features_val
                high_estimators = n_estimators_val
                high_depth = max_depth_val
                high_features = max_features_val
        clf = RandomForestClassifier(n_estimators=n_estimators_val, max_features=max_features_val, max_depth=None, min_samples_split=1, random_state=0)
        scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
#        print "random foreset : "
#        print scores.mean()
        if scores.mean() > high_score:
            high_score = scores.mean()
            print high_score
            print n_estimators_val," ",max_depth_val," ",max_features_val
            high_estimators = n_estimators_val
            high_depth = max_depth_val
            high_features = max_features_val

# clf = RandomForestClassifier(n_estimators=250, max_features=7, max_depth=9, min_samples_split=1, random_state=0)
# scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
# print "random foreset : "
# print scores.mean()




# clf = DecisionTreeClassifier(max_depth=None, min_samples_split=1, random_state=0)
# scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
# print "decision tree : "
# print  scores.mean()

# clf = ExtraTreesClassifier(n_estimators=100, max_depth=None,  min_samples_split=1, random_state=0)
# scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
# print "extra tree : "
# print  scores.mean()

