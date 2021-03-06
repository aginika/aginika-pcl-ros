#!/usr/bin/env python
import numpy as np
from sklearn.cross_validation import cross_val_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.tree import DecisionTreeClassifier
import glob

non_clothes_directories=""
clothes_directories=""

class RandomForestLearner:
    def __init__(self):
        self.file_name_hint = "*.txt"
        self.estimators = 350
        self.features = 7

    def getFiles(self, directory):
        return glob.glob( directory + self.file_name_hint)

    def getDatas(self, directory):
        file_names = self.getFiles(directory)
        data_list = []
        counter = 0
        for file_name in file_names:
            rospy.loginfo("Reading... " + str(counter) + "/" + str(len(file_names)) + file_name );
            for l in open(file_name).readlines():
                double_strings = l.split(",")
                data_list.append(map(lambda x: float(x), double_strings[:len(double_strings) - 1]))

            counter += 1
        return data_list

    def setupBothClass(self, non_clothes_directory, clothes_directory):
        self.non_clothes_directory = non_clothes_directory
        self.clothes_directory = clothes_directory
        data_x_non_clothes = self.getDatas(non_clothes_directory)
        data_y_non_clothes = [1] * int(len(data_x_non_clothes))
        rospy.loginfo("Data non_clothes setup done" + str(len(data_x_non_clothes)))

        data_x_clothes = self.getDatas(clothes_directory)
        data_y_clothes = [0] * int(len(data_x_clothes))
        rospy.loginfo("Data clothes setup done" + str(len(data_x_clothes)))

        self.data_x_con = data_x_clothes + data_x_non_clothes
        self.data_y_con = data_y_clothes + data_y_non_clothes

    def calculate(self):
        import os
        home = os.environ['HOME']
        import time
        from datetime import datetime
        target_path = home + "/random_forest_result/" + self.filename + str(int(time.mktime(datetime.now().timetuple())))
        f = open(target_path)
        f.write("Target Directory is below")
        f.write("    "+self.non_clothes_directory)
        f.write("    "+self.clothes_directory)

        clf = RandomForestClassifier(n_estimators=self.estimators, max_features=self.features, max_depth=None, min_split=1, random_state=0)
        scores = cross_val_score(clf, np.array(self.data_x_con), np.array(self.data_y_con))
        f.write("random foreset result: "+ str(scores.mean()))
        f.write("Parmas:")
        f.write("    estimators : "+str(self.estimators))
        f.write("    features   : "+str(self.features))
        f.close()
        print scores.mean()

    def save_clf(self):
        clf = RandomForestClassifier(n_estimators=250, max_features=7, max_depth=29, min_split=1, random_state=0)
        clf.fit(data_x, data_y)
        from sklearn.externals import joblib
        joblib.dump(clf, "/tmp/")


    def set_file_name(self, filename):
        self.filename = filename

    def set_estimators(self, estimators):
        self.estimators = estimators

    def set_features(self, features):
        self.features = features

if __name__ == "__main__":
    import rospy
    import imp
    import sys

    argvs = sys.argv
    argc = len(argvs)

    rospy.loginfo("Random Forest Before")
    if  argc < 6:
        exit()

    rospy.init_node('leaner', anonymous=True)
    imp.find_module("aginika_pcl_ros")
    rospy.loginfo("Random Forest Setup")
    classifier = RandomForestLearner()
    rospy.loginfo("Random Forest Setup Both Class")
    classifier.setupBothClass(argvs[1], argvs[2])
    classifier.set_file_name(argvs[3])
    classifier.set_estimators(int(argvs[4]))
    classifier.set_features(int(argvs[5]))
    rospy.loginfo("Random Forest Calculate Start")
    classifier.calculate()
    rospy.loginfo("Random Forest Calculate end")

# from os import listdir
# from os.path import isfile, join
# non_clothes_onlyfiles = [ non_clothes_directories+f for f in listdir(non_clothes_directories) if isfile(join(non_clothes_directories,f)) ]
# clothes_onlyfiles = [ clothes_directories+f for f in listdir(clothes_directories) if isfile(join(clothes_directories,f)) ]


# #non clothes file open
# data_x_non_clothes = [];
# for file_name in non_clothes_onlyfiles:
#     for l in open(file_name).readlines():
#         double_strings = l.split(",")
#         data_x_non_clothes.append(map(lambda x: float(x), double_strings[:len(double_strings) - 1]))

# #back1
# data_y_non_clothes = [1] * int(len(data_x_non_clothes))

# #clothes file open
# data_x_clothes = [];
# for file_name in clothes_onlyfiles:
#     for l in open(file_name).readlines():
#         double_strings = l.split(",")
#         data_x_clothes.append(map(lambda x: float(x), double_strings[:len(double_strings) - 1]))

# #back1
# data_y_clothes = [0] * int(len(data_x_clothes))

#############
#concatenate
#############

# data_x_con = data_x_clothes + data_x_non_clothes
# data_y_con = data_y_clothes + data_y_non_clothes

# high_score = 0
# high_estimators=0
# high_depth=0
# high_features=0
# for n_estimators_val in range(100, 2000, 50):
#     for max_features_val in range(2, 10):
#         for max_depth_val in range(1,50):
#             clf = RandomForestClassifier(n_estimators=n_estimators_val, max_features=max_features_val, max_depth=max_depth_val, min_samples_split=1, random_state=0)
#             scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
#             print "random foreset : "
#             print scores.mean()
#             if scores.mean() > high_score:
#                 high_score = scores.mean()
#                 print high_score
#                 print n_estimators_val," ",max_depth_val," ",max_features_val
#                 high_estimators = n_estimators_val
#                 high_depth = max_depth_val
#                 high_features = max_features_val
#         clf = RandomForestClassifier(n_estimators=n_estimators_val, max_features=max_features_val, max_depth=None, min_samples_split=1, random_state=0)
#         scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
#         print "random foreset : "
#         print scores.mean()
#         if scores.mean() > high_score:
#             high_score = scores.mean()
#             print high_score
#             print n_estimators_val," ",max_depth_val," ",max_features_val
#             high_estimators = n_estimators_val
#             high_depth = max_depth_val
#             high_features = max_features_val




# clf = DecisionTreeClassifier(max_depth=None, min_samples_split=1, random_state=0)
# scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
# print "decision tree : "
# print  scores.mean()

# clf = ExtraTreesClassifier(n_estimators=100, max_depth=None,  min_samples_split=1, random_state=0)
# scores = cross_val_score(clf, np.array(data_x), np.array(data_y))
# print "extra tree : "
# print  scores.mean()

