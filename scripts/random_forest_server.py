#!/usr/bin/env python
import rospy
import numpy as np
from sklearn.cross_validation import cross_val_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.ensemble import ExtraTreesClassifier
from aginika_pcl_ros.srv import *
from  sklearn.externals import joblib


class RandomForestServer:
    def __init__(self, clf):
        self.clf = clf
        s = rospy.Service('predict', PredictData, self.predictRequest)

    @classmethod
    def initWithData(cls, data_x, data_y):
        if len(data_x) != len(data_y):
            print "Lenght of datas are different"
            exit()
        rospy.loginfo("InitWithData please wait..")
        clf = RandomForestClassifier(n_estimators=250, max_features=7, max_depth=29, min_samples_split=1, random_state=0)
        clf.fit(data_x, data_y)
        return cls(clf)

    @classmethod
    def initWithFileModel(cls, filename):
        rospy.loginfo("InitWithFileModel with%s please wait.."%filename)
        clf = joblib.load(filename)
        return cls(clf)

    #Return predict result
    def predictRequest(self, req):
        ret = int(self.clf.predict(req.data))
        rospy.loginfo("Get service Return Index %d!", ret)
        return PredictDataResponse(ret)

    #Run random forest
    def run(self):
        rospy.loginfo("RandomForestServer is running!")
#        from sklearn.externals import joblib
#        joblib.dump(clf, "/tmp/")
        rospy.spin()




if __name__ == "__main__":
    rospy.init_node('random_forest_cloth_classifier')

    try:
        private_filename = rospy.get_param('~random_forest_train_file')
    except KeyError:
        rospy.logerr("Train File is not Set. Set train_data file or tree model file as ~random_forest_train_file.")
        exit()

    if private_filename.endswith("pkl"):
        node = RandomForestServer.initWithFileModel(private_filename)
    else:
        try:
            private_filename2 = rospy.get_param('~random_forest_train_class_file')

            data_x = []
            data_y = []
            for l in open(private_filename).readlines():
                float_strings = l.split(",");
                data_x.append(map(lambda x: float(x), float_strings))

            for l in open(private_filename2).readlines():
                data_y.append(float(l))

            #build servece server
            node = RandomForestServer.initWithData(np.array(data_x), np.array(data_y))

        except KeyError:
            rospy.logerr("Train Class File is not Set. Set train_data file or tree model file.")
            rospy.logerr("Or Did you expect  Extension to be pkl?.")
            exit()


    #run
    node.run()
