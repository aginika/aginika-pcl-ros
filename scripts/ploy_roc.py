import numpy as np
import pylab as pl
from sklearn import svm, datasets
from sklearn.utils import shuffle
from sklearn.metrics import roc_curve, auc
import glob
from sklearn.cross_validation import cross_val_score
from sklearn.ensemble import RandomForestClassifier

color=["r","g","b","c","m","y","k"]
base_path="/home/inagaki/feature_result/"
target_datas=["random_region_FPFH_Average",
              "random_square_FPFH_Average",
              "random_region_FPFH",
              "random_region_DOT",
              "random_region_DIFFDOT",
              "random_region_GRAVDOT"
              ]

def getFiles(directory):
    return glob.glob( directory + "*.txt")


def getDatas(directory):
    file_names = getFiles(directory)
    data_list = []
    counter = 0
    for file_name in file_names:
        rospy.loginfo("Reading... " + str(counter) + "/" + str(len(file_names)) + file_name );
        for l in open(file_name).readlines():
            double_strings = l.split(",")
            data_list.append(map(lambda x: float(x), double_strings[:len(double_strings) - 1]))
        counter += 1
    return data_list

if __name__=="__main__":
    import rospy
    import imp
    import sys

    print "Here2"
    # Plot ROC curve
    pl.clf()

    for i in range(0, int(len(target_datas))):
        print "target_datas " + target_datas[i]

        x_non_clothes_data = getDatas(base_path+target_datas[i]+"/non_clothes/")
        x_clothes_data = getDatas(base_path+target_datas[i]+"/clothes/")
        y_non_clothes_data = [1] * int(len(x_non_clothes_data))
        y_clothes_data = [0] * int(len(x_clothes_data))

        print "Here3"
        x_con_data = x_clothes_data + x_non_clothes_data
        y_con_data = y_clothes_data + y_non_clothes_data

        print "Here4"
        random_state = np.random.RandomState(0)

        X, y = shuffle(x_con_data, y_con_data, random_state=random_state)

        half = int(len(x_con_data) / 2)
        X_train, X_test = X[:half], X[half:]
        y_train, y_test = y[:half], y[half:]

        print "Here5"
        classifier = RandomForestClassifier(n_estimators=250, max_features=7, max_depth=29, min_samples_split=1, random_state=0)
        print "Here6"
        probas_ = classifier.fit(X_train, y_train).predict_proba(X_test)
        print "Here7"
        # Compute ROC curve and area the curve
        fpr, tpr, thresholds = roc_curve(y_test, probas_[:, 1])
        roc_auc = auc(fpr, tpr)
        print("Area under the ROC curve : %f" % roc_auc)

        pl.plot(fpr, tpr, color[i], label= target_datas[i]+' (area = %0.2f)' % roc_auc, lw=4)

    pl.plot([0, 1], [0, 1], 'k--')
    pl.xlim([0.0, 1.0])
    pl.ylim([0.0, 1.0])
    pl.xlabel('false-positive Rate', fontweight="bold")
    pl.ylabel('true-positive Rate', fontweight="bold")
    pl.title('ROC of each features')
    pl.legend(loc="lower right", prop={'size' : 12, 'weight' : "bold"})
    pl.show()
