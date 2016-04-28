import numpy as np
import matplotlib.pyplot as plt
import pdb

algorithmNumberToStrategyMap = {0: 'MCR', 1: 'RRT', 2: 'BiRRT', 3: 'TLP MCR', 4: 'Greedy Removal', 5: 'Probabilistic Removal',
                                    6: 'Ignore Non-Permanent', 7: 'Repeat Probabilistic Removal',
                                    8: 'Search Then Greedy Removal', 9: 'HPN Like Removal'}

def analyzeHistory(fileName):
    history = readPickledHistory(fileName)
    # plt.ion()
    # print history
    for algorithmNumber in range(10):
        historyForAlgorithm = history[algorithmNumber]
        coverHistory = np.array([x[1] for x in historyForAlgorithm if len(x) == 3])
        # print algorithmNumberToStrategyMap[algorithmNumber], coverHistory
        coverVar = np.var(coverHistory)
        successTimeHistory = np.array([x[2] for x in historyForAlgorithm if len(x) == 3])
        successTimeVar = np.var(successTimeHistory)
        failureTimeHistory = np.array([x[0] for x in historyForAlgorithm if len(x) == 1])
        failureTimeVar = np.var(failureTimeHistory)
        fig = plt.figure(algorithmNumber, figsize=(8,10))
        fig = plt.figure(algorithmNumber)
        fig.subplots_adjust(hspace = 0.5)
        if len(coverHistory) != 0:
            plt.subplot(311)
            n, bins, patches = plt.hist(coverHistory, bins = range(int(max(coverHistory) + 2)), normed = 1, facecolor='green', alpha=0.5, align = 'left')
            print 'bins', bins
            plt.title("Cover Histogram for " + fileName + " for algorithm " + algorithmNumberToStrategyMap[algorithmNumber])
        if len(successTimeHistory) != 0:
            plt.subplot(312)
            n, bins, patches = plt.hist(successTimeHistory, bins = 10, normed = True, facecolor='green', alpha= 0.5)
            plt.title("Success Time Histogram for " + fileName + " for algorithm " + algorithmNumberToStrategyMap[algorithmNumber])
        if len((failureTimeHistory) != 0):
            plt.subplot(313)
            plt.hist(failureTimeHistory, facecolor='green', alpha= 0.5)
            plt.title("Failure Time Histogram for " + fileName + " for algorithm " + algorithmNumberToStrategyMap[algorithmNumber])
        pdb.set_trace()
        plt.show()
    

def readPickledHistory(fileName):
    import cPickle, gzip
    f = gzip.open("test_results/history/" + fileName + ".pkl.gz", 'rb')
    history = cPickle.load(f)
    f.close()
    return history
    

fileNames = ['SomeObstaclesFeasibleWorld', 'ManyObstaclesWorld', 'TwoSodaHandle', 'ClutteredWorld', 'TopLightClutteredWorld']

analyzeHistory(fileNames[0])
# analyzeHistory(fileNames[1])
# analyzeHistory(fileNames[2])
# analyzeHistory(fileNames[3])


# for fileName in fileNames:
    # analyzeHistory(fileName)
# 