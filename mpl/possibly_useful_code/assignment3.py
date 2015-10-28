import timeit
import experiments
import birrt_experiments

def main():
    # print 'RUNNING RRT EXPERIMENTS'
    # print 'polygonal'
    # runExperiment(experiments.experiment1, False, False)
    # print 'links'
    # runExperiment(experiments.experiment2, False, False)
    
    # print 'RUNNING BiRRT EXPERIMENTS'
    # print 'polygonal'
    # runExperiment(birrt_experiments.experiment1, False, False)
    # print 'links'
    # runExperiment(birrt_experiments.experiment2, False, False)

    # print 'RUNNING BiRRT EXPERIMENTS WITH PRUNING'
    # print 'polygonal'
    # runExperiment(birrt_experiments.experiment1, False, True)
    # print 'links'
    # runExperiment(birrt_experiments.experiment2, False, True)
    print 'polygonal with more than one avenue'
    runExperiment(birrt_experiments.experiment3, False, True)


def runExperiment(experiment, shouldDraw, shouldSmooth):
    experimentName = experiment.__name__
    start_time = timeit.default_timer()
    pathLength = experiment(shouldDraw = shouldDraw, shouldSmooth = shouldSmooth)
    elapsed = timeit.default_timer() - start_time
    print "Took {0} seconds".format(elapsed)
    print 'pathLength', pathLength

if __name__ == '__main__':
    main()